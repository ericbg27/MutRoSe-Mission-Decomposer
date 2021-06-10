#include "xmloutputgenerator.hpp"

#include <boost/property_tree/xml_parser.hpp>

#include "../validmissiongenerator/validmissiongenerator.hpp"

using namespace std;

/*
    Function: generate_instances_output
    Objective: This function generates the XML output with all task instances, constraints, actions and valid mission
    decompositions. By valid we mean that are viable given the knowledge that we have about the world. Robot-related
    predicates are not resolved and are left as attributes to be evaluated

    @ Input 1: The semantic mappings vector
	@ Input 2: The sorts map, where we have our objects
	@ Input 3: The sort definitions
	@ Input 4: The predicates definitions
    @ Input 5: The mapping of Goal Model variables and their values/types
    @ Output: Void. The output file is generated in the given relative path
*/
void XMLOutputGenerator::generate_instances_output(vector<SemanticMapping> semantic_mapping, map<string,set<string>> sorts, vector<sort_definition> sort_definitions, 
                                                    vector<predicate_definition> predicate_definitions, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map) {
    ConstraintManager constraint_generator(gm, mission_decomposition, verbose);
    vector<Constraint> mission_constraints = constraint_generator.generate_mission_constraints();
    
    // With the final constraints and the mission decomposition graph we generate our output
    pt::ptree output_file;

    ValidMissionGenerator valid_missions_generator(mission_decomposition, gm, mission_constraints, world_state, world_state_functions, semantic_mapping, gm_var_map, verbose);
    pair<vector<vector<pair<int,ATNode>>>,set<Decomposition>> valid_mission_decompositions_and_expanded_decompositions = valid_missions_generator.generate_valid_mission_decompositions();

    vector<vector<pair<int,ATNode>>> valid_mission_decompositions = valid_mission_decompositions_and_expanded_decompositions.first;
    set<Decomposition> expanded_decompositions = valid_mission_decompositions_and_expanded_decompositions.second;

    vector<Decomposition> task_instances;
    map<string,task> actions;

    auto nodes = vertices(mission_decomposition);

    int graph_size = *nodes.second - *nodes.first;

    for(int index = 0;index < graph_size;index++) {
        if(mission_decomposition[index].node_type == DECOMPOSITION) {
            Decomposition d = std::get<Decomposition>(mission_decomposition[index].content);

            set<Decomposition>::iterator d_it = expanded_decompositions.find(d);
            if(d_it != expanded_decompositions.end()) {
                d = *d_it;
            }

            for(task a : d.path.decomposition) {
                if(actions.find(a.name) == actions.end() && a.name.find(method_precondition_action_name) == string::npos) {
                    actions[a.name] = a;
                }
            }

            task_instances.push_back(d);
        }
    }

    output_actions(output_file, actions);

    map<string,string> task_id_map = output_tasks(output_file, task_instances, semantic_mapping);

    output_constraints(output_file, mission_constraints, task_id_map);

    output_mission_decompositions(output_file, valid_mission_decompositions, task_id_map);

    string file_type = output.second;
    std::transform(file_type.begin(),file_type.end(),file_type.begin(),::toupper);

    pt::xml_writer_settings<string> settings(' ',4);
    pt::write_xml(output.first, output_file, std::locale(), settings);
}

/*
    Function: output_actions
    Objective: Insert Actions into the XML Output file

    @ Input 1: A reference to the output file ptree object
    @ Input 2: The map of actions
    @ Output: Void. The output file ptree oject is filled

    NOTES: -> Fields:
            - Name
            - Capabilities
*/
void XMLOutputGenerator::output_actions(pt::ptree& output_file, map<string,task> actions) {
    output_file.put("actions","");

    map<string,string>actions_id_map;

    int actions_counter = 0;
    map<string,task>::iterator actions_it;
    for(actions_it = actions.begin();actions_it != actions.end();++actions_it) {
        string action_name = "actions.action" + to_string(actions_counter);
        output_file.add(action_name,"");

        string action_attr;

        action_attr = action_name + ".name";
        output_file.put(action_attr,actions_it->first);

        action_attr = action_name + ".capabilities.<xmlattr>.number";
        output_file.put(action_attr,actions_it->second.required_capabilities.size());

        int capability_counter = 0;
        for(string cap : actions_it->second.required_capabilities) {
            string cap_attr = action_name + ".capabilities.capability" + to_string(capability_counter);
            output_file.put(cap_attr,cap);

            capability_counter++;
        }

        actions_counter++;
        actions_id_map[actions_it->first] = action_name;
    }    
}

/*
    Function: output_tasks
    Objective: Insert Task Decompositions into the XML Output file

    @ Input 1: A reference to the output file ptree object
    @ Input 2: The vector of task_decompositions
    @ Input 3: The vector of semantic mappings
    @ Output: A map with the tasks XML ID's. Also, the output file ptree oject is filled

    NOTES: -> Fields:
            - ID
            - Name
            - Location
            - Preconditions
            - Effects
            - Decomposition (Into actions)
           -> Each instance corresponds to a decomposition of an AT
*/
map<string,string> XMLOutputGenerator::output_tasks(pt::ptree& output_file, vector<Decomposition> task_instances, vector<SemanticMapping> semantic_mapping) {
    map<string,string> task_id_map;

    output_file.put("tasks","");

    int task_counter = 0;
    for(Decomposition instance : task_instances) {
        string task_name = "tasks.task" + to_string(task_counter);
        output_file.add(task_name,"");

        string task_attr;

        task_attr = task_name + ".id";
        output_file.put(task_attr,instance.id);

        task_attr = task_name + ".name";
        output_file.put(task_attr,instance.at.name);

        task_attr = task_name + ".locations";
        if(holds_alternative<vector<string>>(instance.at.location.first)) {
            vector<string> locations = get<vector<string>>(instance.at.location.first);
            int loc_counter = 0;
            for(string loc : locations) {
                string aux = task_attr + ".t" + to_string(loc_counter);
                output_file.put(aux,loc);
                
                loc_counter++;
            }
        } else {
            string location = get<string>(instance.at.location.first);
            task_attr += ".t0";
            output_file.put(task_attr,location);
        }

        task_attr = task_name + ".robots_num.<xmlattr>.fixed";
        if(instance.at.fixed_robot_num) {
            output_file.put(task_attr,"True");
            
            task_attr = task_name + ".robots_num.<xmlattr>.num";
            output_file.put(task_attr,get<int>(instance.at.robot_num));
        } else {
            output_file.put(task_attr,"False");

            pair<int,int> robot_range = get<pair<int,int>>(instance.at.robot_num);

            task_attr = task_name + ".robots_num.<xmlattr>.min";
            output_file.put(task_attr,to_string(robot_range.first));

            task_attr = task_name + ".robots_num.<xmlattr>.max";
            output_file.put(task_attr,to_string(robot_range.second));
        }

        task_attr = task_name + ".preconditions";

        int prec_index = 0;
        for(auto prec : instance.prec) {
            map<string,string> task_vars;
            for(pair<string,string> args : instance.at.at.vars) {
                task_vars[args.first] = args.second;
            }

            pair<SemanticMapping,bool> prec_mapping = find_predicate_mapping(prec,semantic_mapping,sorts,task_vars,sort_definitions);

            if(!prec_mapping.second) {
                string semantic_mapping_error;
                if(holds_alternative<ground_literal>(prec)) {
                    ground_literal p = get<ground_literal>(prec);
                    semantic_mapping_error += "No Semantic Mapping exists for predicate [" + p.predicate + " ";
                    unsigned int index = 0;
                    for(string arg : p.args) {
                        if(index == p.args.size()-1) {
                            semantic_mapping_error += arg + "] ";
                        } else {
                            semantic_mapping_error += arg + " "; 
                        }
                    }
                } else {
                    literal p = get<literal>(prec);
                    semantic_mapping_error = "No Semantic Mapping exists for predicate [" + p.predicate + " ";
                    unsigned int index = 0;
                    for(string arg : p.arguments) {
                        if(index == p.arguments.size()-1) {
                            semantic_mapping_error += arg + "] ";
                        } else {
                            semantic_mapping_error += arg + " "; 
                        }
                    }
                }
                semantic_mapping_error += "] when trying to generate output for task " + instance.id + ": " + instance.at.name;

                throw std::runtime_error(semantic_mapping_error);
            } else {
                /*
                    Here we output the predicate as an attribute in the preconditions attribute of the XML

                    -> For now we only map predicates to attributes. These predicates have only one argument
                */
                string prec_output;

                if(prec_mapping.first.get_mapping_type() == attribute_mapping_type) {  
                    vector<string> arguments;
                    string prec_name;
                    if(holds_alternative<ground_literal>(prec)) {
                        ground_literal p = get<ground_literal>(prec);

                        if(!p.positive) prec_output += "not ";
                        prec_output += p.args.at(0) + ".";
                        arguments = p.args;
                        prec_name = p.predicate;
                    } else {
                        literal p = get<literal>(prec);

                        if(!p.positive) prec_output += "not ";
                        prec_output += p.arguments.at(0) + ".";
                        arguments = p.arguments;
                        prec_name = p.predicate;
                    }
                    prec_output += get<string>(prec_mapping.first.get_prop("name"));

                    vector<string> arg_sorts;
                    for(predicate_definition pred : predicate_definitions) {
                        if(pred.name == prec_name) {
                            arg_sorts = pred.argument_sorts;
                            break;
                        }
                    }


                    string prec_name_xml = task_attr + ".prec" + to_string(prec_index);

                    string prec_attr_xml = prec_name_xml + ".<xmlattr>.vars";
                    string vars_list;
                    for(unsigned int i = 0;i < arguments.size();i++) {
                        if(i == arguments.size()-1) {
                            vars_list += arguments.at(i);
                        } else {
                            vars_list += arguments.at(i) + ",";
                        }
                    }
                    output_file.put(prec_attr_xml,vars_list);

                    prec_attr_xml = prec_name_xml + ".<xmlattr>.var_types";
                    string var_types_list;
                    for(unsigned int i = 0;i < arg_sorts.size();i++) {
                        if(i == arg_sorts.size()-1) {
                            var_types_list += arg_sorts.at(i);
                        } else {
                            var_types_list += arg_sorts.at(i) + ",";
                        }
                    }
                    output_file.put(prec_attr_xml,var_types_list);

                    output_file.put(prec_name_xml,prec_output);
                } else if(prec_mapping.first.get_mapping_type() == ownership_mapping_type) {
                    /*
                        Do we need to output preconditions related to ownership type semantic mappings?
                    */
                }

                prec_index++;
            }
        }

        /*
            For effects we have to verify how we dealt with non ground literals so that we can try to verify the types
        */
        task_attr = task_name + ".effects";

        int eff_index = 0;
        for(auto eff : instance.eff) {
            
            map<string,string> task_vars;
            for(pair<string,string> args : instance.at.at.vars) {
                task_vars[args.first] = args.second;
            }

            pair<SemanticMapping,bool> eff_mapping = find_predicate_mapping(eff,semantic_mapping,sorts,task_vars,sort_definitions);

            if(!eff_mapping.second) {
                string semantic_mapping_error;
                if(holds_alternative<ground_literal>(eff)) {
                    ground_literal e = get<ground_literal>(eff);
                    semantic_mapping_error += "No Semantic Mapping exists for predicate [" + e.predicate + " ";
                    unsigned int index = 0;
                    for(string arg : e.args) {
                        if(index == e.args.size()-1) {
                            semantic_mapping_error += arg + "] ";
                        } else {
                            semantic_mapping_error += arg + " "; 
                        }
                    }
                } else {
                    literal e = get<literal>(eff);
                    semantic_mapping_error += "No Semantic Mapping exists for predicate [" + e.predicate + " ";
                    unsigned int index = 0;
                    for(string arg : e.arguments) {
                        if(index == e.arguments.size()-1) {
                            semantic_mapping_error += arg + "] ";
                        } else {
                            semantic_mapping_error += arg + " "; 
                        }
                    }
                }
                semantic_mapping_error += "when trying to generate output for task " + instance.id + ": " + instance.at.name;
                throw std::runtime_error(semantic_mapping_error);
            } else {
                /*
                    Here we output the predicate as an attribute in the preconditions attribute of the XML

                    -> For now we only map predicates to attributes. These predicates have only one argument
                */
                string eff_output;

                if(eff_mapping.first.get_mapping_type() == "attribute") {   
                    vector<string> arguments;
                    string eff_name;
                    if(holds_alternative<ground_literal>(eff)) {
                        ground_literal e = get<ground_literal>(eff);

                        if(!e.positive) eff_output += "not ";
                        eff_output += e.args.at(0) + ".";
                        arguments = e.args;
                        eff_name = e.predicate;
                    } else {
                        literal e = get<literal>(eff);

                        if(!e.positive) eff_output += "not ";
                        eff_output += e.arguments.at(0) + ".";
                        arguments = e.arguments;
                        eff_name = e.predicate;
                    }
                    eff_output += get<string>(eff_mapping.first.get_prop("name"));

                    vector<string> arg_sorts;
                    for(predicate_definition pred : predicate_definitions) {
                        if(pred.name == eff_name) {
                            arg_sorts = pred.argument_sorts;
                            break;
                        }
                    }

                    string eff_name_xml = task_attr + ".eff" + to_string(eff_index);

                    string eff_attr_xml = eff_name_xml + ".<xmlattr>.vars";
                    string vars_list;
                    for(unsigned int i = 0;i < arguments.size();i++) {
                        if(i == arguments.size()-1) {
                            vars_list += arguments.at(i);
                        } else {
                            vars_list += arguments.at(i) + ",";
                        }
                    }
                    output_file.put(eff_attr_xml,vars_list);

                    eff_attr_xml = eff_name_xml + ".<xmlattr>.var_types";
                    string var_types_list;
                    for(unsigned int i = 0;i < arg_sorts.size();i++) {
                        if(i == arg_sorts.size()-1) {
                            var_types_list += arg_sorts.at(i);
                        } else {
                            var_types_list += arg_sorts.at(i) + ",";
                        }
                    }
                    output_file.put(eff_attr_xml,var_types_list);

                    output_file.put(eff_name_xml,eff_output);
                }

                eff_index++;
            }
        }
        
        /*
            Here we output the list of triggering events
        */
        task_attr = task_name + ".triggering_events";
        int event_counter = 0;
        for(string e : instance.at.triggering_events) {
            string event_id = task_attr + ".event" + to_string(event_counter);
            output_file.put(event_id,e);

            event_counter++;
        }

        /*
            Introducing the decomposition using actions

            -> For now we put action names but we should instantiate actions in the XML first so we can have a map to their id's in the XML file
        */
        task_attr = task_name + ".decomposition";
        int action_counter = 0;
        for(task a : instance.path.decomposition) {
            if(a.name.find(method_precondition_action_name) == string::npos) {
                string action_id = task_attr + ".action" + to_string(action_counter);
                output_file.put(action_id,a.name);

                action_counter++;
            }
        }

        size_t id_separator = task_name.find(".");
        task_id_map[instance.id] = task_name.substr(id_separator+1);
        task_counter++;
    }

    return task_id_map;
}

/*
    Function: output_constraints
    Objective: Insert Constraints into the XML Output file

    @ Input 1: A reference to the output file ptree object
    @ Input 2: The final mission constraints
    @ Input 3: The map of task XML ID's
    @ Output: Void. The output file ptree oject is filled

    NOTES: -> Fields:
            - Type
            - Task Instances
            - Group (Important only if constraint is of NC type)
            - Divisible (Important only if constraint is of NC type)
*/
void XMLOutputGenerator::output_constraints(pt::ptree& output_file, vector<Constraint> mission_constraints, map<string,string> task_id_map) {
    output_file.put("constraints","");
    
    int constraint_counter = 0;
    for(Constraint c : mission_constraints) {
        string constraint_name = "constraints.constraint" + to_string(constraint_counter);

        string constraint_attr = constraint_name + ".type";
        if(c.type == SEQ) {
            output_file.put(constraint_attr,"SEQ");
        } else {
            output_file.put(constraint_attr,"EC");
        }

        constraint_attr = constraint_name + ".task_instances.id0";

        string t_id = get<Decomposition>(c.nodes_involved.first.second.content).id;
        output_file.put(constraint_attr,task_id_map[t_id]);

        constraint_attr = constraint_name + ".task_instances.id1";

        t_id = get<Decomposition>(c.nodes_involved.second.second.content).id;
        output_file.put(constraint_attr,task_id_map[t_id]);

        if(c.type == NC) {
            string attr_value;

            constraint_attr = constraint_name + ".group";
            if(c.group) {
                attr_value = "True";
            } else {
                attr_value = "False";
            }
            output_file.put(constraint_attr,attr_value);

            constraint_attr = constraint_name + ".divisible";
            if(c.divisible) {
                attr_value = "True";
            } else {
                attr_value = "False";
            }
            output_file.put(constraint_attr,attr_value);
        }
        constraint_counter++;
    }
}

/*
    Function: output_mission_decompositions
    Objective: Insert Mission Decompositions into the XML Output file

    @ Input 1: A reference to the output file ptree object
    @ Input 2: The valid mission decompositions
    @ Input 3: The map of task XML ID's
    @ Output: Void. The output file ptree oject is filled

    NOTES: -> Fields:
            - Decomposition
            - Task Instances
*/
void XMLOutputGenerator::output_mission_decompositions(pt::ptree& output_file, vector<vector<pair<int,ATNode>>> valid_mission_decompositions, map<string,string> task_id_map) {
    output_file.put("mission_decompositions","");

    int decomposition_counter = 0;
    for(vector<pair<int,ATNode>> mission_decomposition : valid_mission_decompositions) {
        string decomposition_name = "mission_decompositions.decomposition" + to_string(decomposition_counter);
        int task_counter = 0;
        for(pair<int,ATNode> task : mission_decomposition) {
            string task_name = decomposition_name + ".tasks.t" + to_string(task_counter);

            output_file.put(task_name,task_id_map[get<Decomposition>(task.second.content).id]);
            task_counter++;
        }

        decomposition_counter++;
    }
}