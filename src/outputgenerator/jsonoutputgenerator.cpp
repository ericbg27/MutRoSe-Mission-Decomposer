#include "jsonoutputgenerator.hpp"

#include <boost/property_tree/json_parser.hpp>

#include "../validmissiongenerator/validmissiongenerator.hpp"

using namespace std;

void JSONOutputGenerator::generate_instances_output(vector<SemanticMapping> semantic_mapping, map<string,set<string>> sorts, vector<sort_definition> sort_definitions, vector<predicate_definition> predicate_definitions,
                                                        map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, set<string> robot_related_sorts) {
    ConstraintManager constraint_generator(gm, mission_decomposition, verbose);
    vector<Constraint> mission_constraints = constraint_generator.generate_mission_constraints();
    
    // With the final constraints and the mission decomposition graph we generate our output
    pt::ptree output_file;

    ValidMissionGenerator valid_missions_generator(mission_decomposition, gm, mission_constraints, world_state, world_state_functions, semantic_mapping, gm_var_map, verbose, robot_related_sorts);
    pair<vector<vector<pair<int,ATNode>>>,set<Decomposition>> valid_mission_decompositions_and_expanded_decompositions = valid_missions_generator.generate_valid_mission_decompositions();

    vector<vector<pair<int,ATNode>>> valid_mission_decompositions = valid_mission_decompositions_and_expanded_decompositions.first;
    set<Decomposition> expanded_decompositions = valid_mission_decompositions_and_expanded_decompositions.second;

    vector<pair<Decomposition,pair<bool,bool>>> task_instances;
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

            int parent_index = mission_decomposition[index].parent;

            bool group = mission_decomposition[parent_index].group;
            bool divisible = mission_decomposition[parent_index].divisible;

            task_instances.push_back(make_pair(d,make_pair(group,divisible)));
        }
    }

    output_actions(output_file, actions);

    map<string,string> task_id_map = output_tasks(output_file, task_instances, semantic_mapping);

    output_constraints(output_file, mission_constraints, task_id_map);

    output_mission_decompositions(output_file, valid_mission_decompositions, task_id_map);

    string file_type = output.second;
    std::transform(file_type.begin(),file_type.end(),file_type.begin(),::toupper);

    pt::write_json(output.first, output_file, std::locale());
}

void JSONOutputGenerator::output_actions(pt::ptree& output_file, map<string,task> actions) {
    pt::ptree actions_node;

    map<string,task>::iterator actions_it;
    for(actions_it = actions.begin(); actions_it != actions.end(); ++actions_it) {
        pt::ptree action_node;
        action_node.put("name", actions_it->first);

        pt::ptree capabilities_node;
        for(string cap : actions_it->second.required_capabilities) {
            capabilities_node.put("", cap);
        }
        action_node.add_child("capabilities", capabilities_node);

        actions_node.push_back(std::make_pair("", action_node));
    }

    output_file.add_child("actions", actions_node);
}

map<string,string> JSONOutputGenerator::output_tasks(pt::ptree& output_file, vector<pair<Decomposition,pair<bool,bool>>> task_instances, vector<SemanticMapping> semantic_mapping) {
    map<string,string> task_id_map;

    pt::ptree tasks_node;

    int task_counter = 0;
    for(pair<Decomposition,pair<bool,bool>> instance_info : task_instances) {
        Decomposition instance = instance_info.first;
        
        pt::ptree task_node;

        task_node.put("id", instance.id);
        task_node.put("name", instance.at.name);
        
        pt::ptree locations_node;
        if(holds_alternative<vector<string>>(instance.at.location.first)) {
            vector<string> locations = std::get<vector<string>>(instance.at.location.first);
            for(string loc : locations) {
                pt::ptree location_node;
                location_node.put("", loc);

                locations_node.push_back(std::make_pair("", location_node));
            }
        } else {
            string location = std::get<string>(instance.at.location.first);
            locations_node.put("", location);
        }
        task_node.add_child("locations", locations_node);

        pt::ptree robotsnum_node;
        if(instance.at.fixed_robot_num) {
            robotsnum_node.put("fixed","True");

            robotsnum_node.put("num",std::get<int>(instance.at.robot_num));
        } else {
            robotsnum_node.put("fixed","False");

            pair<int,int> robot_range = std::get<pair<int,int>>(instance.at.robot_num);

            robotsnum_node.put("min",robot_range.first);
            robotsnum_node.put("max",robot_range.second);
        }
        task_node.add_child("robots_num", robotsnum_node);

        pt::ptree preconditions_node;
        for(auto prec : instance.prec) {
            pt::ptree precondition_node;
            bool output_prec = true;
            
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
                string prec_output;

                if(prec_mapping.first.get_mapping_type() == attribute_mapping_type) {
                    if(prec_mapping.first.get_mapped_type() == predicate_mapped_type) {
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

                        precondition_node.put("predicate", prec_output);
                        
                        pt::ptree vars_node;
                        for(string arg : arguments) {
                            vars_node.put("", arg);
                        }
                        precondition_node.push_back(std::make_pair("vars", vars_node));

                        pt::ptree vartypes_node;
                        for(string type : arg_sorts) {
                            vartypes_node.put("", type);
                        }
                        precondition_node.push_back(std::make_pair("var_types", vartypes_node));
                    } else if(prec_mapping.first.get_mapped_type() == function_mapped_type) {
                        // TODO
                        output_prec = false;
                    }
                } else if(prec_mapping.first.get_mapping_type() == ownership_mapping_type) {
                    /*
                        Do we need to output preconditions related to ownership type semantic mappings?
                    */
                    output_prec = false;
                }

                if(output_prec) {
                    preconditions_node.push_back(std::make_pair("",precondition_node));
                }
            }
        }
        task_node.add_child("preconditions", preconditions_node);

        pt::ptree effects_node;
        for(auto eff : instance.eff) {
            pt::ptree effect_node;

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

                if(eff_mapping.first.get_mapping_type() == attribute_mapping_type) {
                    if(eff_mapping.first.get_mapped_type() == predicate_mapped_type) {   
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

                        effect_node.put("predicate", eff_output);
                        
                        pt::ptree vars_node;
                        for(string arg : arguments) {
                            vars_node.put("", arg);
                        }
                        effect_node.push_back(std::make_pair("vars", vars_node));

                        pt::ptree vartypes_node;
                        for(string type : arg_sorts) {
                            vartypes_node.put("", type);
                        }
                        effect_node.push_back(std::make_pair("var_types", vartypes_node));
                    } else if(eff_mapping.first.get_mapped_type() == function_mapped_type) {
                        // TODO
                    }
                }

                effects_node.push_back(std::make_pair("", effect_node));
            }
        }
        task_node.add_child("effects", effects_node);

        pt::ptree events_node;
        for(string e : instance.at.triggering_events) {
            pt::ptree event_node;
            event_node.put("", e);

            events_node.push_back(std::make_pair("", event_node));
        }
        task_node.add_child("triggering_events", events_node);

        pt::ptree decomposition_node;
        for(task a : instance.path.decomposition) {
            pt::ptree action_node;

            if(a.name.find(method_precondition_action_name) == string::npos) {
                action_node.put("", a.name);

                decomposition_node.push_back(std::make_pair("", action_node));
            }
        }
        task_node.add_child("decomposition", decomposition_node);

        string group = instance_info.second.first ? "True" : "False";
        task_node.put("group", group);

        string divisible = instance_info.second.second ? "True" : "False";
        task_node.put("divisible", divisible);

        string task_name = "t" + to_string(task_counter);;
        tasks_node.push_back(std::make_pair(task_name,task_node));

        task_id_map[instance.id] = task_name;
        task_counter++;
    }

    output_file.add_child("tasks", tasks_node);

    return task_id_map;
}

void JSONOutputGenerator::output_constraints(pt::ptree& output_file, vector<Constraint> mission_constraints, map<string,string> task_id_map) {
    pt::ptree constraints_node;

    for(Constraint c : mission_constraints) {
        pt::ptree constraint_node;

        if(c.type == SEQ) {
            constraint_node.put("type", "SEQ");
        } else {
            constraint_node.put("type", "EC");
        }   

        pt::ptree taskinstances_node;

        string t_id = std::get<Decomposition>(c.nodes_involved.first.second.content).id;
        taskinstances_node.put("t0",task_id_map[t_id]);

        t_id = std::get<Decomposition>(c.nodes_involved.second.second.content).id;
        taskinstances_node.put("t1",task_id_map[t_id]);

        constraint_node.add_child("task_instances", taskinstances_node);

        if(c.type == NC) {
            string attr_value;

            if(c.group) {
                constraint_node.put("group", "True");
            } else {
                constraint_node.put("group", "False");
            }

            if(c.divisible) {
                constraint_node.put("divisible", "True");
            } else {
                constraint_node.put("divisible", "False");
            }
        }

        constraints_node.push_back(std::make_pair("", constraint_node));
    }

    output_file.add_child("constraints", constraints_node);
}

void JSONOutputGenerator::output_mission_decompositions(pt::ptree& output_file, vector<vector<pair<int,ATNode>>> valid_mission_decompositions, map<string,string> task_id_map) {
    pt::ptree mission_decompositions_node;

    for(vector<pair<int,ATNode>> mission_decomposition : valid_mission_decompositions) {
        pt::ptree mission_decomposition_node;
        
        for(pair<int,ATNode> task : mission_decomposition) {
            pt::ptree task_node;
            task_node.put("", task_id_map[get<Decomposition>(task.second.content).id]);

            mission_decomposition_node.push_back(std::make_pair("", task_node));
        }

        mission_decompositions_node.push_back(std::make_pair("", mission_decomposition_node));
    }

    output_file.add_child("mission_decompositions", mission_decompositions_node);
}