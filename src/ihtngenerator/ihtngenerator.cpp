#include "ihtngenerator.hpp"

#include <boost/property_tree/json_parser.hpp>

#include "../validmissiongenerator/validmissiongenerator.hpp"

using namespace std;

IHTNGenerator::IHTNGenerator(GMGraph gm, ATGraph mission_decomposition, bool verbose, bool pretty_print, vector<ground_literal> world_state, vector<pair<ground_literal,variant<int,float>>> world_state_functions, vector<string> high_level_loc_types, map<string,string> type_mappings, map<string,CompleteDecompositionPath> decomposition_path_mapping) {
    this->gm = gm;
    this->mission_decomposition = mission_decomposition;
    this->verbose = verbose;
    this->pretty_print = pretty_print;
    this->world_state = world_state;
    this->world_state_functions = world_state_functions;
    this->high_level_loc_types = high_level_loc_types;
    this->type_mappings = type_mappings;
    this->decomposition_path_mapping = decomposition_path_mapping;
}

void IHTNGenerator::generate_ihtn(vector<SemanticMapping> semantic_mapping, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, set<string> robot_related_sorts) {
    ConstraintManager constraint_generator(gm, mission_decomposition, verbose, pretty_print);
    vector<Constraint> mission_constraints = constraint_generator.generate_mission_constraints();

    ValidMissionGenerator valid_missions_generator(mission_decomposition, gm, mission_constraints, world_state, world_state_functions, semantic_mapping, gm_var_map, verbose, robot_related_sorts, pretty_print);
    pair<vector<vector<pair<int,ATNode>>>,set<Decomposition>> valid_mission_decompositions_and_expanded_decompositions = valid_missions_generator.generate_valid_mission_decompositions();

    vector<vector<pair<int,ATNode>>> valid_mission_decompositions = valid_mission_decompositions_and_expanded_decompositions.first;
    set<Decomposition> expanded_decompositions = valid_mission_decompositions_and_expanded_decompositions.second;

    map<int,ATNode> nodes_map;
    map<int,pair<vector<int>,vector<constraint_type>>> seq_fb_constraints_map; 
    map<int,pair<vector<int>,vector<pair<bool,bool>>>> exec_constraints_map;

    for(Constraint c : mission_constraints) {
        if(c.type == SEQ || c.type == FB) {
            if(seq_fb_constraints_map.find(c.nodes_involved.first.first) == seq_fb_constraints_map.end()) {
                pair<vector<int>,vector<constraint_type>> aux;
                aux.first.push_back(c.nodes_involved.second.first);
                aux.second.push_back(c.type);

                seq_fb_constraints_map[c.nodes_involved.first.first] = aux;
            } else {
                seq_fb_constraints_map[c.nodes_involved.first.first].first.push_back(c.nodes_involved.second.first);
                seq_fb_constraints_map[c.nodes_involved.first.first].second.push_back(c.type);
            }
        } else if(c.type == NC) {
            if(exec_constraints_map.find(c.nodes_involved.first.first) == exec_constraints_map.end()) {
                pair<vector<int>,vector<pair<bool,bool>>> aux1, aux2;

                aux1.first.push_back(c.nodes_involved.second.first);
                aux1.second.push_back(make_pair(c.group,c.divisible));

                aux2.first.push_back(c.nodes_involved.first.first);
                aux2.second.push_back(make_pair(c.group,c.divisible));

                exec_constraints_map[c.nodes_involved.first.first] = aux1;
                exec_constraints_map[c.nodes_involved.second.first] = aux2;
            } else {
                exec_constraints_map[c.nodes_involved.first.first].first.push_back(c.nodes_involved.second.first);
                exec_constraints_map[c.nodes_involved.first.first].second.push_back(make_pair(c.group,c.divisible));

                exec_constraints_map[c.nodes_involved.second.first].first.push_back(c.nodes_involved.first.first);
                exec_constraints_map[c.nodes_involved.second.first].second.push_back(make_pair(c.group,c.divisible));
            }
        }
    }

    map<int,pair<vector<int>,vector<constraint_type>>>::iterator ctrs_it;
    for(ctrs_it = seq_fb_constraints_map.begin(); ctrs_it != seq_fb_constraints_map.end(); ctrs_it++) {
        map<int,pair<vector<int>,vector<constraint_type>>>::iterator ctrs_it2;
        for(ctrs_it2 = seq_fb_constraints_map.begin(); ctrs_it2 != seq_fb_constraints_map.end(); ctrs_it2++) {
            if(std::find(ctrs_it->second.first.begin(),ctrs_it->second.first.end(),ctrs_it2->first) != ctrs_it->second.first.end()) {
                int elem_pos = std::distance(ctrs_it->second.first.begin(),std::find(ctrs_it->second.first.begin(),ctrs_it->second.first.end(),ctrs_it2->first));

                for(int el : ctrs_it2->second.first) {
                    ctrs_it->second.first.push_back(el);
                    ctrs_it->second.second.push_back(ctrs_it->second.second.at(elem_pos));
                }
            }
        }
    }

    // For each valid decomposition generate a different iHTN
    int ihtn_counter = 1;
    for(vector<pair<int,ATNode>> decomposition : valid_mission_decompositions) {
        map<int,vector<string>> agents_map;
        set<string> agents_set;
        set<int> verified_tasks;
        map<string,string> non_ground_agents_map;

        for(pair<int,ATNode> task_decomposition : decomposition) {
            if(nodes_map.find(task_decomposition.first) == nodes_map.end()) {
                nodes_map[task_decomposition.first] = task_decomposition.second;
            }

            Decomposition d = std::get<Decomposition>(task_decomposition.second.content);

            vector<string> task_agents;
            vector<pair<string,string>> task_non_ground_args;
            for(auto arg : d.arguments) {
                if(holds_alternative<string>(arg.first)) {
                    string argument = std::get<string>(arg.first);

                    bool ground = (argument == "") ? false : true;
                    if(ground) {
                        bool is_not_location_var = false;
                        if(holds_alternative<vector<string>>(d.at.location.first)) {
                            vector<string> aux = std::get<vector<string>>(d.at.location.first);

                            if(std::find(aux.begin(), aux.end(), argument) == aux.end()) {
                                is_not_location_var = true;
                            }
                        } else {
                            if(std::get<string>(d.at.location.first) != argument) {
                                is_not_location_var = true;
                            }
                        }

                        if(is_not_location_var) {
                            agents_set.insert(argument);

                            task_agents.push_back(argument);
                        }
                    } else {
                        task_non_ground_args.push_back(arg.second);
                    }
                } else {
                    string vector_of_agents_error("Agent-related type container is not supported for iHTN generation");

                    throw std::runtime_error(vector_of_agents_error);
                }
            }
            
            // TODO: Use the sorts vector to find types that inherit from robot and robotteam
            for(pair<string,string> ng_arg : task_non_ground_args) {
                if(ng_arg.second != hddl_robot_type && ng_arg.second != hddl_robotteam_type) {
                    string non_ground_arg_error = "Variable " + ng_arg.first + " of HDDL type " + ng_arg.second + " is not grounded. iHTN generation does not support non-ground variables that are not robot-related";

                    throw std::runtime_error(non_ground_arg_error);
                }
            }

            pair<vector<int>,vector<pair<bool,bool>>> exec_constraints = exec_constraints_map[task_decomposition.first];

            bool found_ng_args = false;
            vector<string> ng_args;
            vector<string> ng_args_found;
            for(int task_id : verified_tasks) {
                vector<int>::iterator t_pos = std::find(exec_constraints.first.begin(), exec_constraints.first.end(), task_id);
                if(t_pos != exec_constraints.first.end()) {
                    int t_pos_index = std::distance(exec_constraints.first.begin(), t_pos);

                    int t_id = exec_constraints.first.at(t_pos_index);
                    pair<bool,bool> t_ctr = exec_constraints.second.at(t_pos_index);

                    if(task_non_ground_args.size() == 1) {
                        pair<string,string> only_ng_arg = task_non_ground_args.at(0);

                        if(only_ng_arg.second == hddl_robot_type) {
                            if(t_ctr.first == false) {
                                found_ng_args = true;

                                for(string arg : agents_map[task_id]) {
                                    if(arg.at(0) == '?') {
                                        ng_args.push_back(arg);

                                        break;
                                    }
                                }

                                break;
                            } else {
                                found_ng_args = true;

                                for(string arg : agents_map[task_id]) {
                                    if(arg.at(0) == '?') {
                                        ng_args.push_back(arg);
                                        ng_args_found.push_back(only_ng_arg.first);

                                        break;
                                    }
                                }

                                break;
                            }
                        } else { // The only argument is of robotteam type
                            found_ng_args = true;

                            for(string arg : agents_map[task_id]) {
                                if(arg.at(0) == '?') {
                                    ng_args.push_back(arg);

                                    break;
                                }
                            }
                        }
                    } else {
                        bool robot_type_vars = false;
                        for(pair<string,string> ng_arg : task_non_ground_args) {
                            if(ng_arg.second == hddl_robot_type) { // For now we assume that tasks can either have robotteam or robot type variables but not both
                                robot_type_vars = true;

                                break;
                            } else if(ng_arg.second == hddl_robotteam_type) {
                                break;
                            }
                        }

                        if(robot_type_vars) {
                            vector<string> constrained_task_agents = agents_map[task_id];
                            
                            vector<string>::iterator ct_ag_it;
                            for(ct_ag_it = constrained_task_agents.begin(); ct_ag_it != constrained_task_agents.end(); ) {
                                if(ct_ag_it->at(0) != '?') {
                                    constrained_task_agents.erase(ct_ag_it);
                                } else {
                                    ct_ag_it++;
                                }
                            }

                            if(ng_args_found.size() < constrained_task_agents.size()) {
                                unsigned int agents_found = ng_args_found.size();
                                for(unsigned int var_index = agents_found; var_index < task_non_ground_args.size(); var_index++) {
                                    if(var_index == constrained_task_agents.size()) {
                                        break;
                                    }

                                    unsigned int arg_index = 1;
                                    for(string agent : constrained_task_agents) {
                                        if(arg_index > ng_args_found.size()) {
                                            ng_args.push_back(agent);
                                            ng_args_found.push_back(task_non_ground_args.at(var_index).first);

                                            if(ng_args_found.size() == task_non_ground_args.size()) {
                                                found_ng_args = true;
                                            }

                                            break;
                                        }

                                        arg_index++;
                                    }

                                    if(found_ng_args) {
                                        break;
                                    }
                                }
                            }
                        } else {
                            // TODO: CHECK IF THIS CAN HAPPEN. CAN WE HAVE TWO robotteam type variables?
                        }

                        if(found_ng_args) {
                            break;
                        }
                    }
                }
            }

            // If we did not map non-ground arguments, we must insert them into the agents set
            // In order to avoid name conflicts, we will insert the decomposition ID into the variable name
            if(!found_ng_args) {
                string d_id = d.id;
                std::transform(d_id.begin(), d_id.end(), d_id.begin(), ::tolower);

                int arg_index = 0;
                for(pair<string,string> ng_arg : task_non_ground_args) {
                    if(std::find(ng_args_found.begin(),ng_args_found.end(),ng_arg.first) == ng_args_found.end()) {
                        string arg_name = ng_arg.first + "@" + d_id;

                        int non_ground_agents_index = non_ground_agents_map.size() + 1;
                        non_ground_agents_map[arg_name] = "r" + to_string(non_ground_agents_index);

                        agents_set.insert(arg_name);
                        task_agents.push_back(arg_name);
                    } else {
                        task_agents.push_back(ng_args.at(arg_index));

                        arg_index++;
                    }
                }
            } else {
                for(string ng_arg : ng_args) {
                    task_agents.push_back(ng_arg);
                }
            }

            agents_map[task_decomposition.first] = task_agents;
            verified_tasks.insert(task_decomposition.first);
            nodes_map[task_decomposition.first] = task_decomposition.second;
        }

        // Generate iHTN for valid decomposition

        //Verify if task decomposition is totally ordered. If not, generate multiple iHTNs for each ordering
        vector<int> decomposition_ids;
        for(pair<int,ATNode> dmp : decomposition) {
            decomposition_ids.push_back(dmp.first);
        }

        vector<vector<int>> decomposition_orderings = find_decomposition_orderings(decomposition_ids, seq_fb_constraints_map);

        for(vector<int> ordering : decomposition_orderings) {
            IHTN ordering_ihtn = ihtn_create(ordering, nodes_map, agents_set, agents_map);

            auto indexmap = boost::get(boost::vertex_index, ordering_ihtn);
            auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

            IHTNDFSVisitor vis;
            boost::depth_first_search(ordering_ihtn, vis, colormap, 0);

            vector<int> dfs_nodes = vis.GetVector();
            
            int node_id = 0;
            map<int,int> ihtn_id_to_json_id;
            map<int,int> parent_map;

            pt::ptree ihtn_tree;
            for(int ihtn_node : dfs_nodes) {
                IHTNNode node = ordering_ihtn[ihtn_node];

                int parent_id = -1;
                if(parent_map.find(ihtn_node) != parent_map.end()) {
                    parent_id = ihtn_id_to_json_id[parent_map[ihtn_node]];
                }

                IHTN::out_edge_iterator oi, oi_end;
                for(boost::tie(oi,oi_end) = out_edges(ihtn_node,ordering_ihtn);oi != oi_end;++oi) {
                    int target = boost::target(*oi,ordering_ihtn);
                    
                    parent_map[target] = ihtn_node;
                }

                if(node.type == IHTNACTION) {
                    pt::ptree action_node;

                    ActionNode a = std::get<ActionNode>(node.content);

                    action_node.put("name", a.name);
                    action_node.put("type", "action");
                    action_node.put("parent", parent_id);

                    pt::ptree locations_node;
                    for(string loc : a.locations) {
                        pt::ptree location_node;
                        location_node.put("", loc);

                        locations_node.push_back(std::make_pair("", location_node));
                    }
                    action_node.add_child("locations", locations_node);

                    pt::ptree agents_node;
                    for(string agent : a.agents) {
                        pt::ptree agent_node;
                        if(non_ground_agents_map.find(agent) == non_ground_agents_map.end()) {
                            agent_node.put("", agent);
                        } else {
                            agent_node.put("", non_ground_agents_map[agent]);
                        }

                        agents_node.push_back(std::make_pair("", agent_node));
                    }
                    action_node.add_child("agents", agents_node);

                    ihtn_tree.push_back(make_pair(to_string(node_id), action_node));
                } else if(node.type == IHTNTASK) {
                    pt::ptree task_node;

                    TaskNode t = std::get<TaskNode>(node.content);

                    task_node.put("name", t.name);
                    task_node.put("type", "task");
                    task_node.put("parent", parent_id);

                    pt::ptree agents_node;
                    for(string agent : t.agents) {
                        pt::ptree agent_node;
                        if(non_ground_agents_map.find(agent) == non_ground_agents_map.end()) {
                            agent_node.put("", agent);
                        } else {
                            agent_node.put("", non_ground_agents_map[agent]);
                        }

                        agents_node.push_back(std::make_pair("", agent_node));
                    }
                    task_node.add_child("agents", agents_node);

                    ihtn_tree.push_back(make_pair(to_string(node_id), task_node));
                } else {
                    pt::ptree method_node;

                    MethodNode m = std::get<MethodNode>(node.content);

                    method_node.put("name", m.name);
                    method_node.put("type", "method");
                    method_node.put("parent", parent_id);

                    pt::ptree agents_node;
                    for(string agent : m.agents) {
                        pt::ptree agent_node;
                        if(non_ground_agents_map.find(agent) == non_ground_agents_map.end()) {
                            agent_node.put("", agent);
                        } else {
                            agent_node.put("", non_ground_agents_map[agent]);
                        }

                        agents_node.push_back(std::make_pair("", agent_node));
                    }
                    method_node.add_child("agents", agents_node);

                    ihtn_tree.push_back(make_pair(to_string(node_id), method_node));
                }

                ihtn_id_to_json_id[ihtn_node] = node_id;
                node_id++;
            }

            for(int ihtn_node : dfs_nodes) {
                vector<int> children_ids;
                IHTN::out_edge_iterator oi, oi_end;
                for(boost::tie(oi,oi_end) = out_edges(ihtn_node,ordering_ihtn);oi != oi_end;++oi) {
                    int target = boost::target(*oi,ordering_ihtn);

                    children_ids.push_back(ihtn_id_to_json_id[target]);
                    parent_map[target] = ihtn_node;
                }

                int json_id = ihtn_id_to_json_id[ihtn_node];
                auto node = ihtn_tree.find(to_string(json_id));

                pt::ptree children_node;
                for(int child : children_ids) {
                    pt::ptree child_node;
                    child_node.put("", to_string(child));

                    children_node.push_back(make_pair("",child_node));
                }
                node->second.add_child("children", children_node);
            }

            string file_name = "ihtn_" + to_string(ihtn_counter) + ".json";
            pt::write_json(file_name, ihtn_tree, std::locale());

            ihtn_counter++;
        }
    }
}

IHTN IHTNGenerator::ihtn_create(vector<int> nodes, map<int,ATNode> nodes_map, set<string> agents, map<int,vector<string>> agents_map) {
    IHTN ihtn;
    
    TaskNode root;
    root.name = "ROOT";
    root.id = "ROOT";
    root.agents = agents;

    IHTNNode root_ihtn_node;
    root_ihtn_node.type = IHTNTASK;
    root_ihtn_node.content = root;

    int root_id = boost::add_vertex(root_ihtn_node, ihtn);

    MethodNode root_method;
    root_method.name = "ROOT_M";
    root_method.task_id = -1;
    root_method.agents = agents;
    
    IHTNNode root_method_ithn_node;
    root_method_ithn_node.type = IHTNMETHOD;
    root_method_ithn_node.content = root_method;

    int root_method_id = boost::add_vertex(root_method_ithn_node, ihtn);
    boost::add_edge(root_id,root_method_id,ihtn);

    map<string,pair<int,int>> decomposition_indexes;
    for(int node_id : nodes) {
        ATNode node = nodes_map[node_id];

        Decomposition d = std::get<Decomposition>(node.content);
        
        TaskNode decomposition_node;
        decomposition_node.name = d.at.at.name;
        decomposition_node.id = d.id;
        set<string> d_agents(agents_map[node_id].begin(),agents_map[node_id].end());
        decomposition_node.agents = d_agents;

        IHTNNode decomposition_ithn_node;
        decomposition_ithn_node.content = decomposition_node;
        decomposition_ithn_node.type = IHTNTASK;

        int decomposition_node_id = boost::add_vertex(decomposition_ithn_node, ihtn);

        boost::add_edge(root_method_id, decomposition_node_id, ihtn);

        CompleteDecompositionPath path = decomposition_path_mapping[d.id];
        
        map<int,int> decomposition_id_to_path_id;
        map<int,int> path_id_to_ihtn_id;

        int path_index = 0;
        for(DecompositionNode path_node : path.decomposition) {
            if(path_node.parent != -1) {
                decomposition_id_to_path_id[path_node.id] = path_index;

                int ihtn_node_id = -1;
                if(holds_alternative<method>(path_node.content)) {
                    method m = std::get<method>(path_node.content);

                    MethodNode m_node;
                    m_node.name = m.name;
                    //m_node.task_id
                    set<string> method_agents;

                    int agent_index = 0;
                    for(pair<string,string> var : m.vars) {
                        string mapping_val;
                        for(auto arg : d.arguments) {
                            if(arg.second.first == var.first) {
                                if(holds_alternative<string>(arg.first)) { // Only alternative for now
                                    string aux = std::get<string>(arg.first);

                                    if(aux == "") {
                                        mapping_val = arg.second.first;
                                    } else {
                                        mapping_val = aux;
                                    }
                                }

                                break;
                            }
                        }

                        if(mapping_val == "") {
                            // TODO: DEAL WITH ERROR IF IT CAN HAPPEN!
                            throw std::runtime_error("Could not find mapping for variable");
                        } else {
                            bool is_not_location_var = false;
                            if(holds_alternative<vector<string>>(d.at.location.first)) {
                                vector<string> aux = std::get<vector<string>>(d.at.location.first);

                                if(std::find(aux.begin(), aux.end(), mapping_val) == aux.end()) {
                                    is_not_location_var = true;
                                }
                            } else {
                                if(std::get<string>(d.at.location.first) != mapping_val) {
                                    is_not_location_var = true;
                                }
                            }

                            if(is_not_location_var) {
                                string agent_val;

                                if(mapping_val.find("?") != std::string::npos) {
                                    int map_index = 0;
                                    for(string agent : agents_map[node_id]) {
                                        if(agent.find("@") != std::string::npos) {
                                            if(map_index == agent_index) {
                                                agent_val = agent;
                                                break;
                                            }

                                            map_index++;
                                        }
                                    }

                                    agent_index++;
                                } else {
                                    agent_val = mapping_val;
                                }

                                method_agents.insert(agent_val);
                            }
                        }
                    }
                    m_node.agents = method_agents;

                    IHTNNode m_ihtn_node;
                    m_ihtn_node.content = m_node;
                    m_ihtn_node.type = IHTNMETHOD;

                    ihtn_node_id = boost::add_vertex(m_ihtn_node, ihtn);
                    path_id_to_ihtn_id[path_index] = ihtn_node_id;
                } else {
                    task t = std::get<task>(path_node.content);
                    if(t.name.find(method_precondition_action_name) == std::string::npos) {
                        set<string> task_agents;
                        int agent_index = 0;
                        for(int var_index = 0; var_index < t.number_of_original_vars; var_index++) {
                            string var = t.vars.at(var_index).first;

                            string mapping_val;
                            for(auto arg : d.arguments) {
                                if(arg.second.first == var) {
                                    if(holds_alternative<string>(arg.first)) { // Only alternative for now
                                        string aux = std::get<string>(arg.first);

                                        if(aux == "") {
                                            mapping_val = arg.second.first;
                                        } else {
                                            mapping_val = aux;
                                        }
                                    }

                                    break;
                                }
                            }

                            if(mapping_val == "") {
                                // TODO: DEAL WITH ERROR IF IT CAN HAPPEN!
                                throw std::runtime_error("Could not find mapping for variable");
                            } else {
                                bool is_not_location_var = false;
                                if(holds_alternative<vector<string>>(d.at.location.first)) {
                                    vector<string> aux = std::get<vector<string>>(d.at.location.first);

                                    if(std::find(aux.begin(), aux.end(), mapping_val) == aux.end()) {
                                        is_not_location_var = true;
                                    }
                                } else {
                                    if(std::get<string>(d.at.location.first) != mapping_val) {
                                        is_not_location_var = true;
                                    }
                                }

                                if(is_not_location_var) {
                                    string agent_val;

                                    if(mapping_val.find("?") != std::string::npos) {
                                        int map_index = 0;
                                        for(string agent : agents_map[node_id]) {
                                            if(agent.find("@") != std::string::npos) {
                                                if(map_index == agent_index) {
                                                    agent_val = agent;
                                                    break;
                                                }

                                                map_index++;
                                            }
                                        }

                                        agent_index++;
                                    } else {
                                        agent_val = mapping_val;
                                    }

                                    task_agents.insert(agent_val);
                                }
                            }
                        }

                        if(!path_node.is_primitive_task_node) {
                            TaskNode t_node;
                            t_node.name = t.name;
                            //t_node.id

                            t_node.agents = task_agents;

                            IHTNNode t_ihtn_node;
                            t_ihtn_node.content = t_node;
                            t_ihtn_node.type = IHTNTASK;

                            ihtn_node_id = boost::add_vertex(t_ihtn_node, ihtn);
                            path_id_to_ihtn_id[path_index] = ihtn_node_id;
                        } else {
                            ActionNode a_node;
                            a_node.name = t.name;
                            
                            vector<string> action_locations;
                            for(int var_index = 0; var_index < t.number_of_original_vars; var_index++) {
                                string var = t.vars.at(var_index).first;

                                string mapping_val;
                                for(auto arg : d.arguments) {
                                    if(arg.second.first == var) {
                                        if(holds_alternative<string>(arg.first)) { // Only alternative for now
                                            string aux = std::get<string>(arg.first);

                                            if(aux == "") {
                                                mapping_val = arg.second.first;
                                            } else {
                                                mapping_val = aux;
                                            }
                                        }

                                        break;
                                    }
                                }

                                if(mapping_val == "") {
                                    // TODO: DEAL WITH ERROR IF IT CAN HAPPEN!
                                    throw std::runtime_error("Could not find mapping for variable");
                                } else {
                                    bool is_location_var = false;
                                    if(holds_alternative<vector<string>>(d.at.location.first)) {
                                        vector<string> aux = std::get<vector<string>>(d.at.location.first);

                                        if(std::find(aux.begin(), aux.end(), mapping_val) != aux.end()) {
                                            is_location_var = true;
                                        }
                                    } else {
                                        if(std::get<string>(d.at.location.first) == mapping_val) {
                                            is_location_var = true;
                                        }
                                    }

                                    if(is_location_var) {
                                        action_locations.push_back(mapping_val);
                                        break;
                                    }
                                }
                            }
                            a_node.locations = action_locations;

                            a_node.agents = task_agents;

                            IHTNNode a_ihtn_node;
                            a_ihtn_node.content = a_node;
                            a_ihtn_node.type = IHTNACTION;

                            ihtn_node_id = boost::add_vertex(a_ihtn_node, ihtn);
                            path_id_to_ihtn_id[path_index] = ihtn_node_id;
                        }
                    }
                }

                if(path_node.parent == 0) {
                    boost::add_edge(decomposition_node_id, ihtn_node_id, ihtn);
                } else {
                    if(ihtn_node_id != -1) {
                        int parent_ihtn_id = path_id_to_ihtn_id[decomposition_id_to_path_id[path_node.parent]];

                        boost::add_edge(parent_ihtn_id, ihtn_node_id, ihtn);
                    }
                }
            }

            path_index++;
        }
    }

    return ihtn;
}   

vector<vector<int>> find_decomposition_orderings(vector<int> decomposition, map<int,pair<vector<int>,vector<constraint_type>>> seq_fb_constraints_map) {
    vector<vector<int>> possible_orderings;
    
    for(int decomposition_index = 0; decomposition_index < decomposition.size(); decomposition_index++) {
        vector<int> aux = decomposition;
        aux.erase(aux.begin()+decomposition_index);

        vector<vector<int>> current_ordering;
        current_ordering.push_back(vector<int>{decomposition.at(decomposition_index)});

        vector<vector<int>> orderings_to_add = recursive_decomposition_ordering_find(current_ordering, aux, seq_fb_constraints_map);
        possible_orderings.insert(possible_orderings.end(), orderings_to_add.begin(), orderings_to_add.end());
    }

    return possible_orderings;
}

vector<vector<int>> recursive_decomposition_ordering_find(vector<vector<int>> current_orderings, vector<int> decomposition, map<int,pair<vector<int>,vector<constraint_type>>> seq_fb_constraints_map) {
    if(decomposition.size() == 0) {
        return current_orderings;
    }

    vector<vector<int>> possible_orderings;

    vector<int>::iterator decomposition_it;
    for(decomposition_it = decomposition.begin(); decomposition_it != decomposition.end(); decomposition_it++) {
        vector<vector<int>> new_orderings;
        pair<vector<int>,vector<constraint_type>> current_node_constraints = seq_fb_constraints_map[*decomposition_it];

        int current_orderings_index;
        for(current_orderings_index = 0; current_orderings_index < current_orderings.size(); ) {
            bool is_possible_ordering = true;
            for(int element : current_orderings.at(current_orderings_index)) {
                if(std::find(current_node_constraints.first.begin(), current_node_constraints.first.end(), element) != current_node_constraints.first.end()) {
                    is_possible_ordering = false;

                    break;
                }
            }

            if(!is_possible_ordering) {
                current_orderings.erase(current_orderings.begin()+current_orderings_index);
            } else {
                vector<int> new_ordering = current_orderings.at(current_orderings_index);
                new_ordering.push_back(*decomposition_it);

                new_orderings.push_back(new_ordering);

                current_orderings_index++;
            }
        }

        if(current_orderings.size() == 0) {
            possible_orderings.clear();

            break;
        } else {
            vector<int> aux = decomposition;
            aux.erase(aux.begin()+std::distance(decomposition.begin(),decomposition_it));

            vector<vector<int>> orderings_to_add = recursive_decomposition_ordering_find(new_orderings, aux, seq_fb_constraints_map);
            possible_orderings.insert(possible_orderings.end(), orderings_to_add.begin(), orderings_to_add.end());
        }
    }

    return possible_orderings;
}