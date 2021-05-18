#include "configchecker.hpp"

#include <set>
#include <variant>
#include <iostream>

using namespace std;

void check_config(vector<VariableMapping> variable_mapping, map<string,string> type_mapping, GMGraph gm, vector<task> abstract_tasks,
                    vector<SemanticMapping> semantic_mapping, vector<string> high_level_loc_types, vector<predicate_definition> predicate_definitions) {
    map<string,vector<VariableMapping>> gm_vars;
    map<string,int> task_id_map;
    set<string> ocl_types;

    for(VariableMapping vm : variable_mapping) {
        gm_vars[vm.get_gm_var()].push_back(vm);
        if(task_id_map.find(vm.get_task_id()) == task_id_map.end()) {
            task_id_map[vm.get_task_id()] = find_gm_node_by_id(vm.get_task_id(), gm);
        }
    } 

    check_types_in_var_mapping(gm_vars, task_id_map, type_mapping, gm, abstract_tasks, ocl_types);

    check_high_level_loc_types(high_level_loc_types, ocl_types);

    check_semantic_mapping(semantic_mapping, predicate_definitions, type_mapping, ocl_types);
}

void check_types_in_var_mapping(map<string,vector<VariableMapping>> gm_vars, map<string,int> task_id_map, map<string,string> type_mapping, 
                                GMGraph gm, vector<task> abstract_tasks, set<string>& ocl_types) {
    vector<int> gm_dfs_nodes = get_dfs_gm_nodes(gm);

    for(int v : gm_dfs_nodes) {
        if(gm[v].custom_props.find(controls_prop) != gm[v].custom_props.end()) {
            vector<pair<string,string>> controlled_vars = get<vector<pair<string,string>>>(gm[v].custom_props[controls_prop]);
            for(pair<string,string> cvar : controlled_vars) {
                ocl_types.insert(cvar.second);
                if(gm_vars.find(cvar.first) != gm_vars.end()) {
                    for(VariableMapping vm : gm_vars[cvar.first]) {
                        int task_gm_id = task_id_map[vm.get_task_id()];
                        
                        string at_name = parse_at_text(gm[task_gm_id].text).second;
                        
                        task at;
                        for(task t : abstract_tasks) {
                            if(t.name == at_name) {
                                at = t;
                                break;
                            }
                        }
                        if(at.name == "") {
                            string at_not_found_error = "Could not find AT " + at_name + " from variable mappings in HDDL AT's";

                            throw runtime_error(at_not_found_error);
                        }

                        bool found_hddl_var = false;
                        for(pair<string,string> at_var : at.vars) {
                            if(at_var.first == vm.get_hddl_var()) {
                                if(at_var.second != type_mapping[cvar.second]) {
                                    string type_error = "Type error in AT: " + at.name;

                                    throw runtime_error(type_error);
                                }

                                found_hddl_var = true;
                                break;
                            }
                        }
                        if(!found_hddl_var) {
                            string hddl_var_not_found_err = "Could not find variable " + vm.get_hddl_var() + " in AT " + at.name + " definition";

                            throw runtime_error(hddl_var_not_found_err);
                        }
                    }
                }
            }
        }
    }
}

void check_high_level_loc_types(vector<string> high_level_loc_types, set<string> ocl_types) {
    for(string loc : high_level_loc_types) {
        if(ocl_types.find(loc) == ocl_types.end()) {
            string loc_type_error = "High-level location type [" + loc + "] was not declared in the Goal Model";

            throw std::runtime_error(loc_type_error); 
        }
    }
}

void check_semantic_mapping(vector<SemanticMapping> semantic_mapping, vector<predicate_definition> predicate_definitions, map<string,string> type_mapping, set<string> ocl_types) {
    for(SemanticMapping sm : semantic_mapping) {
        bool is_robot_related_mapping = false;
        if(sm.get_mapping_type() == attribute_mapping_type) {
            if(holds_alternative<string>(sm.get_prop(relatesto_key))) {
                if(std::get<string>(sm.get_prop(relatesto_key)) == "robot") {
                    is_robot_related_mapping = true;
                }
            }
        } else if(sm.get_mapped_type() == ownership_mapping_type) {
            string owner = std::get<string>(sm.get_prop(owner_key));
            string owned = std::get<string>(sm.get_prop(owned_key));

            if(owner == "robot" || owned == "robot") { //This probably will need to change
                is_robot_related_mapping = true;
            }
        }

        if(!is_robot_related_mapping) {
            if(sm.get_mapped_type() == predicate_mapped_type) {
                if(sm.get_mapping_type() == attribute_mapping_type) {
                    variant<string,predicate_definition> relation_types = sm.get_prop(relatesto_key);
                    if(holds_alternative<string>(relation_types)) {
                        string relation_type = std::get<string>(relation_types);

                        if(ocl_types.find(relation_type) == ocl_types.end()) {
                            string relation_not_found_error = "Could not find in GM relation type [" + relation_type + "] declared in semantic mappings";

                            throw std::runtime_error(relation_not_found_error);
                        }

                        if(sm.has_prop("predicate_type")) {
                            if(parse_gm_var_type(relation_type) != "COLLECTION") {
                                string predicate_type_attr_error = "Cannot have attribute predicate_type for semantic mapping with relation type [" + relation_type + "]";

                                throw std::runtime_error(predicate_type_attr_error);
                            }
                        }
                    }

                    predicate_definition sm_pred = std::get<predicate_definition>(sm.get_prop(map_key));

                    bool found_sm_pred = false;
                    for(predicate_definition pred : predicate_definitions) {
                        if(pred.name == sm_pred.name) {
                            bool equal_args = true;
                            if(pred.argument_sorts.size() == sm_pred.argument_sorts.size()) {
                                for(unsigned int arg_index = 0; arg_index < pred.argument_sorts.size(); ++arg_index) {
                                    if(pred.argument_sorts.at(arg_index) != sm_pred.argument_sorts.at(arg_index)) {
                                        equal_args = false;
                                        break;
                                    }
                                }

                                if(equal_args) {
                                    found_sm_pred = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(!found_sm_pred) {
                        string predicate_not_found_error = "Could not find predicate [" + sm_pred.name + "] declared in semantic mappings";

                        throw std::runtime_error(predicate_not_found_error); 
                    } else {
                        if(holds_alternative<string>(relation_types)) {
                            string relation_type = std::get<string>(relation_types);

                            if(sm_pred.argument_sorts.size() != 1) {
                                string semantic_mapping_error = "Cannot map predicate [" + sm_pred.name + "] with " + to_string(sm_pred.argument_sorts.size()) + " arguments to relation type " + relation_type;  
                            
                                throw std::runtime_error(semantic_mapping_error);
                            }

                            if(type_mapping[relation_type] != sm_pred.argument_sorts.at(0)) {
                                string semantic_mapping_error = "Cannot map predicate [" + sm_pred.name + "] with argument of type " + sm_pred.argument_sorts.at(0) + " to relation type " + relation_type;

                                throw std::runtime_error(semantic_mapping_error);
                            }
                        }
                    }
                } else if(sm.get_mapping_type() == ownership_mapping_type) {
                    string owner_type = std::get<string>(sm.get_prop(owner_key));
                    string owned_type = std::get<string>(sm.get_prop(owned_key));
                    predicate_definition sm_pred = std::get<predicate_definition>(sm.get_prop(map_key));

                    bool found_sm_pred = false;
                    for(predicate_definition pred : predicate_definitions) {
                        if(pred.name == sm_pred.name) {
                            bool equal_args = true;
                            if(pred.argument_sorts.size() == sm_pred.argument_sorts.size()) {
                                for(unsigned int arg_index = 0; arg_index < pred.argument_sorts.size(); ++arg_index) {
                                    if(pred.argument_sorts.at(arg_index) != sm_pred.argument_sorts.at(arg_index)) {
                                        equal_args = false;
                                        break;
                                    }
                                }

                                if(equal_args) {
                                    found_sm_pred = true;
                                    break;
                                }
                            }
                        }
                    }

                    if(!found_sm_pred) {
                        string predicate_not_found_error = "Could not find predicate [" + sm_pred.name + "] declared in semantic mappings";

                        throw std::runtime_error(predicate_not_found_error); 
                    } else {
                        if(sm_pred.argument_sorts.size() != 2) {
                            string argument_number_error = "Wrong number of arguments in predicate of ownership type mapping";

                            throw std::runtime_error(argument_number_error);
                        }
                        /*
                            -> For now, first argument must be of owned type and second argument must be of owner type
                        */
                        if(type_mapping[owned_type] != sm_pred.argument_sorts.at(0)) {
                            string semantic_mapping_error = "Cannot map predicate [" + sm_pred.name + "] with argument of type " + sm_pred.argument_sorts.at(0) + " to owned type " + owned_type;

                            throw std::runtime_error(semantic_mapping_error);
                        }
                        if(type_mapping[owner_type] != sm_pred.argument_sorts.at(1)) {
                            string semantic_mapping_error = "Cannot map predicate [" + sm_pred.name + "] with argument of type " + sm_pred.argument_sorts.at(1) + " to owner type " + owner_type;

                            throw std::runtime_error(semantic_mapping_error);
                        }
                    }
                }
            }
        }
    }
}