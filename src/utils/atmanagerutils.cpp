#include "atmanagerutils.hpp"

#include <iostream>
#include <regex>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "math_utils.hpp"

using namespace std;

/*
    Function: print_at_instances_info
    Objective: Print abstract tasks instances info in terminal

    @ Input: The abstract task instances in a map format
    @ Output: Void. We just print to the terminal
*/ 
void print_at_instances_info(map<string,vector<AbstractTask>> at_instances) {
	map<string,vector<AbstractTask>>::iterator at_it;
	cout << "AT instances:" << endl;
	for(at_it = at_instances.begin();at_it != at_instances.end();++at_it) {
		cout << "AT name: " << at_it->first << endl;
		for(AbstractTask inst : at_it->second) {
			cout << "ID: " << inst.id << endl;
			cout << "Name: " << inst.name << endl;
			cout << "Variable Mappings:" << endl;
			for(auto var_map : inst.variable_mapping) {
				if(holds_alternative<string>(var_map.first.first)) {
					cout << var_map.second << ": " << std::get<string>(var_map.first.first) << endl;
				} else {
					vector<string> map_values = std::get<vector<string>>(var_map.first.first);
					cout << var_map.second << ": [";
					unsigned int index = 0;
					for(string val : map_values) {
						if(index == map_values.size()-1) {
							cout << val << "]" << endl;
						} else {
							cout << val << ",";
						}
						index++;
					}
				}
			}
			cout << "Triggering Events:" << endl;
			for(string event : inst.triggering_events) {
				cout << event << ", ";
			}
			cout << endl;
			cout << "Location(s):" << endl;
			cout << "Location(s) Var: " << inst.location.second.first << " : " << inst.location.second.second << endl;
			cout << "Location(s) Value: " << endl;
			if(holds_alternative<vector<string>>(inst.location.first)) {
				vector<string> locs = get<vector<string>>(inst.location.first);
				for(string loc : locs) {
					cout << loc << endl;
				}
			} else {
				string loc = get<string>(inst.location.first);
				cout << loc << endl;
			}
			cout << endl;
		}
	}
}

/*
    Function: print_at_paths_info
    Objective: Print abstract tasks decomposition paths info in terminal

    @ Input: The abstract tasks decomposition paths in a map format
    @ Output: Void. We just print to the terminal
*/ 
void print_at_paths_info(map<string,vector<DecompositionPath>> at_decomposition_paths) {
	map<string,vector<DecompositionPath>>::iterator at_paths_it;

	for(at_paths_it = at_decomposition_paths.begin();at_paths_it != at_decomposition_paths.end();++at_paths_it) {
		cout << "Abstract task " << at_paths_it->first << " decomposition paths:" << endl;

		for(auto path : at_paths_it->second) {
			cout << "#####################################" << endl;
			cout << "Path: ";
			for(auto t : path.decomposition) {
				cout << t.name;
				if(t.name != path.decomposition.back().name) {
					cout << " -> ";
				} else {
					cout << endl;
				}
			}
			cout << endl;
			cout << "Needs expansion? " << path.needs_expansion << endl;
			if(path.needs_expansion) {
				for(auto fragment : path.fragments_to_expand) {
					cout << "Expansion of tasks " << fragment.first.first << " to " << fragment.first.second << endl;
					cout << "Number of expansions: ( " << fragment.second.predicate << " ";
					for(string arg : fragment.second.arguments) {
						std::cout << arg << " ";
					}
					if(holds_alternative<int>(fragment.second.comparison_op_and_value.second)) {
						cout << ") - " << to_string(std::get<int>(fragment.second.comparison_op_and_value.second)) << endl;
					} else {
						cout << ") - " << to_string(std::get<float>(fragment.second.comparison_op_and_value.second)) << endl;
					}
				}
			}
			cout << "#####################################" << endl;
		}

		cout << endl;
	}
}

/*
    Function: check_path_validity
    Objective: Check if a path of tasks is valid given a world state

    @ Input 1: The path to be checked
	@ Input 2: The world state used for the evaluation
	@ Input 3: The abstract task that originates the path of decomposition
	@ Input 4: The semantic mappings defined in the configuration file
    @ Output: A boolean value indicating if the path is valid or not
*/
bool check_path_validity(vector<task> path, vector<ground_literal> world_state, AbstractTask at, vector<SemanticMapping> semantic_mappings) {
	bool valid_path = true;
	for(task t : path) {
		bool prec_satistfied = true;
		for(literal prec : t.prec) {
			/*
				Check if predicate involves an instantiated variable that belongs to the variable mapping of the AT
			*/
			bool instantiated_prec = true;
			vector<pair<string,pair<variant<vector<string>,string>,string>>> arg_map;
			for(string arg : prec.arguments) {
				bool found_arg = false;
				pair<variant<vector<string>,string>,string> mapped_var;
				for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
					if(arg == var_map.second) {
						found_arg = true;
						mapped_var = var_map.first;
						break;
					}
				}

				if(!found_arg) {
					instantiated_prec = false;
					break;
				}

				arg_map.push_back(make_pair(arg,mapped_var));
			}

			if(instantiated_prec) {
				vector<ground_literal> inst_precs;
				bool is_universal = true;

				// Here is probably one place where we have to expand the predicate if we have a collection mapping
				for(pair<string,pair<variant<vector<string>,string>,string>> arg_inst : arg_map) {
					if(holds_alternative<string>(arg_inst.second.first)) {
						ground_literal p;
						p.positive = prec.positive;
						p.predicate = prec.predicate;
						p.args.push_back(std::get<string>(arg_inst.second.first));

						inst_precs.push_back(p);
					} else {
						vector<string> arg_values = std::get<vector<string>>(arg_inst.second.first);
						string var_ocl_type = arg_inst.second.second; 
						std::transform(var_ocl_type.begin(),var_ocl_type.end(),var_ocl_type.begin(),::toupper);
						for(SemanticMapping sm : semantic_mappings) {
							predicate_definition sm_pred = std::get<predicate_definition>(sm.get_prop("map"));
							if(sm_pred.name == prec.predicate) {
								string relation_type = std::get<string>(sm.get_prop(relatesto_key));
								std::transform(relation_type.begin(),relation_type.end(),relation_type.begin(),::toupper);
								if(relation_type == var_ocl_type) {
									map<string, variant<string, predicate_definition>> sm_props = sm.get_mapping_props();
									if(sm_props.find("predicate_type") != sm_props.end()) {
										string sm_pred_type = std::get<string>(sm.get_prop("predicate_type"));
										std::transform(sm_pred_type.begin(),sm_pred_type.end(),sm_pred_type.begin(),::toupper);
										if(sm_pred_type == "EXISTENTIAL") {
											is_universal = false;
										}
									}

									break;
								}
							}
						}

						for(string arg_val : arg_values) {
							ground_literal p;
							p.positive = prec.positive;
							p.predicate = prec.predicate;
							p.args.push_back(arg_val);

							inst_precs.push_back(p);
						}
					}
				}

				vector<bool> prec_evals;
				for(ground_literal inst_prec : inst_precs) {
					for(ground_literal state : world_state) {
						if(state.predicate == inst_prec.predicate) {
							bool equal_args = true;
							for(unsigned int arg_index = 0;arg_index < state.args.size();arg_index++) {
								if(state.args.at(arg_index) != inst_prec.args.at(arg_index)) {
									equal_args = false;
									break;
								}
							}

							if(equal_args && (prec.positive != state.positive)) {
								prec_evals.push_back(false);
								if(is_universal) {
									prec_satistfied = false;
								}
								break;
							} else if(equal_args && (prec.positive == state.positive)) {
								prec_evals.push_back(true);
								break;
							}
						}
					}
				}

				if(!is_universal) {
					bool at_least_one_eval_true = false;
					for(bool eval : prec_evals) {
						if(eval) {
							at_least_one_eval_true = true;
							break;
						}
					}
					if(!at_least_one_eval_true) {
						prec_satistfied = false;
					}
				}

				if(!prec_satistfied) {
					valid_path = false;
					break;
				}
			}
		}

		if(!valid_path) break;
	}

	return valid_path;
}

pair<vector<pt::ptree>,set<string>> solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>>& valid_variables, 
							map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map) {
	if(holds_alternative<pair<Query*,Query*>>(q.query->query)) {
        pair<Query*,Query*> query_items = std::get<pair<Query*,Query*>>(q.query->query);

        QueriedProperty aux1;
        aux1.queried_var = q.queried_var;
        aux1.query_var = q.query_var;
        aux1.query = query_items.first;

        pair<vector<pt::ptree>,set<string>> valid_query1 = solve_query_statement(queried_tree, aux1, gm, node_id, valid_variables, gm_var_map);

        QueriedProperty aux2;
        aux2.queried_var = q.queried_var;
        aux2.query_var = q.query_var;
        aux2.query = query_items.second;

        pair<vector<pt::ptree>,set<string>> valid_query2 = solve_query_statement(queried_tree, aux2, gm, node_id, valid_variables, gm_var_map);

		pair<vector<pt::ptree>,set<string>> final_result = valid_query1;

        if(q.query->is_and) {
            set<string> aux;

            std::set_difference(valid_query1.second.begin(), valid_query1.second.end(), valid_query2.second.begin(), valid_query2.second.end(), std::inserter(aux, aux.end()));

            for(string elem : aux) {
                set<string>::iterator pos = final_result.second.find(elem);
                if(pos != final_result.second.end()) {
                    final_result.second.erase(pos);
                }
            }

            vector<pt::ptree>::iterator result_it;
            for(result_it = final_result.first.begin(); result_it != final_result.first.end(); ) {
                string result_val = result_it->get<string>("name");

                if(final_result.second.find(result_val) == final_result.second.end()) {
                    final_result.first.erase(result_it);
                } else {
                    result_it++;
                }
            }
        } else {
            set<string> aux;

            std::set_difference(valid_query1.second.begin(), valid_query1.second.end(), valid_query2.second.begin(), valid_query2.second.end(), std::inserter(aux, aux.end()));

            for(string elem : aux) {
                set<string>::iterator pos = final_result.second.find(elem);
                if(pos == final_result.second.end()) {
                    final_result.second.insert(elem);
                }
            }

			for(pt::ptree res : valid_query2.first) {
				if(aux.find(res.get<string>("name")) != aux.end()) {
					final_result.first.push_back(res);
				}
			}
        }

		return final_result;
    } else {
		vector<pt::ptree> aux;
		set<string> accepted_records;

		vector<string> query_item = std::get<vector<string>>(q.query->query);
					
		if(!queried_tree.empty()) {
			BOOST_FOREACH(pt::ptree::value_type& child, queried_tree) {
				if(child.first == q.query_var.second) {
					if(query_item.size() == 0) {
						aux.push_back(child.second);
						accepted_records.insert(child.second.get<string>("name"));
					} else if(query_item.size() == 1) {
						if(query_item.at(0) != "") {
							string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

							boost::optional prop_val_opt = child.second.get_optional<string>(prop);

							if(prop_val_opt) {
								bool prop_val;

								istringstream(boost::to_lower_copy(prop_val_opt.get())) >> std::boolalpha >> prop_val;
								if(query_item.at(0).find('!') != string::npos) {
									prop_val = !prop_val;
								}
								if(prop_val) {
									aux.push_back(child.second);
									accepted_records.insert(child.second.get<string>("name"));
								}
							}
						} else {
							aux.push_back(child.second);
							accepted_records.insert(child.second.get<string>("name"));
						}
					} else {
						if(query_item.at(1) == ocl_equal || query_item.at(1) == ocl_different) {
							string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

							boost::optional prop_val_opt = child.second.get_optional<string>(prop);

							if(prop_val_opt) {
								string prop_val = prop_val_opt.get();

								bool result;
								if(query_item.at(1) == ocl_equal) {
									result = (prop_val == query_item.at(2));
								} else {
									result = (prop_val != query_item.at(2));
								}
								if(result) {
									aux.push_back(child.second);
									accepted_records.insert(child.second.get<string>("name"));
								}
							}
						} else if(query_item.at(1) == ocl_in) {
							bool in_for_ownership = false;
							if(query_item.at(2).find(".") != std::string::npos) {
								string attr_to_search = query_item.at(2);
								std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');

								vector<string> split_query_attr;

								stringstream ss(attr_to_search);
								string tmp;
								while(ss >> tmp) {
									split_query_attr.push_back(tmp);
								}

								if(split_query_attr.at(0) == q.query_var.first) {
									in_for_ownership = true;
								}
							}

							if(in_for_ownership) {
								if(std::count(query_item.at(2).begin(),query_item.at(2).end(),'.') > 1) {
									string too_many_levels_for_ownership_in = "The in operator for ownership relations currently only accepts a variable direct attribute ([var].[attr])";

									throw std::runtime_error(too_many_levels_for_ownership_in);
								}

								string tree_prop = query_item.at(2).substr(query_item.at(2).find(".")+1);
								boost::optional actual_queried_tree_opt = child.second.get_child_optional(tree_prop);

								if(actual_queried_tree_opt) {
									pt::ptree actual_queried_tree = actual_queried_tree_opt.get();

									if(!actual_queried_tree.empty()) {
										BOOST_FOREACH(pt::ptree::value_type& actual_child, actual_queried_tree) {
											string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

											boost::optional prop_val_opt = actual_child.second.get_optional<string>(prop);

											if(prop_val_opt) {
												string prop_val = prop_val_opt.get();

												string attr_to_search = query_item.at(0);
												std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');
												
												vector<string> split_attr;
												
												stringstream ss(attr_to_search);
												string tmp;
												while(ss >> tmp) {
													split_attr.push_back(tmp);
												}

												pt::ptree attr_tree = valid_variables[split_attr.at(0)].second.at(0).get_child(split_attr.at(1));

												string attr_data = attr_tree.data();
												boost::trim(attr_data);
												if(attr_tree.empty() && attr_data != "") {
													vector<string> attr_values;

													stringstream ss(attr_data);
													string tmp;
													while(ss >> tmp) {
														attr_values.push_back(tmp);
													}

													if(std::find(attr_values.begin(), attr_values.end(), prop_val) != attr_values.end()) {
														aux.push_back(child.second);
														accepted_records.insert(child.second.get<string>("name"));
													}
												} else if(!attr_tree.empty() && attr_data == "") {
													BOOST_FOREACH(pt::ptree::value_type val, attr_tree) {
														if(prop_val == val.second.data()) {
															aux.push_back(child.second);
															accepted_records.insert(child.second.get<string>("name"));

															break;
														}
													}
												} else {
													string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

													throw std::runtime_error(bad_condition);
												}
											}
										}
									}
								}
							} else {
								string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

								boost::optional prop_val_opt = child.second.get_optional<string>(prop);

								if(prop_val_opt) {
									string prop_val = prop_val_opt.get();

									string attr_to_search = query_item.at(2);
									std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');
									
									vector<string> split_attr;
									
									stringstream ss(attr_to_search);
									string tmp;
									while(ss >> tmp) {
										split_attr.push_back(tmp);
									}

									/*
										If we have [VAR].[ATTR] in [VAR].[ÁTTR] we search in the ptree

										If we have [VAR].[ATTR] in [VAR], where VAR is a collection variable, we search in the variable value
									*/
									if(split_attr.size() == 1) {
										if(holds_alternative<pair<vector<string>,string>>(gm_var_map[split_attr.at(0)])) {
											pair<vector<string>,string> var_value_and_type = std::get<pair<vector<string>,string>>(gm_var_map[split_attr.at(0)]);

											if(std::find(var_value_and_type.first.begin(), var_value_and_type.first.end(), prop_val) != var_value_and_type.first.end()) {
												aux.push_back(child.second);
												accepted_records.insert(child.second.get<string>("name"));
											}
										} else {
											string query_statement_non_collection_var_error = "Wrong query statement in Goal " + get_node_name(gm[node_id].text) + ". Usage of in statement in non-collection variable.";

											throw std::runtime_error(query_statement_non_collection_var_error);
										}
									} else if(split_attr.size() == 2) {
										// Here we need to get the query ptree for the second attribute
										pt::ptree attr_tree = valid_variables[split_attr.at(0)].second.at(0).get_child(split_attr.at(1));

										string attr_data = attr_tree.data();
										boost::trim(attr_data);
										if(attr_tree.empty() && attr_data != "") {
											vector<string> attr_values;

											stringstream ss(attr_data);
											string tmp;
											while(ss >> tmp) {
												attr_values.push_back(tmp);
											}

											if(std::find(attr_values.begin(), attr_values.end(), prop_val) != attr_values.end()) {
												aux.push_back(child.second);
												accepted_records.insert(child.second.get<string>("name"));
											}
										} else if(!attr_tree.empty() && attr_data == "") {
											BOOST_FOREACH(pt::ptree::value_type val, attr_tree) {
												if(prop_val == val.second.data()) {
													aux.push_back(child.second);
													accepted_records.insert(child.second.get<string>("name"));

													break;
												}
											}
										} else {
											string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

											throw std::runtime_error(bad_condition);
										}
									} else {
										string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

										throw std::runtime_error(bad_condition);
									}
								}
							}
						} else if(query_item.at(1) == ocl_gt || query_item.at(1) == ocl_lt || query_item.at(1) == ocl_geq || query_item.at(1) == ocl_leq) {
							string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

							boost::optional prop_val_opt = child.second.get_optional<string>(prop);

							if(prop_val_opt) {
								string prop_val = prop_val_opt.get();

								std::regex integer("[0-9]+");

								bool result = false;
								if(std::regex_match(query_item.at(2), integer)) {
									int q_val = stoi(query_item.at(2));

									if(prop_val.find(".") == string::npos) {
										int val = stoi(prop_val);

										if(query_item.at(1) == ocl_gt) {
											result = (val > q_val);
										} else if(query_item.at(1) == ocl_lt) {
											result = (val < q_val);
										} else if(query_item.at(1) == ocl_geq) {
											result = (val >= q_val);
										} else if(query_item.at(1) == ocl_leq) {
											result = (val <= q_val);
										}
									} else {
										float val = static_cast<float>(::atof(prop_val.c_str()));

										if(query_item.at(1) == ocl_gt) {
											result = greater_than_float_and_int(q_val, val);
										} else if(query_item.at(1) == ocl_lt) {
											result = greater_than_int_and_float(q_val, val);
										} else if(query_item.at(1) == ocl_geq) {
											result = !greater_than_int_and_float(q_val, val);
										} else if(query_item.at(1) == ocl_leq) {
											result = !greater_than_float_and_int(q_val, val);
										}
									}
								} else {
									float q_val = static_cast<float>(::atof(query_item.at(2).c_str()));

									if(prop_val.find(".") == string::npos) {
										int val = stoi(prop_val);

										if(query_item.at(1) == ocl_gt) {
											result = greater_than_int_and_float(val, q_val);
										} else if(query_item.at(1) == ocl_lt) {
											result = greater_than_float_and_int(val, q_val);
										} else if(query_item.at(1) == ocl_geq) {
											result = !greater_than_float_and_int(val, q_val);
										} else if(query_item.at(1) == ocl_leq) {
											result = !greater_than_int_and_float(val, q_val);
										}
									} else {
										float val = static_cast<float>(::atof(prop_val.c_str()));

										if(query_item.at(1) == ocl_gt) {
											result = greater_than_floats(val, q_val);
										} else if(query_item.at(1) == ocl_lt) {
											result = greater_than_floats(q_val, val);
										} else if(query_item.at(1) == ocl_geq) {
											result = !greater_than_floats(q_val, val);
										} else if(query_item.at(1) == ocl_leq) {
											result = !greater_than_floats(val, q_val);
										}
									}
								}

								if(result) {
									aux.push_back(child.second);
									accepted_records.insert(child.second.get<string>("name"));
								}
							}
						}
					}
				}
			}

			/*string var_name = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).first;
			string var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).second;

			valid_variables[var_name] = make_pair(var_type,aux);
						
			string gm_var_type = parse_gm_var_type(var_type);
			if(gm_var_type == "VALUE") {
				//We assume everything has a name attribute
				gm_var_map[var_name] = make_pair(aux.at(0).get<string>("name"),var_type); 
			} else if(gm_var_type == "COLLECTION") {
				vector<string> var_value;
				for(pt::ptree t : aux) {
					var_value.push_back(t.get<string>("name"));
				}

				gm_var_map[var_name] = make_pair(var_value,var_type);
			}*/
		}

		return make_pair(aux,accepted_records);
	}
}