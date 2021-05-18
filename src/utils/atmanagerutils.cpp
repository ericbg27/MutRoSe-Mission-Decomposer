#include "atmanagerutils.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

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
					cout << ") - " << fragment.second.comparison_op_and_value.second << endl;
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

void solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>>& valid_variables, 
							map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map) {
	vector<pt::ptree> aux;
				
	if(!queried_tree.empty()) {
		BOOST_FOREACH(pt::ptree::value_type& child, queried_tree) {
			if(child.first == q.query_var.second) {
				if(q.query.size() == 1) {
					if(q.query.at(0) != "") {
						string prop = q.query.at(0).substr(q.query.at(0).find('.')+1);
						bool prop_val;
						istringstream(boost::to_lower_copy(child.second.get<string>(prop))) >> std::boolalpha >> prop_val;
						if(q.query.at(0).find('!') != string::npos) {
							prop_val = !prop_val;
						}
						if(prop_val) aux.push_back(child.second);
					} else {
						aux.push_back(child.second);
					}
				} else {
					string prop = q.query.at(0).substr(q.query.at(0).find('.')+1);
					string prop_val;
					try {
						prop_val = child.second.get<string>(prop);
					} catch(...) {
						string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

						throw std::runtime_error(bad_condition);
					}

					bool result;
					if(q.query.at(1) == "==") {
						result = (prop_val == q.query.at(2));
					} else {
						result = (prop_val != q.query.at(2));
					}
					if(result) aux.push_back(child.second);
				}
			}
		}

		string var_name = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).first;
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
		}
	}
}

pt::ptree get_query_ptree(GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>> valid_variables, map<int,AchieveCondition> valid_forAll_conditions, map<string,pair<int,QueriedProperty>>& props_to_query,
							map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, pt::ptree world_tree) {
	pt::ptree queried_tree;
	QueriedProperty q = std::get<QueriedProperty>(gm[node_id].custom_props[queried_property_prop]);

	if(q.queried_var == world_db_query_var) {
		queried_tree = world_tree;
	} else {
		bool valid_query = true;
		if(q.queried_var.find(".") == string::npos) {
			if(valid_variables.find(q.queried_var) != valid_variables.end()) {
				if(valid_variables[q.queried_var].second.size() != 1) {
					valid_query = false;
				} else {
					queried_tree = valid_variables[q.queried_var].second.at(0);
				}
			} else {
				bool found_var = false;
				map<int,AchieveCondition>::iterator f_it;
				for(f_it = valid_forAll_conditions.begin();f_it != valid_forAll_conditions.end();++f_it) {
					if(q.queried_var == f_it->second.get_iteration_var()) {
						props_to_query[q.queried_var] = make_pair(node_id,q);
						found_var = true;
						break;
					}
				}

				if(!found_var) {
					valid_query = false;
				}
			}
		} else {
			vector<string> query_attrs;
						
			string queried_var = q.queried_var;
			std::replace(queried_var.begin(), queried_var.end(),'.',' ');

			stringstream ss(queried_var);
			string temp;
			while(ss >> temp) {
				query_attrs.push_back(temp);
			}

			bool found_forAll_var = false;
			map<int,AchieveCondition>::iterator f_it;
			for(f_it = valid_forAll_conditions.begin();f_it != valid_forAll_conditions.end();++f_it) {
				if(query_attrs.at(0) == f_it->second.get_iteration_var()) {
					props_to_query[query_attrs.at(0)] = make_pair(node_id,q);
					found_forAll_var = true;
					break;
				}
			}

			if(!found_forAll_var) {
				pt::ptree var_to_query;
				string var_type;

				bool found_var = false;
				int current_attr = 0;

				if(valid_variables.find(query_attrs.at(0)) != valid_variables.end()) {
					var_type = valid_variables[query_attrs.at(0)].first;
					found_var = true;
				} else if(gm_var_map.find(query_attrs.at(0)) != gm_var_map.end()) {
					if(holds_alternative<pair<string,string>>(gm_var_map[query_attrs.at(0)])) {
						var_type = std::get<pair<string,string>>(gm_var_map[query_attrs.at(0)]).second;
						found_var = true;
					}
				}

				if(!found_var) {
					valid_query = false;
				}					
							
				if(valid_query) {
					BOOST_FOREACH(pt::ptree::value_type& child, world_tree) {
						if(child.first == var_type) {				
							if(child.second.get<string>("name") == valid_variables[query_attrs.at(0)].second.at(0).get<string>("name")) { //Doesn't work for collection variables
								boost::optional<pt::ptree&> attr = child.second.get_child_optional(query_attrs.at(1));
								if(!attr) {
									valid_query = false;
								} else {
									current_attr = 1;
									var_to_query = attr.get();
								}

								break;
							}
						}
					}

					while(current_attr < int(query_attrs.size())-1 && valid_query) {
						boost::optional<pt::ptree&> attr = var_to_query.get_child_optional(query_attrs.at(current_attr+1));
						if(!attr) {
							valid_query = false;
						} else {
							current_attr++;
							var_to_query = attr.get();
						}
					}

					if(valid_query) {
						string queried_var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).second;
										
						string gm_var_type = parse_gm_var_type(queried_var_type);
						if(gm_var_type == "COLLECTION") {
							queried_var_type = queried_var_type.substr(queried_var_type.find("(")+1,queried_var_type.find(")"));
						}

						BOOST_FOREACH(pt::ptree::value_type& child, var_to_query) {
							if(child.first != queried_var_type) {
								valid_query = false;
								break;
							}
						}
					}
				}

				if(valid_query) {
					queried_tree = var_to_query;
				} else { 
					string invalid_query_error = "Invalid query in Goal " + get_node_name(gm[node_id].text);

					throw std::runtime_error(invalid_query_error);
				}	
			}
		}
	}

	return queried_tree;
}