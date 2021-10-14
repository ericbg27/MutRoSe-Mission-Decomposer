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
    Function: print_complete_at_paths_info
    Objective: Print abstract tasks complete decomposition paths info in terminal. Complete decomposition paths have methods and abstract tasks, in
	addition to the actions

    @ Input: The abstract tasks complete decomposition paths in a map format
    @ Output: Void. We just print to the terminal
*/ 
void print_complete_at_paths_info(map<string,vector<CompleteDecompositionPath>> at_complete_decomposition_paths) {
	map<string,vector<CompleteDecompositionPath>>::iterator at_paths_it;

	for(at_paths_it = at_complete_decomposition_paths.begin();at_paths_it != at_complete_decomposition_paths.end();++at_paths_it) {
		cout << "Abstract task " << at_paths_it->first << " complete decomposition paths:" << endl;

		for(auto path : at_paths_it->second) {
			cout << "#####################################" << endl;
			cout << "Path: ";
			unsigned int path_index = 0;
			for(auto node : path.decomposition) {
				variant<task,method> n = node.content;
				if(holds_alternative<task>(n)) {
					task t = std::get<task>(n);

					cout << t.name << "(" << node.parent << ")";
				} else {
					method m = std::get<method>(n);

					cout << m.name << "(" << node.parent << ")";
				}

				if(path_index < path.decomposition.size() - 1) {
					cout << " -> ";
				} else {
					cout << endl;
				}

				path_index++;
			}
			cout << endl;
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