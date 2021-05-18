#include "mission_decomposer_utils.hpp"

#include <iostream>

using namespace std;

/*
    Function: generate_trimmed_at_graph
    Objective: Trim ATGraph by removing operator nodes with only one child

    @ Input 1: The ATGraph object
    @ Output: The trimmed ATGraph object and a map of id's to the actual ATGraph object
*/

pair<ATGraph,map<int,int>> generate_trimmed_at_graph(ATGraph mission_decomposition) {
	ATGraph trimmed_mission_decomposition;

	map<int,int> ids_map, reverse_ids_map;

	ATGraph::vertex_iterator i, end;

	int root = 0;
	for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
		int out_edge_num = 0;
		ATGraph::out_edge_iterator ei, ei_end;

		for(boost::tie(ei,ei_end) = out_edges(*i,mission_decomposition);ei != ei_end;++ei) {
			auto source = boost::source(*ei,mission_decomposition);
			auto target = boost::target(*ei,mission_decomposition);
			auto edge = boost::edge(source,target,mission_decomposition);

			if(mission_decomposition[edge.first].edge_type == NORMAL) {
				out_edge_num++;
			}
		}

		if(out_edge_num > 1) {
			root = *i;
			break;
		}
	}
	
	bool found_root = false;

	for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
		if(int(*i) == root) {
			found_root = true;
		}
		if(!found_root) {
			continue;
		}
		ATGraph::out_edge_iterator ai, a_end;
		ATNode node = mission_decomposition[*i];

		int parent = -1;
		ATNode current_node = node;
		bool found_parent = false;

		if(int(*i) != root) {
			while(!found_parent) {
				if(ids_map.find(current_node.parent) == ids_map.end()) {
					current_node = mission_decomposition[current_node.parent];
				} else {
					parent = current_node.parent;
					found_parent = true;
				}
			}
		}

		if(node.node_type == OP) {
			int out_edge_num = 0;
			ATGraph::out_edge_iterator ei, ei_end;

			//Only insert OP nodes that have more than one outer edge of normal type (more than one child)
			for(boost::tie(ei,ei_end) = out_edges(*i,mission_decomposition);ei != ei_end;++ei) {
				auto source = boost::source(*ei,mission_decomposition);
				auto target = boost::target(*ei,mission_decomposition);
				auto edge = boost::edge(source,target,mission_decomposition);

				if(mission_decomposition[edge.first].edge_type == NORMAL) {
					out_edge_num++;
				}
			}

			if(out_edge_num > 1) {
				if(parent != mission_decomposition[root].parent) {
					node.parent = ids_map[parent];
				} else { 
					node.parent = mission_decomposition[root].parent;
				}
				int node_id = boost::add_vertex(node, trimmed_mission_decomposition);

				if(int(*i) != root) {
					ATEdge e;
					e.edge_type = NORMAL;
					e.source = ids_map[parent];
					e.target = node_id;

					boost::add_edge(boost::vertex(ids_map[parent], trimmed_mission_decomposition), boost::vertex(node_id, trimmed_mission_decomposition), e, trimmed_mission_decomposition);
				}

				ids_map[*i] = node_id;
				reverse_ids_map[node_id] = *i;
					
				for(boost::tie(ai,a_end) = out_edges(*i,mission_decomposition); ai != a_end;++ai) {
					auto source = boost::source(*ai,mission_decomposition);
					auto target = boost::target(*ai,mission_decomposition);
					auto edge = boost::edge(source,target,mission_decomposition);
						
					ATNode a_node = mission_decomposition[target];
					if(parent != mission_decomposition[root].parent) {
						a_node.parent = ids_map[parent];
					} else {
						a_node.parent = mission_decomposition[root].parent;
					}

					if(mission_decomposition[edge.first].edge_type == NORMAL) {
						if(a_node.node_type == ATASK) {
							int task_id = boost::add_vertex(a_node, trimmed_mission_decomposition);
								
							ATEdge e;
							e.edge_type = NORMAL;
							e.source = node_id;
							e.target = task_id;

							reverse_ids_map[task_id] = target;
							reverse_ids_map[a_node.parent] = source;

							boost::add_edge(boost::vertex(node_id, trimmed_mission_decomposition), boost::vertex(task_id, trimmed_mission_decomposition), e, trimmed_mission_decomposition);
						}
					}
				}
			} else {
				for(boost::tie(ai,a_end) = out_edges(*i,mission_decomposition); ai != a_end;++ai) {
					auto source = boost::source(*ai,mission_decomposition);
					auto target = boost::target(*ai,mission_decomposition);
					auto edge = boost::edge(source,target,mission_decomposition);
						
					ATNode a_node = mission_decomposition[target];
					if(parent != mission_decomposition[root].parent) {
						a_node.parent = ids_map[parent];
					} else {
						a_node.parent = mission_decomposition[root].parent;
					}

					if(mission_decomposition[edge.first].edge_type == NORMAL) {
						if(a_node.node_type == ATASK) {
							int task_id = boost::add_vertex(a_node, trimmed_mission_decomposition);
								
							ATEdge e;
							e.edge_type = NORMAL;
							e.source = parent;
							e.target = task_id;

							reverse_ids_map[task_id] = target;
							reverse_ids_map[a_node.parent] = source;

							boost::add_edge(boost::vertex(ids_map[parent], trimmed_mission_decomposition), boost::vertex(task_id, trimmed_mission_decomposition), e, trimmed_mission_decomposition);
						}
					}
				}
			}
		} else if(node.node_type == GOALNODE) {
			for(boost::tie(ai,a_end) = out_edges(*i,mission_decomposition); ai != a_end;++ai) {
				auto source = boost::source(*ai,mission_decomposition);
				auto target = boost::target(*ai,mission_decomposition);
				auto edge = boost::edge(source,target,mission_decomposition);

				ATNode a_node = mission_decomposition[target];
				if(parent != mission_decomposition[root].parent) {
					a_node.parent = ids_map[parent];
				} else {
					a_node.parent = mission_decomposition[root].parent;
				}

				if(mission_decomposition[edge.first].edge_type == NORMAL) {
					if(a_node.node_type == ATASK) {
						int task_id = boost::add_vertex(a_node, trimmed_mission_decomposition);
								
						ATEdge e;
						e.edge_type = NORMAL;
						e.source = parent;
						e.target = task_id;

						reverse_ids_map[task_id] = target;
						reverse_ids_map[a_node.parent] = source;

						boost::add_edge(boost::vertex(ids_map[parent], trimmed_mission_decomposition), boost::vertex(task_id, trimmed_mission_decomposition), e, trimmed_mission_decomposition);
					}
				}
			}
		}
	}

	return make_pair(trimmed_mission_decomposition, reverse_ids_map);
}

/*
    Function: instantiate_decomposition_predicates
    Objective: Ground as many predicates as possible for some decomposition of an abstract task

    @ Input 1: The abstract task that generates the decomposition
	@ Input 2: A reference to the decomposition being instantiated
	@ Input 3: The map between OCL goal model variables and HDDL variables
    @ Output: Void. The decomposition predicates are instantiated
*/
void instantiate_decomposition_predicates(AbstractTask at, Decomposition& d) {
	int task_counter = 1, task_number;

	task_number = d.path.decomposition.size();

	vector<variant<ground_literal,literal>> combined_effects;
	vector<variant<pair<ground_literal,int>,literal>> combined_func_effects; //DEAL WITH LITERALS FOR FUNC EFFECTS

	for(task t : d.path.decomposition) {
		if(task_counter == 1) { //First task defines preconditions
			for(literal prec : t.prec) {
				bool can_ground = true;
				for(string arg : prec.arguments) {
					bool found_arg = false;
					for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
						if(arg == var_map.second) {
							found_arg = true;
							break;
						}
					}

					if(!found_arg) {
						std::cout << "Could not find argument [" << arg << "] for predicate [" << prec.predicate << "]" << std::endl;
						can_ground = false;
						break;
					}
				}

				if(can_ground) {
					vector<ground_literal> inst_prec;

					// Here one place where we have to expand collection related predicates
					for(string arg : prec.arguments) {
						for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
							if(arg == var_map.second) {
								if(holds_alternative<string>(var_map.first.first)) {
									if(inst_prec.size() == 0) {
										ground_literal p;
										p.positive = prec.positive;
										p.predicate = prec.predicate; 
										p.args.push_back(std::get<string>(var_map.first.first));

										if(prec.isComparisonExpression) {
											p.isComparison = true;
											p.comparison_op_and_value = prec.comparison_op_and_value;
										}

										inst_prec.push_back(p);
									} else {
										vector<ground_literal> new_inst_prec;

										for(ground_literal p : inst_prec) {
											p.args.push_back(std::get<string>(var_map.first.first));

											new_inst_prec.push_back(p);
										}

										inst_prec = new_inst_prec;
									}
								} else { // We cannot have function predicates when collection-related variables are used
									vector<string> prec_vars = std::get<vector<string>>(var_map.first.first);
									if(inst_prec.size() == 0) {
										for(string var : prec_vars) {
											ground_literal p;
											p.positive = prec.positive;
											p.predicate = prec.predicate; 
											p.args.push_back(var);

											inst_prec.push_back(p);
										}
									} else {
										vector<ground_literal> new_inst_prec;

										for(ground_literal p : inst_prec) {
											for(string var : prec_vars) {
												ground_literal aux = p;
												aux.args.push_back(var);

												new_inst_prec.push_back(aux);
											}
										}

										inst_prec = new_inst_prec;
									}
								}
							}
						}
					}

					for(ground_literal p : inst_prec) {
						d.prec.push_back(p);
					}
				} else {
					d.prec.push_back(prec);
				}
			}
		}

		//Here we apply the effects
		for(literal eff : t.eff) {
			bool can_ground = true;
			for(string arg : eff.arguments) {
				bool found_arg = false;
				for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
					if(arg == var_map.second) {
						found_arg = true;
						break;
					}
				}

				if(!found_arg) {
					can_ground = false;
					break;
				}
			}
		
			if(can_ground) {
				vector<ground_literal> inst_eff;

				// Here is one place where we have to expand collection related predicates
				for(string arg : eff.arguments) {
					for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
						if(arg == var_map.second) {
							if(holds_alternative<string>(var_map.first.first)) {
								ground_literal e;
	
								e.positive = eff.positive;
								e.predicate = eff.predicate;
								e.args.push_back(std::get<string>(var_map.first.first));		

								inst_eff.push_back(e);
							} else {
								vector<string> eff_vars = std::get<vector<string>>(var_map.first.first);
								for(string var : eff_vars) {
									ground_literal e;
									e.positive = eff.positive;
									e.predicate = eff.predicate;
									e.args.push_back(var);

									inst_eff.push_back(e);
								}
							}
						}
					}
				}

				bool applied_effect = false;
				for(unsigned int i = 0;i < combined_effects.size();i++) {
					if(holds_alternative<ground_literal>(combined_effects.at(i))) {
						ground_literal ceff = std::get<ground_literal>(combined_effects.at(i));
						for(ground_literal e : inst_eff) {
							bool same_predicate = is_same_predicate(e, ceff);
							
							if(same_predicate) {
								if(e.positive != ceff.positive) {
									combined_effects.at(i) = e;
								}
								applied_effect = true;
								break;
							}
						}
					}
				}

				if(!applied_effect) {
					for(ground_literal e : inst_eff) {
						combined_effects.push_back(e);
					}
				}
			} else {
				bool applied_effect = false;
				for(unsigned int i = 0;i < combined_effects.size();i++) {
					if(holds_alternative<literal>(combined_effects.at(i))) {
						literal ceff = std::get<literal>(combined_effects.at(i));
						bool same_predicate = is_same_predicate(eff, ceff);
						
						if(same_predicate) {
							if(eff.positive != ceff.positive) {
								combined_effects.at(i) = eff;
							}
							applied_effect = true;
							break;
						}
					}
				}

				if(!applied_effect) {
					combined_effects.push_back(eff);
				}
			}
		}

		// Here we apply the function effects
		for(literal func_eff : t.costExpression) {
			bool can_ground = true;
			for(string arg : func_eff.arguments) {
				bool found_arg = false;
				for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
					if(arg == var_map.second) {
						found_arg = true;
						break;
					}
				}

				if(!found_arg) {
					can_ground = false;
					break;
				}
			}
		
			if(can_ground) {
				vector<pair<ground_literal,int>> inst_func_eff;

				// Here is one place where we have to expand collection related predicates
				for(string arg : func_eff.arguments) {
					for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
						if(arg == var_map.second) {
							if(holds_alternative<string>(var_map.first.first)) {
								ground_literal e;
								e.positive = func_eff.positive;
								e.predicate = func_eff.predicate;
								e.args.push_back(std::get<string>(var_map.first.first));
								e.isAssignCostChange = func_eff.isAssignCostChangeExpression;

								inst_func_eff.push_back(make_pair(e,func_eff.costValue));
							} else {
								vector<string> eff_vars = std::get<vector<string>>(var_map.first.first);
								for(string var : eff_vars) {
									ground_literal e;
									e.positive = func_eff.positive;
									e.predicate = func_eff.predicate;
									e.args.push_back(var);
									e.isAssignCostChange = func_eff.isAssignCostChangeExpression;

									inst_func_eff.push_back(make_pair(e,func_eff.costValue));
								}
							}
						}
					}
				}

				bool applied_effect = false;
				for(unsigned int i = 0;i < combined_func_effects.size();i++) {
					if(holds_alternative<pair<ground_literal,int>>(combined_func_effects.at(i))) {
						pair<ground_literal,int> ceff = std::get<pair<ground_literal,int>>(combined_func_effects.at(i));
						for(pair<ground_literal,int> e : inst_func_eff) {
							bool same_predicate = is_same_predicate(e.first, ceff.first);

							if(same_predicate) {
								if(e.first.isAssignCostChange) {
									ceff.second = e.second;
								} else {	
									ceff.second += e.second;
								}

								combined_func_effects.at(i) = ceff; 

								applied_effect = true;
								break;
							}
						}
					}
				}

				if(!applied_effect) {
					for(pair<ground_literal,int> e : inst_func_eff) {
						combined_func_effects.push_back(e);
					}
				}
			} else {
				bool applied_effect = false;
				for(unsigned int i = 0;i < combined_func_effects.size();i++) {
					if(holds_alternative<literal>(combined_func_effects.at(i))) {
						literal ceff = std::get<literal>(combined_func_effects.at(i));
						bool same_predicate = is_same_predicate(func_eff, ceff);

						if(same_predicate) {
							if(func_eff.isAssignCostChangeExpression) {
								combined_func_effects.at(i) = func_eff;
							} else {
								ceff.costValue += func_eff.costValue;
								combined_func_effects.at(i) = ceff;
							}
								
							applied_effect = true;
							break;
						}
					}
				}

				if(!applied_effect) {
					combined_func_effects.push_back(func_eff);
				}
			}
		}

		if(task_counter == task_number) { //Last task defines effects
			d.eff = combined_effects;
			d.func_eff = combined_func_effects;
		}

		task_counter++;
	}
}

/*
    Function: find_decompositions
    Objective: Search for all decompositions for a specific node and return them. We must note that only 
	abstract task nodes have decompositions.

    @ Input 1: The Task Graph as an ATGraph object
	@ Input 2: The ID of the node being evaluated
    @ Output: A vector of pairs of ID's and Nodes which are the roots of the decompositions (if any)
*/
vector<pair<int,ATNode>> find_decompositions(ATGraph mission_decomposition, int node_id) {
	vector<pair<int,ATNode>> node_decompositions;
	if(mission_decomposition[node_id].node_type == ATASK) {
		ATGraph::out_edge_iterator ei, ei_end;

		//Insert only the nodes which are linked by outer edges of type NORMAL
		for(boost::tie(ei,ei_end) = out_edges(node_id,mission_decomposition);ei != ei_end;++ei) {
			auto target = boost::target(*ei,mission_decomposition);

			if(mission_decomposition[target].node_type == DECOMPOSITION) {
				node_decompositions.push_back(make_pair(target,mission_decomposition[target]));		
			}
		}
	} else {
		string node_type;

		if(mission_decomposition[node_id].node_type == OP) {
			node_type = "Operation";
		} else if(mission_decomposition[node_id].node_type == DECOMPOSITION) {
			node_type = "Decomposition";
		} else if(mission_decomposition[node_id].node_type == GOALNODE) {
			node_type = "Goal";
		} else {
			node_type = "Unknown";
		}

		cout << "WARNING: Node of type " <<  node_type << " does not have decompositions." << endl;
	}

	return node_decompositions;
}

/*
    Function: find_non_coop_task_ids
    Objective: Find the tasks which are involved in execution constraints with a given task

    @ Input 1: The Task Graph as an ATGraph object
	@ Input 2: The ID of the node being evaluated
	@ Input 3: A reference of a set object of task ID's
    @ Output: Void. The set of task ID's is filled 
*/
void find_non_coop_task_ids(ATGraph mission_decomposition, int node_id, set<int>& task_ids) {
	if(mission_decomposition[node_id].node_type != ATASK) {
		ATGraph::out_edge_iterator ei, ei_end;

		for(boost::tie(ei,ei_end) = out_edges(node_id,mission_decomposition);ei != ei_end;++ei) {
			auto source = boost::source(*ei,mission_decomposition);
			auto target = boost::target(*ei,mission_decomposition);
			auto edge = boost::edge(source,target,mission_decomposition);

			if(mission_decomposition[edge.first].edge_type == NORMAL) {
				find_non_coop_task_ids(mission_decomposition,target,task_ids);
			}
		}
	} else {
		task_ids.insert(node_id);
	}
}

/*
    Function: can_unite_decompositions
    Objective: Here we check if the effects of one decomposition affect the preconditions of another decomposition

    @ Input 1: The first decomposition object
	@ Input 2: The second decomposition object
	@ Input 3: A flag indicating if these decompositions are decompositions of nodes involved in execution constraints
    @ Output: A flag indicating if the decompositions can be united

	NOTES: -> Predicates not present in the effects are considered to be true from the beginning
			-> If tasks are non_coop we cannot assume nothing about the non_instantiated predicates
				- If they are, we can assume everything robot related refers to the same constant(s)
			-> The way we are performing this right now does not seem to be right
				- IDEA: Transform the initial state and then confront it with the preconditions, if we don't have conflicts we are ok
*/
bool can_unite_decompositions(Decomposition d1, Decomposition d2, bool non_coop_nodes) {
	bool can_unite = true;

	vector<variant<ground_literal,literal>> d1_eff = d1.eff;
	vector<variant<ground_literal,literal>> d2_prec = d2.prec;

	if(non_coop_nodes) {
		vector<variant<ground_literal,literal>> initial_state = d2_prec;

		for(auto& state : initial_state) {
			if(holds_alternative<literal>(state)) {
				literal s = std::get<literal>(state);
				int found_eff = -1;

				for(unsigned int i = 0;i < d1_eff.size();i++) {
					if(holds_alternative<literal>(d1_eff.at(i))) {
						literal eff = std::get<literal>(d1_eff.at(i));
						bool same_predicate = is_same_predicate(eff, s);
						
						if(same_predicate) {
							if(s.positive == eff.positive) {
								found_eff = i;
								break;
							} else {
								s.positive = eff.positive;
								found_eff = i;
								state = s;
								break;
							}
						}
					}
				}

				if(found_eff != -1) {
					d1_eff.erase(d1_eff.begin()+found_eff);
				}
			} else {
				ground_literal s = std::get<ground_literal>(state);
				int found_eff = -1;
				for(unsigned int i = 0;i < d1_eff.size();i++) {
					if(holds_alternative<ground_literal>(d1_eff.at(i))) {
						ground_literal eff = std::get<ground_literal>(d1_eff.at(i));
						bool same_predicate = is_same_predicate(eff, s);

						if(same_predicate) {
							if(s.positive == eff.positive) {
								found_eff = i;
								break;
							} else {
								found_eff = i;
								s.positive = eff.positive;
								state = s;
								break;
							}
						}
					}
				}

				if(found_eff != -1) {
					d1_eff.erase(d1_eff.begin()+found_eff);
				}
			}
		}

		int index = 0;
		for(auto prec : d2_prec) {
			if(holds_alternative<literal>(prec)) {
				literal p = std::get<literal>(prec);
				literal p1 = std::get<literal>(initial_state.at(index));

				if(p.positive != p1.positive) {
					can_unite = false;
					break;
				}
			} else {
				ground_literal p = std::get<ground_literal>(prec);
				ground_literal p1 = std::get<ground_literal>(initial_state.at(index));

				if(p.positive != p1.positive) {
					can_unite = false;
					break;
				}
			}

			index++;
		}
	} else {
		vector<ground_literal> initial_state;
		for(auto prec : d2_prec) {
			if(holds_alternative<ground_literal>(prec)) {
				initial_state.push_back(std::get<ground_literal>(prec));
			}
		}

		for(ground_literal& state : initial_state) {
			int found_eff = -1;
			for(unsigned int i = 0;i < d1_eff.size();i++) {
				if(holds_alternative<ground_literal>(d1_eff.at(i))) {
					ground_literal eff = std::get<ground_literal>(d1_eff.at(i));
					bool same_predicate = is_same_predicate(eff, state);
			
					if(same_predicate) {
						if(state.positive == eff.positive) {
							found_eff = i;
							break;
						} else {
							state.positive = eff.positive;
							found_eff = i;
						}
					}
				}
			}

			if(found_eff != -1) {
				d1_eff.erase(d1_eff.begin()+found_eff);
			}
		}

		int index = 0;
		for(auto prec : d2_prec) {
			if(holds_alternative<ground_literal>(prec)) {
				ground_literal p = std::get<ground_literal>(prec);
				if(p.positive != initial_state.at(index).positive) {
					can_unite = false;
					break;
				}

				index++;
			}
		}
	}

	return can_unite;
}

/*
    Function: print_mission_decomposition
    Objective: Print the mission decomposition to a terminal

    @ Input: The mission decomposition as an ATGraph object
    @ Output: Void. We just have an output in a terminal
*/
void print_mission_decomposition(ATGraph mission_decomposition) {
	ATGraph::vertex_iterator i, end;
	ATGraph::adjacency_iterator ai, a_end;

	for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
		ATNode node = mission_decomposition[*i];
		if(holds_alternative<AbstractTask>(node.content)) {
			std::cout << std::get<AbstractTask>(node.content).id << "(" << *i << ")" << "(" << node.parent << ")" << "[AT]";
		} else if(holds_alternative<string>(node.content)) {
			std::cout << std::get<string>(node.content) << "(" << *i << ")" << "(" << node.parent << ")" << "[OP]";
		} else {
			std::cout << std::get<Decomposition>(node.content).id << "(" << *i << ")" << "(" << node.parent << ")" << "[D]";	
		}

		for(boost::tie(ai,a_end) = adjacent_vertices(*i,mission_decomposition); ai != a_end;++ai) {
			ATNode a_node = mission_decomposition[*ai];
			auto e = boost::edge(*i,*ai,mission_decomposition).first;

			ATEdge edge = mission_decomposition[e];
			if(edge.edge_type == NORMAL) {
				std::cout << " ----> ";
			} else if(edge.edge_type == NONCOOP) { 
				std::cout << " -NC-> ";
			} else {
				std::cout << " -CD-> "; 
			}
			if(holds_alternative<AbstractTask>(a_node.content)) {
				std::cout << std::get<AbstractTask>(a_node.content).id << "(" << *ai << ")" << "(" << a_node.parent << ")" << "[AT]" << " ";
			} else if(holds_alternative<string>(a_node.content)) {
				std::cout << std::get<string>(a_node.content) << "(" << *ai << ")" << "(" << a_node.parent << ")" << "[OP]" << " ";
			} else {
				std::cout << std::get<Decomposition>(a_node.content).id << "(" << *ai << ")" << "(" << a_node.parent << ")" << "[D]" << " ";
			}
		}	
		std::cout << std::endl;
	}

	std::cout << std::endl;
}

bool is_unique_branch(ATGraph mission_decomposition) {
    ATGraph::vertex_iterator i, end;

    int changed_root = false;
	for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
		if(mission_decomposition[*i].node_type != ATASK) {
			int out_edge_num = 0;
			ATGraph::out_edge_iterator ei, ei_end;

			for(boost::tie(ei,ei_end) = out_edges(*i,mission_decomposition);ei != ei_end;++ei) {
				auto source = boost::source(*ei,mission_decomposition);
				auto target = boost::target(*ei,mission_decomposition);
				auto edge = boost::edge(source,target,mission_decomposition);

				if(mission_decomposition[edge.first].edge_type == NORMAL) {
					out_edge_num++;
				}
			}

			if(out_edge_num > 1) {
				changed_root = true;
				break;
			}
		}
	}

    changed_root = !changed_root;

    return changed_root;
}