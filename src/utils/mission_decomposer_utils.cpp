#include "mission_decomposer_utils.hpp"

#include <iostream>

using namespace std;

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

*/
void final_context_dependency_links_generation(ATGraph& mission_decomposition) {
	auto vertices = boost::vertices(mission_decomposition);
	
	for(auto v_it = vertices.first; v_it != vertices.second; ++v_it) {
		ATGraph::out_edge_iterator ei, ei_end;
		for(boost::tie(ei,ei_end) = out_edges(*v_it,mission_decomposition);ei != ei_end;++ei) {
			int source = boost::source(*ei,mission_decomposition);
            int target = boost::target(*ei,mission_decomposition);
            auto edge = boost::edge(source,target,mission_decomposition).first;

			ATEdge e = mission_decomposition[edge];

			if(e.edge_type == CDEPEND) {
				auto indexmap = boost::get(boost::vertex_index, mission_decomposition);
				auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

				DFSATVisitor vis;
				boost::depth_first_search(mission_decomposition, vis, colormap, target);

				vector<int> vctr = vis.GetVector();

				int current_node = vctr.at(0);
				vctr.erase(vctr.begin());

				while(current_node != 0) {
					if(mission_decomposition[current_node].node_type == ATASK) {
						ATEdge cd_edge;
						cd_edge.edge_type = CDEPEND;
						cd_edge.source = source;
						cd_edge.target = current_node;
						
						boost::add_edge(boost::vertex(source, mission_decomposition), boost::vertex(current_node, mission_decomposition), cd_edge, mission_decomposition);
					}

					current_node = vctr.at(0);
					vctr.erase(vctr.begin());
				}

				boost::remove_edge(edge, mission_decomposition);
			}
		}
	}
}

/*
    Function: instantiate_decomposition_predicates
    Objective: Ground as many predicates as possible for some decomposition of an abstract task

    @ Input 1: The abstract task that generates the decomposition
	@ Input 2: A reference to the decomposition being instantiated
	@ Input 3: The map between OCL goal model variables and HDDL variables
    @ Output: Void. The decomposition predicates are instantiated
*/
void instantiate_decomposition_predicates(AbstractTask at, Decomposition& d, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_vars_map) {
	int task_counter = 1, task_number;

	task_number = d.path.size();

	vector<variant<ground_literal,literal>> combined_effects;

	for(task t : d.path) {
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
						can_ground = false;
						break;
					}
				}

				if(can_ground) {
					vector<ground_literal> inst_prec;
					//inst_prec.positive = prec.positive;
					//inst_prec.predicate = prec.predicate;

					// Here is probably one place where we have to expand collection related predicates
					for(string arg : prec.arguments) {
						for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
							if(arg == var_map.second) {
								if(holds_alternative<string>(var_map.first.first)) {
									ground_literal p;
									p.positive = prec.positive;
									p.predicate = prec.predicate; 
									p.args.push_back(std::get<string>(var_map.first.first));

									inst_prec.push_back(p);
								} else {
									vector<string> prec_vars = std::get<vector<string>>(var_map.first.first);
									for(string var : prec_vars) {
										ground_literal p;
										p.positive = prec.positive;
										p.predicate = prec.predicate; 
										p.args.push_back(var);

										inst_prec.push_back(p);
									}
									//string not_implemented_collection_pred_error = "Collection-related predicates are not supported yet.";
									//throw std::runtime_error(not_implemented_collection_pred_error);
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
				//inst_eff.positive = eff.positive;
				//inst_eff.predicate = eff.predicate;

				// Here is probably one place where we have to expand collection related predicates
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
								//string not_implemented_collection_pred_error = "Collection-related predicates are not supported yet.";
								//throw std::runtime_error(not_implemented_collection_pred_error);
							}
						}
					}
				}

				bool applied_effect = false;
				for(unsigned int i = 0;i < combined_effects.size();i++) {
					if(holds_alternative<ground_literal>(combined_effects.at(i))) {
						ground_literal ceff = std::get<ground_literal>(combined_effects.at(i));
						for(ground_literal e : inst_eff) {
							if(e.predicate == ceff.predicate) {
								bool equal_args = true;
								int index = 0;
								for(auto arg : e.args) {
									if(arg != ceff.args.at(index)) {
										equal_args = false;
										break;
									}
									index++;
								}

								if(equal_args) {
									if(e.positive != ceff.positive) {
										combined_effects.at(i) = e;
									}
									applied_effect = true;
									break;
								}
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
						if(eff.predicate == ceff.predicate) {
							bool equal_args = true;
							int index = 0;
							for(auto arg : eff.arguments) {
								if(arg != ceff.arguments.at(index)) {
									equal_args = false;
									break;
								}
								index++;
							}

							if(equal_args) {
								if(eff.positive != ceff.positive) {
									combined_effects.at(i) = eff;
								}
								applied_effect = true;
								break;
							}
						}
					}
				}

				if(!applied_effect) {
					combined_effects.push_back(eff);
				}
			}
		}

		if(task_counter == task_number) { //Last task defines effects
			d.eff = combined_effects;
		}

		task_counter++;
	}
}

/*
    Function: check_context_dependency
    Objective: Verify context dependencies involving a given node. This is called if the context of some task is not
	valid at the moment we evaluate it, so we go through all of the paths to the left of the Goal Model

    @ Input 1: A reference to Task Graph as an ATGraph object
	@ Input 2: The parent node ID of the node being evaluated 
	@ Input 3: The ID of the node being evaluated
	@ Input 4: The context of the current node
	@ Input 5: The runtime annotation of the current node
	@ Input 6: The current world state
	@ Input 7: The instantiated OCL goal model variables
	@ Input 8: The abstract tasks decomposition paths map
	@ Input 9: The semantic mapping vector
    @ Output: A boolean flag indicating if the context of the node was satisfied with some abstract task
*/
bool check_context_dependency(ATGraph& mission_decomposition, int parent_node, int current_node, Context context, general_annot* rannot, vector<ground_literal> world_state,
								map<string, variant<string,vector<string>>> instantiated_vars, map<string,vector<vector<task>>> at_decomposition_paths,
									vector<SemanticMapping> semantic_mapping) {
	auto indexmap = boost::get(boost::vertex_index, mission_decomposition);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    DFSATVisitor vis;
    boost::depth_first_search(mission_decomposition, vis, colormap, parent_node);

    vector<int> vctr = vis.GetVector();

	/*
		Go through the goal model and verify effects of abstract tasks that are children of it

		-> If any effects corresponds to the given context we link this node with the node that has the context
	*/

	pair<bool,pair<string,predicate_definition>> var_and_pred =  get_pred_from_context(context, semantic_mapping);

	bool found_at = false;
	for(int v : vctr) {
		//Go through the valid paths of an AbstractTask and create ContextDependency links
		if(mission_decomposition[v].node_type == ATASK) {
			AbstractTask at = std::get<AbstractTask>(mission_decomposition[v].content);

			/*
				-> When we have them we need to verify the effects related to the variable in the var_map
				-> If at the end of one decomposition we have the effect that makes the context valid, we need to make a
				ContextDependency link between this task node and the one related to the context

				-> For now we are only considering effects and not conditional effects
			*/
			vector<int> decompositions;

			ATGraph::edge_iterator init, end;

			for(boost::tie(init,end) = edges(mission_decomposition);init != end;++init) {
				int source, target;

				source = (*init).m_source;
				target = (*init).m_target;
				if(source == v && mission_decomposition[target].node_type == DECOMPOSITION) {
					decompositions.push_back(target);
				}
			}
			
			for(int d_id : decompositions) {
				bool context_satisfied = false;
				vector<task> path = std::get<Decomposition>(mission_decomposition[d_id].content).path;
				vector<ground_literal> world_state_copy = world_state;
				for(task t : path) {
					for(literal eff : t.eff) {
						bool instantiated_eff = true;
						vector<pair<string,string>> arg_map;
						for(string arg : eff.arguments) {
							bool found_arg = false;
							string mapped_var;
							// Here is probably one place where we have to expand collection related predicates
							for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
								if(arg == var_map.second) {
									found_arg = true;
									if(holds_alternative<string>(var_map.first.first)) {
										mapped_var = std::get<string>(var_map.first.first);
									} else {
										string not_implemented_collection_pred_error = "Collection-related predicates are not supported yet.";
										throw std::runtime_error(not_implemented_collection_pred_error);
									}
									break;
								}
							}

							if(!found_arg) {
								instantiated_eff = false;
								break;
							}

							arg_map.push_back(make_pair(arg,mapped_var));
						}

						if(instantiated_eff) {
							ground_literal inst_eff;
							inst_eff.positive = eff.positive;
							inst_eff.predicate = eff.predicate;
							for(pair<string,string> arg_inst : arg_map) {
								inst_eff.args.push_back(arg_inst.second);
							}

							bool effect_applied = false;
							for(ground_literal& state : world_state_copy) {
								if(state.predicate == inst_eff.predicate) {
									bool equal_args = true;
									for(unsigned int arg_index = 0;arg_index < state.args.size();arg_index++) {
										if(state.args.at(arg_index) != inst_eff.args.at(arg_index)) {
											equal_args = false;
											break;
										}
									}

									if(equal_args && (eff.positive != state.positive)) {
										state.positive = eff.positive;
										effect_applied = true;
										break;
									}
								}
							}

							if(!effect_applied) {
								world_state_copy.push_back(inst_eff);
							}
						}
					}
				}

				/*
					Build a ground literal from the predicate definition
				*/
				ground_literal context_pred;
				context_pred.predicate = var_and_pred.second.second.name;
				string var_name = std::get<string>(instantiated_vars[var_and_pred.second.first]);
				context_pred.args.push_back(var_name);
				context_pred.positive = var_and_pred.first;

				for(ground_literal state : world_state_copy) {
					if(state.predicate == context_pred.predicate) {
						bool equal_args = true;
						for(unsigned int arg_index = 0;arg_index < state.args.size();arg_index++) {
							if(state.args.at(arg_index) != context_pred.args.at(arg_index)) {
								equal_args = false;
								break;
							}
						}

						if(equal_args && (context_pred.positive == state.positive)) {
							context_satisfied = true;
							break;
						}
					}
				}

				//If context is satisfied, insert a ContextDependency edge
				if(context_satisfied) {
					ATEdge e;
					e.edge_type = CDEPEND;
					e.source = d_id;
					e.target = current_node;

					boost::add_edge(boost::vertex(d_id, mission_decomposition), boost::vertex(current_node, mission_decomposition), e, mission_decomposition);

					cout << "Context satisfied with task " << std::get<Decomposition>(mission_decomposition[d_id].content).id << ": " << at.name << endl;

					//For now we create the dependency between the first AT that satisfies its context
					found_at = true;
				}
			}

			if(found_at) break;
		}
	}

	return found_at;
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
    Function: create_non_coop_edges
    Objective: Create execution constraint edges with the current node (if they do not exist)

    @ Input 1: A reference to the Task Graph as an ATGraph object
	@ Input 2: The ID of the node being evaluated
    @ Output: Void. The task graph is modified
*/
void create_non_coop_edges(ATGraph& mission_decomposition, int node_id) {
	int non_coop_parent_id = -1;
	int current_node = node_id;

	bool is_group = false;
	bool is_divisible = false;
	while(non_coop_parent_id == -1) {
		ATGraph::in_edge_iterator in_begin, in_end;

        //Find out if the parent has non cooperative set to True
        for(boost::tie(in_begin,in_end) = in_edges(current_node,mission_decomposition);in_begin != in_end;++in_begin) {
            auto source = boost::source(*in_begin,mission_decomposition);
            auto target = boost::target(*in_begin,mission_decomposition);
            auto edge = boost::edge(source,target,mission_decomposition);

			if(mission_decomposition[edge.first].edge_type == NORMAL) {
				if(mission_decomposition[source].non_coop) {
					non_coop_parent_id = source;
					is_group = mission_decomposition[source].group;
					is_divisible = mission_decomposition[source].divisible;
					break;
				} else {
					current_node = source;
				}
			}
		}
	}

	set<int> non_coop_task_ids;
	find_non_coop_task_ids(mission_decomposition, non_coop_parent_id, non_coop_task_ids);

	/*
		Do we really need to create Edges in both ways?
	*/
	for(int t1 : non_coop_task_ids) {
		for(int t2 : non_coop_task_ids) {
			if(t1 != t2) {
				ATEdge e1;
				e1.edge_type = NONCOOP;
				e1.source = t1;
				e1.target = t2;
				e1.group = is_group;
				e1.divisible = is_divisible;
				
				bool edge_exists = boost::edge(t1,t2,mission_decomposition).second;
				if(!edge_exists) {
					boost::add_edge(boost::vertex(t1, mission_decomposition), boost::vertex(t2, mission_decomposition), e1, mission_decomposition);
				}

				ATEdge e2;
				e2.edge_type = NONCOOP;
				e2.source = t2;
				e2.target = t1;
				e2.group = is_group;
				e2.divisible = is_divisible;

				edge_exists = boost::edge(t2,t1,mission_decomposition).second;
				if(!edge_exists) {
					boost::add_edge(boost::vertex(t2, mission_decomposition), boost::vertex(t1, mission_decomposition), e2, mission_decomposition);
				}
			}
		}
	}
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
						if(eff.predicate == s.predicate) {
							bool equal_args = true;

							int arg_index = 0;
							for(string arg : s.arguments) {
								if(arg != eff.arguments.at(arg_index)) {
									equal_args = false;
									break;
								}
								arg_index++;
							}

							if(equal_args) {
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
						if(eff.predicate == s.predicate) {
							bool equal_args = true;

							int arg_index = 0;
							for(string arg : s.args) {
								if(arg != eff.args.at(arg_index)) {
									equal_args = false;
									break;
								}
								arg_index++;
							}

							if(equal_args) {
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
					if(eff.predicate == state.predicate) {
						bool equal_args = true;

						int arg_index = 0;
						for(string arg : state.args) {
							if(arg != eff.args.at(arg_index)) {
								equal_args = false;
								break;
							}
							arg_index++;
						}

						if(equal_args) {
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