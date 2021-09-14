#include "missiondecomposer.hpp"

#include <iostream>

#include <boost/foreach.hpp>

using namespace std;

void MissionDecomposer::set_verbose(bool verb) {
	verbose = verb;
}

void MissionDecomposer::set_mission_decomposer_type(mission_decomposer_type mdt) {
	md_type = mdt;
}

void MissionDecomposer::set_world_state(vector<ground_literal> ws) {
	world_state = ws;
}

void MissionDecomposer::set_world_state_functions(std::vector<std::pair<ground_literal,variant<int,float>>> wsf) {
	world_state_functions = wsf;
}

void MissionDecomposer::set_at_decomposition_paths(map<string,vector<DecompositionPath>> atpaths) {
	at_decomposition_paths = atpaths;
}

void MissionDecomposer::set_at_instances(map<string,vector<AbstractTask>> atinst) {
	at_instances = atinst;
}

void MissionDecomposer::set_gm_annot(general_annot* gma) {
	gmannot = gma;
}

void MissionDecomposer::set_gm(GMGraph g) {
	gm = g;
}

mission_decomposer_type MissionDecomposer::get_mission_decomposer_type() {
	return md_type;
}

void MissionDecomposer::trim_at_graph() {
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

			if(mission_decomposition[edge.first].edge_type == NORMALAND || mission_decomposition[edge.first].edge_type == NORMALOR) {
				out_edge_num++;
			}
		}

		if(out_edge_num > 1) {
			root = int(*i);
			break;
		}
	}
	
	bool found_root = false;

	for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
		int current_vertex_id = int(*i);

		if(current_vertex_id == root) {
			found_root = true;
		}
		if(!found_root) {
			continue;
		}

		ATNode node = mission_decomposition[current_vertex_id];
		if(node.node_type == ATASK || node.node_type == DECOMPOSITION) {
			continue;
		}

		int parent = -1;
		ATNode current_node = node;
		bool found_parent = false;

		if(current_vertex_id != root) {
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

				if(mission_decomposition[edge.first].edge_type == NORMALAND || mission_decomposition[edge.first].edge_type == NORMALOR) {
					out_edge_num++;
				}
			}

			if(out_edge_num > 1) {
				if(parent != -1) {
					node.parent = ids_map[parent];
				} else { 
					node.parent = -1;
				}
				int node_id = boost::add_vertex(node, trimmed_mission_decomposition);

				if(current_vertex_id != root) {
					at_edge_type edge_type = NORMALAND;
					for(boost::tie(ei,ei_end) = out_edges(parent,mission_decomposition);ei != ei_end;++ei) {
						auto source = boost::source(*ei,mission_decomposition);
						auto target = boost::target(*ei,mission_decomposition);
						auto edge = boost::edge(source,target,mission_decomposition);

						if(mission_decomposition[edge.first].edge_type == NORMALAND || mission_decomposition[edge.first].edge_type == NORMALOR) {
							edge_type = mission_decomposition[edge.first].edge_type;
							break;
						}
					}

					ATEdge e;
					e.edge_type = edge_type;
					e.source = ids_map[parent];
					e.target = node_id;

					boost::add_edge(boost::vertex(ids_map[parent], trimmed_mission_decomposition), boost::vertex(node_id, trimmed_mission_decomposition), e, trimmed_mission_decomposition);
				}

				ids_map[current_vertex_id] = node_id;
				reverse_ids_map[node_id] = current_vertex_id;
				
				insert_trimmed_at_graph_node(trimmed_mission_decomposition, current_vertex_id, root, current_vertex_id, ids_map, reverse_ids_map);
			} else {
				insert_trimmed_at_graph_node(trimmed_mission_decomposition, current_vertex_id, root, parent, ids_map, reverse_ids_map);
			}
		} else if(node.node_type == GOALNODE) {
			insert_trimmed_at_graph_node(trimmed_mission_decomposition, current_vertex_id, root, parent, ids_map, reverse_ids_map);
		}
	}

	// Insert non coop and execution constraint edges in the trimmed at graph
	ATGraph::edge_iterator edges_it, edges_end;
	for(boost::tie(edges_it, edges_end) = edges(mission_decomposition); edges_it != edges_end; ++edges_it) {
		ATEdge e = mission_decomposition[*edges_it];

		if(e.edge_type == NONCOOP || e.edge_type == CDEPEND) {
			int t_source, t_target;

			t_source = ids_map[e.source];
			t_target = ids_map[e.target];

			ATEdge te = e;
			te.source = t_source;
			te.target = t_target;

			boost::add_edge(boost::vertex(t_source,trimmed_mission_decomposition), boost::vertex(t_target,trimmed_mission_decomposition), te, trimmed_mission_decomposition);
		}
	}

	mission_decomposition = trimmed_mission_decomposition;
}

void MissionDecomposer::insert_trimmed_at_graph_node(ATGraph& trimmed_mission_decomposition, int node_id, int root, int parent, map<int,int>& ids_map, map<int,int>& reverse_ids_map) {
	int tasks_parent;
	if(parent == node_id) {
		tasks_parent = node_id;
	} else {
		tasks_parent = parent;
	}

	at_edge_type edge_type = NORMALAND;
	ATGraph::out_edge_iterator ei, ei_end;
	for(boost::tie(ei,ei_end) = out_edges(tasks_parent,mission_decomposition); ei != ei_end; ++ei) {
		auto s = boost::source(*ei,mission_decomposition);
		auto t = boost::target(*ei,mission_decomposition);
		auto e = boost::edge(s,t,mission_decomposition);

		if(mission_decomposition[e.first].edge_type == NORMALAND || mission_decomposition[e.first].edge_type == NORMALOR) {
			edge_type = mission_decomposition[e.first].edge_type;
			break;
		}
	}
	
	ATGraph::out_edge_iterator ai, a_end;
	for(boost::tie(ai,a_end) = out_edges(node_id,mission_decomposition); ai != a_end;++ai) {
		auto source = boost::source(*ai,mission_decomposition);
		auto target = boost::target(*ai,mission_decomposition);
		auto edge = boost::edge(source,target,mission_decomposition);

		ATNode a_node = mission_decomposition[target];
		if(tasks_parent != mission_decomposition[root].parent) {
			a_node.parent = ids_map[tasks_parent];
		} else {
			a_node.parent = mission_decomposition[root].parent;
		}

		if(mission_decomposition[edge.first].edge_type == NORMALAND || mission_decomposition[edge.first].edge_type == NORMALOR) {
			if(a_node.node_type == ATASK) {
				int task_id = boost::add_vertex(a_node, trimmed_mission_decomposition);

				ATEdge e;
				e.edge_type = edge_type;
				e.source = tasks_parent;
				e.target = task_id;

				ids_map[target] = task_id;
				ids_map[source] = a_node.parent; 

				reverse_ids_map[task_id] = target;
				reverse_ids_map[a_node.parent] = source;

				boost::add_edge(boost::vertex(ids_map[tasks_parent], trimmed_mission_decomposition), boost::vertex(task_id, trimmed_mission_decomposition), e, trimmed_mission_decomposition);
			
				ATGraph::out_edge_iterator di, di_end;
				for(boost::tie(di,di_end) = out_edges(target,mission_decomposition); di != di_end;++di) {
					auto d_target = boost::target(*di,mission_decomposition);
					auto d_edge = boost::edge(target,d_target,mission_decomposition);

					ATEdge de = mission_decomposition[d_edge.first];
					if(de.edge_type != CDEPEND && de.edge_type != NONCOOP) {
						if(mission_decomposition[d_target].node_type == DECOMPOSITION) {
							ATNode decomposition = mission_decomposition[d_target];
							decomposition.parent = task_id;

							int decomposition_id = boost::add_vertex(decomposition, trimmed_mission_decomposition);

							ids_map[d_target] = decomposition_id;
							reverse_ids_map[decomposition_id] = d_target;
							
							ATEdge decomposition_edge = de;
							decomposition_edge.source = task_id;
							decomposition_edge.target = decomposition_id;

							boost::add_edge(boost::vertex(task_id, trimmed_mission_decomposition), boost::vertex(decomposition_id, trimmed_mission_decomposition), decomposition_edge, trimmed_mission_decomposition);
						}
					}
				}
			}
		}
	}
}
/*
    Function: final_context_dependency_links_generation
    Objective: Generate context dependency links only between tasks and not between tasks and goals

    @ Output: None. Context Dependency links are removed and others are added
*/
void MissionDecomposer::final_context_dependency_links_generation() {
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

				while(current_node != 0) {
					if(mission_decomposition[current_node].node_type == ATASK) {
						ATEdge cd_edge;
						cd_edge.edge_type = CDEPEND;
						cd_edge.source = source;
						cd_edge.target = current_node;
						
						boost::add_edge(boost::vertex(source, mission_decomposition), boost::vertex(current_node, mission_decomposition), cd_edge, mission_decomposition);
					}

					if(vctr.size() == 0) {
						break;
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
    Function: check_context_dependency
    Objective: Verify context dependencies involving a given node. This is called if the context of some task is not
	valid at the moment we evaluate it, so we go through all of the paths to the left of the Goal Model

	@ Input 1: The parent node ID of the node being evaluated 
	@ Input 2: The ID of the node being evaluated
	@ Input 3: The context of the current node
	@ Input 4: The runtime annotation of the current node
	@ Input 5: The instantiated OCL goal model variables
	@ Input 6: The semantic mapping vector
    @ Output: A boolean flag indicating if the context of the node was satisfied with some abstract task
*/
bool MissionDecomposer::check_context_dependency(int parent_node, int context_node, Context context, map<string, variant<pair<string,string>,pair<vector<string>,string>>> vars_map, vector<SemanticMapping> semantic_mapping) {
	/*
		Go through the goal model and verify effects of abstract tasks that are children of it

		-> If any effects corresponds to the given context we link this node with the node that has the context
	*/
	//pair<bool,pair<string,predicate_definition>> var_and_pred = get_pred_from_context(context, semantic_mapping);
	ConditionExpression* inactive_ctx_predicates = context.get_inactive_predicates(vars_map, world_state, semantic_mapping);
	
	vector<int> visited_nodes;

	bool is_sequential = false;
	if(mission_decomposition[context_node].node_type == OP) {
		string node_content = std::get<string>(mission_decomposition[context_node].content);

		if(node_content == sequential_op) {
			is_sequential = true;
		}
	}

	map<string,variant<string,vector<string>>> instantiated_vars;

	map<string, variant<pair<string,string>,pair<vector<string>,string>>>::iterator vars_map_it;
	for(vars_map_it = vars_map.begin(); vars_map_it != vars_map.end(); ++vars_map_it) {
		if(holds_alternative<pair<string,string>>(vars_map_it->second)) {
			instantiated_vars[vars_map_it->first] = std::get<pair<string,string>>(vars_map_it->second).first;
		} else {
			instantiated_vars[vars_map_it->first] = std::get<pair<vector<string>,string>>(vars_map_it->second).first;
		}
	}

	bool found_at = recursive_context_dependency_checking(parent_node, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, false, is_sequential);

	return found_at;
}

/*
	-> This recursion will go from bottom to top

	- Stop conditions:
		* If we check all children of the root of the GM

	-> Idea: Search in children of parent_node if this is not an OR decomposition. 
		- We have two possibilities:
			* If the current parent is an OR decomposition we just call the function on its parent
			* If the current parent is an AND decomposition, we search the children. If we find an Operator or Goal that was not visited, we call the function on them. If we
			find a task we check if it solves the context.
		
		- NOTE: We only search the children of parallel decompositions (if it is not generated by a forAll expression (?)) 

		- If we find the task that solves the context condition we create the context dependency 
*/
bool MissionDecomposer::recursive_context_dependency_checking(int current_node, int context_node, ConditionExpression* inactive_ctx_predicates, map<string, variant<string,vector<string>>> instantiated_vars, vector<SemanticMapping> semantic_mapping,
																vector<int>& visited_nodes, bool parallel_checking, bool is_sequential) {	
	if(current_node == -1) {
		return false;
	}
	
	bool is_or = false;
	ATGraph::out_edge_iterator ei, ei_end;
	for(boost::tie(ei,ei_end) = out_edges(current_node,mission_decomposition);ei != ei_end;++ei) {
		ATEdge e = mission_decomposition[*ei];

		if(e.edge_type == NORMALOR) {
			is_or = true;

			break;
		}
	}

	if(mission_decomposition[current_node].node_type == GOALNODE) {
		bool found_at = false;

		ATGraph::out_edge_iterator ei, ei_end;
		for(boost::tie(ei,ei_end) = out_edges(current_node,mission_decomposition);ei != ei_end;++ei) {
			int target = boost::target(*ei,mission_decomposition);
			ATEdge e = mission_decomposition[*ei];

			if(e.edge_type == NORMALAND && std::find(visited_nodes.begin(),visited_nodes.end(),target) == visited_nodes.end()) {
				found_at = recursive_context_dependency_checking(target, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, parallel_checking, is_sequential);

				visited_nodes.push_back(target);
			}
		}

		if(found_at) {
			return found_at;
		} else {
			visited_nodes.push_back(current_node);

			return recursive_context_dependency_checking(mission_decomposition[current_node].parent, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, false, is_sequential);
		}
	}

	if(is_or) {
		if(current_node != 0) {
			visited_nodes.push_back(current_node);

			return recursive_context_dependency_checking(mission_decomposition[current_node].parent, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, false, false);
		} else {
			return false;
		}
	}

	// Here are the checkings if we have an AND decomposition, must check if we have sequential
	if(mission_decomposition[current_node].node_type == OP) {
		string node_content = std::get<string>(mission_decomposition[current_node].content);

		bool found_at = false;

		if((node_content == parallel_op || (node_content == sequential_op && parallel_checking)) && !is_sequential) {
			ATGraph::out_edge_iterator ei, ei_end;
			for(boost::tie(ei,ei_end) = out_edges(current_node,mission_decomposition);ei != ei_end;++ei) {
				int target = boost::target(*ei,mission_decomposition);
				ATEdge e = mission_decomposition[*ei];

				if(e.edge_type == NORMALAND && target != context_node && (std::find(visited_nodes.begin(), visited_nodes.end(), target) == visited_nodes.end())) {
					found_at = recursive_context_dependency_checking(target, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, true, false);

					visited_nodes.push_back(target);
				}

				if(found_at) break;
			}
		} else {
			ATGraph::out_edge_iterator ei, ei_end;
			for(boost::tie(ei,ei_end) = out_edges(current_node,mission_decomposition);ei != ei_end;++ei) {
				int target = boost::target(*ei,mission_decomposition);
				ATEdge e = mission_decomposition[*ei];

				if(e.edge_type == NORMALAND && target != context_node && (std::find(visited_nodes.begin(), visited_nodes.end(), target) == visited_nodes.end())) {
					found_at = recursive_context_dependency_checking(target, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, true, true);

					visited_nodes.push_back(target);
				}

				if(found_at) break;
			}
		}

		if(found_at) {
			return found_at;
		} else {
			visited_nodes.push_back(current_node);

			return recursive_context_dependency_checking(mission_decomposition[current_node].parent, context_node, inactive_ctx_predicates, instantiated_vars, semantic_mapping, visited_nodes, false, is_sequential);
		}
	} else if(mission_decomposition[current_node].node_type == ATASK) {
		AbstractTask at = std::get<AbstractTask>(mission_decomposition[current_node].content);

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
			if(source == current_node && mission_decomposition[target].node_type == DECOMPOSITION) {
				decompositions.push_back(target);
			}
		}

		bool found_at = false;
		for(int d_id : decompositions) {
			bool context_satisfied = false;
			DecompositionPath path = std::get<Decomposition>(mission_decomposition[d_id].content).path;
			
			vector<ground_literal> world_state_copy = world_state;
			update_world_state(world_state_copy, path, at); // TODO: Is this a MissionDecomposer method?

			vector<pair<ground_literal,variant<int,float>>> world_state_functions_copy; // TODO: Empty for now

			context_satisfied = inactive_ctx_predicates->evaluate_expression(world_state_copy, world_state_functions_copy);

			//If context is satisfied, insert a ContextDependency edge
			if(context_satisfied && !is_sequential) {
				ATEdge e;
				e.edge_type = CDEPEND;
				e.source = d_id;
				e.target = context_node;

				boost::add_edge(boost::vertex(d_id, mission_decomposition), boost::vertex(context_node, mission_decomposition), e, mission_decomposition);

				if(verbose) {
					cout << "Context satisfied with task " << std::get<Decomposition>(mission_decomposition[d_id].content).id << ": " << at.name << endl;
				}

				//For now we create the dependency between the first AT that satisfies its context
				found_at = true;
			} else if(context_satisfied && is_sequential) {
				if(verbose) {
					cout << "Context satisfied with task " << std::get<Decomposition>(mission_decomposition[d_id].content).id << ": " << at.name << endl;
				}
				
				found_at = true;
			}
		}

		return found_at;
	}

	return false;
}

void MissionDecomposer::create_execution_constraint_edges() {
	auto nodes = vertices(mission_decomposition);

    int graph_size = *nodes.second - *nodes.first;

	map<int,pair<bool,bool>> constraint_nodes; // <ID,<GROUP,DIVISIBLE>>
	stack<pair<bool,int>> inactive_constraint_branches;

	pair<bool,int> active_constraint_branch = make_pair(false,-1);

	set<int> current_active_tasks;
	map<int,set<int>> nodes_active_tasks;

	set<int> or_nodes;

	for(int current_node = 0; current_node < graph_size; current_node++) {
		bool is_group = mission_decomposition[current_node].group;
		bool is_divisible = mission_decomposition[current_node].divisible;

		int parent = mission_decomposition[current_node].parent;

		if(active_constraint_branch.first) {
			if(parent <= mission_decomposition[active_constraint_branch.second].parent) {
				if(inactive_constraint_branches.size() == 0) {
					current_active_tasks.clear();

					active_constraint_branch.first = false;
				} else {
					active_constraint_branch = inactive_constraint_branches.top();
					
					inactive_constraint_branches.pop();
				}
			} else if((mission_decomposition[parent].is_achieve_type && active_constraint_branch.second >= parent) || (or_nodes.find(parent) != or_nodes.end())) { 
				current_active_tasks = nodes_active_tasks[parent];
			}
		}

		bool is_or = false;
		ATGraph::out_edge_iterator ei, ei_end;
		for(boost::tie(ei,ei_end) = out_edges(current_node,mission_decomposition);ei != ei_end;++ei) {
			ATEdge e = mission_decomposition[*ei];

			if(e.edge_type == NORMALOR) {
				is_or = true;
				or_nodes.insert(current_node);

				break;
			}
		}

		if(mission_decomposition[current_node].is_achieve_type || is_or) {
			nodes_active_tasks[current_node] = current_active_tasks;
		}

		if(mission_decomposition[current_node].node_type == GOALNODE || mission_decomposition[current_node].node_type == OP) {
			if(!is_group || (is_group && !is_divisible)) {
				if(!active_constraint_branch.first) {
					constraint_nodes[current_node] = make_pair(is_group,is_divisible);

					active_constraint_branch.first = true;
					active_constraint_branch.second = current_node;
				} else {
					pair<bool,bool> active_constraint = constraint_nodes[active_constraint_branch.second];

					// If active constraint is a group and the one just found isn't, replace active constraint
					if(active_constraint.first) {
						if(!is_group) {
							constraint_nodes[current_node] = make_pair(is_group,is_divisible);
							constraint_nodes[active_constraint_branch.second] = make_pair(is_group,is_divisible); // Change the current constraint due to constraint priority

							inactive_constraint_branches.push(active_constraint_branch);
							active_constraint_branch = make_pair(true,current_node);
						}
					}
				}
			}
		} else if(mission_decomposition[current_node].node_type == ATASK) {
			if(active_constraint_branch.first) {
				pair<bool,bool> active_constraint = constraint_nodes[active_constraint_branch.second];

				if(current_active_tasks.size() > 0) {
					for(int task : current_active_tasks) {
						AbstractTask t1 = std::get<AbstractTask>(mission_decomposition[current_node].content);
						AbstractTask t2 = std::get<AbstractTask>(mission_decomposition[task].content);

						ATEdge e1;
						e1.edge_type = NONCOOP;
						e1.source = task;
						e1.target = current_node;
						e1.group = active_constraint.first;
						e1.divisible = active_constraint.second;

						bool edge_exists = boost::edge(task,current_node,mission_decomposition).second;
						if(!edge_exists) {
							boost::add_edge(boost::vertex(task, mission_decomposition), boost::vertex(current_node, mission_decomposition), e1, mission_decomposition);
						}

						ATEdge e2;
						e2.edge_type = NONCOOP;
						e2.target = task;
						e2.source = current_node;
						e2.group = active_constraint.first;
						e2.divisible = active_constraint.second;

						edge_exists = boost::edge(current_node,task,mission_decomposition).second;
						if(!edge_exists) {
							boost::add_edge(boost::vertex(current_node, mission_decomposition), boost::vertex(task, mission_decomposition), e2, mission_decomposition);
						}
					}
				}

				current_active_tasks.insert(current_node);
			}
		}
	}
}

void MissionDecomposer::group_and_divisible_attrs_instantiation(int parent, ATNode& node, general_annot* rannot) { 
	if(parent != -1) {
		if(!mission_decomposition[parent].group) {
			node.group = false;
			node.divisible = mission_decomposition[parent].divisible;
		} else {
			if(!mission_decomposition[parent].divisible) {
				if(rannot->group) {
					node.group = true;
					node.divisible = false;
				} else {
					node.group = rannot->group;
					node.divisible = false;
				}
			} else {
				node.group = rannot->group;
				node.divisible = rannot->divisible;
			}
		}
	} else {
		node.group = rannot->group;
		node.divisible = rannot->divisible;
	}
}

int MissionDecomposer::add_goal_op_node(ATNode& node, general_annot* rannot, int parent, bool is_forAll, bool is_achieve) {
	node.non_coop = rannot->non_coop;
	group_and_divisible_attrs_instantiation(parent, node, rannot);
	if(rannot->type == MEANSEND) {
		node.node_type = GOALNODE;
	} else {
		node.node_type = OP;
	}
	node.content = rannot->content;
	node.parent = parent;
	node.is_achieve_type = is_forAll;
	if(is_forAll) {
		node.achieve_goal_id = rannot->children.at(0)->related_goal;
	}

	int node_id = boost::add_vertex(node, mission_decomposition);

	if(parent != -1) {
		ATEdge e;
		if(rannot->parent->or_decomposition) {
			e.edge_type = NORMALOR;
		} else {
			e.edge_type = NORMALAND;
		}
		e.source = parent;
		e.target = node_id;

		boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(node_id, mission_decomposition), e, mission_decomposition);
	}

	return node_id;
}

int MissionDecomposer::add_task_node(ATNode& node, general_annot* rannot, int parent) {
	node.non_coop = true;
	node.node_type = ATASK;
	node.parent = parent;
	node.group = mission_decomposition[parent].group;
	node.divisible = mission_decomposition[parent].divisible;

	bool non_group_task_error = false;
	AbstractTask abstract_task = std::get<AbstractTask>(node.content);
	if(!node.group && !abstract_task.fixed_robot_num) {
		non_group_task_error = true;
	} else if(!node.group && abstract_task.fixed_robot_num && std::get<int>(abstract_task.robot_num) != 1) {
		non_group_task_error = true;
	}

	if(non_group_task_error) {
		string non_group_task_robot_num_error = "Non group task [" + abstract_task.id + "] does not have 1 robot in its declaration";

		throw std::runtime_error(non_group_task_robot_num_error);
	}

	int node_id = boost::add_vertex(node, mission_decomposition);

	ATEdge e;
	if(rannot->parent->or_decomposition) {
		e.edge_type = NORMALOR;
	} else {
		e.edge_type = NORMALAND;
	}
	e.source = parent;
	e.target = node_id;

	mission_decomposition[node_id].parent = parent;
	
	boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(node_id, mission_decomposition), e, mission_decomposition);

	return node_id;
}

bool MissionDecomposer::find_node_at(ATNode& node, general_annot* rannot) {
	bool found_at = false;

	map<string,vector<AbstractTask>>::iterator at_inst_it;
	for(at_inst_it = at_instances.begin();at_inst_it != at_instances.end();++at_inst_it) {
		for(AbstractTask at : at_inst_it->second) {
			if(at.id == rannot->content) { //If we are dealing with the same task
				node.content = at;
				found_at = true;
				break;
			}
		}

		if(found_at) break;
	}

	return found_at;
}

void MissionDecomposer::add_decomposition_path_nodes(ATNode node, int node_id) {
	AbstractTask at = std::get<AbstractTask>(node.content);

	int path_id = 1;
	for(DecompositionPath path : at_decomposition_paths[at.name]) {
		ATNode path_node;
		path_node.node_type = DECOMPOSITION;
		path_node.non_coop = true;

		Decomposition d;
		d.id = at.id + "|" + to_string(path_id);
		d.path = path;
		d.at = at;
		instantiate_decomposition_predicates(at,d,verbose);

		path_id++;

		path_node.content = d;

		int dnode_id = boost::add_vertex(path_node, mission_decomposition);

		ATEdge d_edge;
		d_edge.edge_type = NORMALAND;
		d_edge.source = node_id;
		d_edge.target = dnode_id;

		mission_decomposition[dnode_id].parent = node_id;

		boost::add_edge(boost::vertex(node_id, mission_decomposition), boost::vertex(dnode_id, mission_decomposition), d_edge, mission_decomposition);
	}
}

void FileKnowledgeMissionDecomposer::set_fk_manager(FileKnowledgeManager* manager) {
	fk_manager = manager;
}

/*
    Function: build_at_graph
    Objective: Call the recursive Task Graph building structure, which generates an ATGraph object. This graph is the
	graph of all possible combinations of tasks

	@ Input 1: The Goal Model variables map
	@ Input 2: The semantic mappings vector
    @ Output: The Task Graph as an ATGraph object

	REMEMBER: AT's contained in at_instances are mandatory, their decompositions are alternative
	NOTES: -> We can add an ALT operator (or similar name) in order to define operators for this alternative decompositions we can have
			-> Main flow of things:
				- Go through the goal model runtime annotation (gmannot) and create nodes for the tasks, operators and decompositions for the ATGraph
					* Use AT instances to create the task nodes (each one of them will have a different id)
				- For each AT Node in the graph, we will add all of the possible decomposition paths
					* One thing to note is that we need to take into consideration the world state in order to define which are the valid decompositions
			-> A recursive implementation seems to be the best approach
			-> Let's not deal with the OPT or the FALLBACK case for the moment (29/11)
*/ 
ATGraph FileKnowledgeMissionDecomposer::build_at_graph(map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_vars_map, vector<SemanticMapping> semantic_mapping) {
	map<string, variant<string,vector<string>>> instantiated_vars;

	shared_ptr<FileKnowledgeBase> world_knowledge_base = fk_manager->get_world_knowledge();

    pt::ptree world_db;
    if(world_knowledge_base->get_knowledge_file_type() == XML) {
        XMLKnowledgeBase* xml_base = dynamic_cast<XMLKnowledgeBase*>(world_knowledge_base.get());

        world_db = xml_base->get_knowledge();
    }

	recursive_at_graph_build(-1, gmannot, gm_vars_map, world_db, semantic_mapping, instantiated_vars);

	final_context_dependency_links_generation();

	create_execution_constraint_edges();

	if(!is_unique_branch(mission_decomposition)) {
		trim_at_graph();
	}

	return mission_decomposition;
}

/*
    Function: recursive_at_graph_build
    Objective: Call the recursive Task Graph building structure, which generates an ATGraph object. This graph is the
	graph of all possible combinations of tasks

	@ Input 1: The ID of the parent of the current node
	@ Input 2: The current node annotation object
	@ Input 3: The map between OCL goal model variables and HDDL variables
	@ Input 4: The world knowledge as a ptree object
	@ Input 5: The semantic mappings vector
	@ Input 6: A map of the instantiated OCL variables at this level of the recursion
    @ Output: Void. The ATGraph object is built
*/
void FileKnowledgeMissionDecomposer::recursive_at_graph_build(int parent, general_annot* rannot, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_vars_map, 
                                                				pt::ptree world_db, vector<SemanticMapping> semantic_mapping, map<string, variant<string,vector<string>>> instantiated_vars) {
	ATNode node;
	int node_id;

	if(rannot->type == OPERATOR || rannot->type == MEANSEND) { // Here wee simply skip Query goals that are kept in the runtime annotation (their type is GOAL)
		bool active_context = true;
		Context context;

		VertexData gm_node;

		bool is_forAll = false;
		bool is_achieve = false;
		
		pair<string,string> iterated_var;
		pair<string,string> iteration_var;

		if(rannot->related_goal != "") { 
			string n_id = rannot->related_goal;
			int gm_node_id = find_gm_node_by_id(n_id, gm);

			gm_node = gm[gm_node_id];
			if(gm_node.custom_props.find(context_prop) != gm_node.custom_props.end()) {
				context = std::get<Context>(gm_node.custom_props[context_prop]);

				if(context.get_context_type() == condition_context_type) {
					map<string, variant<pair<string,string>,pair<vector<string>,string>>> vars_map;

					map<string, variant<string,vector<string>>>::iterator instantiated_vars_it;
					for(instantiated_vars_it = instantiated_vars.begin(); instantiated_vars_it != instantiated_vars.end(); ++instantiated_vars_it) {
						if(holds_alternative<string>(instantiated_vars_it->second)) {
							string var_type = std::get<pair<string,string>>(gm_vars_map[instantiated_vars_it->first]).second;

							vars_map[instantiated_vars_it->first] = make_pair(std::get<string>(instantiated_vars_it->second),var_type);
						} else {
							string var_type = std::get<pair<vector<string>,string>>(gm_vars_map[instantiated_vars_it->first]).second;

							vars_map[instantiated_vars_it->first] = make_pair(std::get<vector<string>>(instantiated_vars_it->second),var_type);
						}
					}
					active_context = context.check_context(world_state, semantic_mapping, vars_map);
				}
			}

			if(std::get<string>(gm_node.custom_props[goal_type_prop]) == achieve_goal_type) {
				is_achieve = true;

				AchieveCondition a = std::get<AchieveCondition>(gm_node.custom_props[achieve_condition_prop]);

				if(a.has_forAll_expr) {
					for(pair<string,string> var : std::get<vector<pair<string,string>>>(gm_node.custom_props[monitors_prop])) {
						if(var.first == a.get_iterated_var()) {
							iterated_var = var;
							break;
						}
					}
					for(pair<string,string> var : std::get<vector<pair<string,string>>>(gm_node.custom_props[controls_prop])) {
						if(var.first == a.get_iteration_var()) {
							iteration_var = var;
							break;
						}
					}
				}
			}
		} else {
			/*
				If we do not have a related goal this means that we have an operator generated by a forAll operator
			*/
			string n_id = rannot->children.at(0)->related_goal;
			int gm_node_id = find_gm_node_by_id(n_id, gm);

			gm_node = gm[gm_node_id];
	
			is_forAll = true;
			AchieveCondition a = std::get<AchieveCondition>(gm_node.custom_props[achieve_condition_prop]);

			for(pair<string,string> var : std::get<vector<pair<string,string>>>(gm_node.custom_props[monitors_prop])) {
				if(var.first == a.get_iterated_var()) {
					iterated_var = var;
					break;
				}
			}
			for(pair<string,string> var : std::get<vector<pair<string,string>>>(gm_node.custom_props[controls_prop])) {
				if(var.first == a.get_iteration_var()) {
					iteration_var = var;
					break;
				}
			}
		}

		node_id = add_goal_op_node(node, rannot, parent, is_forAll, is_achieve);

		if(!active_context) {
			/*
				This will happen in two cases:
					- If we have a wrong model
					- If we have a parallel decomposition which is not completely parallel since we have a context dependency
			*/
			map<string, variant<pair<string,string>,pair<vector<string>,string>>> vars_map;

			map<string, variant<string,vector<string>>>::iterator instantiated_vars_it;
			for(instantiated_vars_it = instantiated_vars.begin(); instantiated_vars_it != instantiated_vars.end(); ++instantiated_vars_it) {
				if(holds_alternative<string>(instantiated_vars_it->second)) {
					string var_type = std::get<pair<string,string>>(gm_vars_map[instantiated_vars_it->first]).second;

					vars_map[instantiated_vars_it->first] = make_pair(std::get<string>(instantiated_vars_it->second),var_type);
				} else {
					string var_type = std::get<pair<vector<string>,string>>(gm_vars_map[instantiated_vars_it->first]).second;

					vars_map[instantiated_vars_it->first] = make_pair(std::get<vector<string>>(instantiated_vars_it->second),var_type);
				}
			}

			bool resolved_context = check_context_dependency(parent, node_id, context, vars_map, semantic_mapping);

			active_context = resolved_context;
		}

		if(active_context) {
			if(is_forAll) {
				int value_index = 0;
				for(general_annot* child : rannot->children) {
					pair<vector<string>,string> var_map = std::get<pair<vector<string>,string>>(gm_vars_map[iterated_var.first]);

					instantiated_vars[iteration_var.first] = var_map.first.at(value_index);
					value_index++;

					recursive_at_graph_build(node_id, child, gm_vars_map, world_db, semantic_mapping, instantiated_vars);
				}
			} else {
				if(is_achieve) {
					if(gm_vars_map.find(iterated_var.first) != gm_vars_map.end()) {
						if(holds_alternative<pair<vector<string>,string>>(gm_vars_map[iterated_var.first])) {
							pair<vector<string>,string> var_map = std::get<pair<vector<string>,string>>(gm_vars_map[iterated_var.first]);

							if(var_map.first.size() == 1) {
								instantiated_vars[iteration_var.first] = var_map.first.at(0);
							}
						}
					}
				}

				for(general_annot* child : rannot->children) {
					recursive_at_graph_build(node_id, child, gm_vars_map, world_db, semantic_mapping, instantiated_vars);
				}
			}
		} else {
			boost::clear_vertex(node_id, mission_decomposition);
			boost::remove_vertex(node_id, mission_decomposition);
		}
	} else if(rannot->type == TASK) {
		//Find AT instance that corresponds to this node and put it in the content
		bool found_at = find_node_at(node, rannot);

		if(!found_at) {
			string at_not_found_error = "Could not find AT " + rannot->content.substr(0,rannot->content.find("_")) + " definition";
			
			throw std::runtime_error(at_not_found_error);
		}

		node_id = add_task_node(node, rannot, parent);

		add_decomposition_path_nodes(node, node_id);
	}
}

shared_ptr<MissionDecomposer> MissionDecomposerFactory::create_mission_decomposer(shared_ptr<KnowledgeManager> k_manager, vector<ground_literal> ws, vector<pair<ground_literal,variant<int,float>>> wsf, map<string,vector<DecompositionPath>> atpaths, 
																							map<string,vector<AbstractTask>> atinst, general_annot* gma, GMGraph g, bool verb) {
	shared_ptr<MissionDecomposer> mission_decomposer;
	
	if(k_manager->get_knowledge_type() == FILEKNOWLEDGE) {
		mission_decomposer = std::make_shared<FileKnowledgeMissionDecomposer>();
		mission_decomposer->set_mission_decomposer_type(FILEMISSIONDECOMPOSER);
	} else {
		string unsupported_manager_type = "Unsupported manager type found";

		throw std::runtime_error(unsupported_manager_type);
	}

	mission_decomposer->set_world_state(ws);
	mission_decomposer->set_world_state_functions(wsf);
	mission_decomposer->set_at_decomposition_paths(atpaths);
	mission_decomposer->set_at_instances(atinst);
	mission_decomposer->set_gm_annot(gma);
	mission_decomposer->set_gm(g);
	mission_decomposer->set_verbose(verb);

	return mission_decomposer;
}