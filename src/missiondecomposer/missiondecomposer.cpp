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
bool MissionDecomposer::check_context_dependency(int parent_node, int context_node, Context context, map<string, variant<string,vector<string>>> instantiated_vars, 
													vector<SemanticMapping> semantic_mapping) {
	/*
		Go through the goal model and verify effects of abstract tasks that are children of it

		-> If any effects corresponds to the given context we link this node with the node that has the context
	*/
	pair<bool,pair<string,predicate_definition>> var_and_pred = get_pred_from_context(context, semantic_mapping);
	
	vector<int> visited_nodes;

	bool is_sequential = false;
	if(mission_decomposition[context_node].node_type == OP) {
		string node_content = std::get<string>(mission_decomposition[context_node].content);

		if(node_content == sequential_op) {
			is_sequential = true;
		}
	}

	bool found_at = recursive_context_dependency_checking(parent_node, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, false, is_sequential);

	return found_at;
}

// Are the ids in the mission_decomposition or in the GM?
bool MissionDecomposer::recursive_context_dependency_checking(int current_node, int context_node, pair<bool,pair<string,predicate_definition>> var_and_pred, map<string, variant<string,vector<string>>> instantiated_vars, vector<SemanticMapping> semantic_mapping,
																vector<int>& visited_nodes, bool parallel_checking, bool is_sequential) {	
	/*
		-> This recursion will go from bottom to top

		- Stop conditions:
			* If the context condition involves a forAll iterated var we stop when we reach the Achieve Goal that controls this variable
			* If we check all children of the root of the GM

		-> Idea: Search in children of parent_node if this is not an OR decomposition. 
			- We have two possibilities:
				* If the current parent is an OR decomposition we just call the function on its parent
				* If the current parent is an AND decomposition, we search the children. If we find an Operator or Goal that was not visited, we call the function on them. If we
				find a task we check if it solves the context.
			
			- NOTE: We only search the children of parallel decompositions (if it is not generated by a forAll expression (?)) 

			- If we find the task that solves the context condition we create the context dependency 
	*/
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
		int node_gm_id = find_gm_node_by_id(std::get<string>(mission_decomposition[current_node].content), gm);

		vector<pair<string,string>> controlled_vars;
		if(gm[node_gm_id].custom_props.find(controls_prop) != gm[node_gm_id].custom_props.end()) {
			controlled_vars = std::get<vector<pair<string,string>>>(gm[node_gm_id].custom_props[controls_prop]);
		}

		bool finished_search = false;
		for(pair<string,string> controlled_var : controlled_vars) {
			if(controlled_var.first == var_and_pred.second.first) { // Current parent node is a the forAll goal that controls the variable in the condition
				if(is_or) {
					return false;
				} else {
					finished_search = true;
				}
			}
		}

		bool found_at = false;
		ATGraph::out_edge_iterator ei, ei_end;
		for(boost::tie(ei,ei_end) = out_edges(current_node,mission_decomposition);ei != ei_end;++ei) {
			int target = boost::target(*ei,mission_decomposition);
			ATEdge e = mission_decomposition[*ei];

			if(e.edge_type == NORMALAND && std::find(visited_nodes.begin(),visited_nodes.end(),target) == visited_nodes.end()) {
				found_at = recursive_context_dependency_checking(target, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, parallel_checking, is_sequential);

				visited_nodes.push_back(target);
			}
		}

		if(found_at || finished_search) {
			return found_at;
		} else {
			visited_nodes.push_back(current_node);

			return recursive_context_dependency_checking(mission_decomposition[current_node].parent, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, false, is_sequential);
		}
	}

	if(is_or) {
		if(current_node != 0) {
			visited_nodes.push_back(current_node);

			return recursive_context_dependency_checking(mission_decomposition[current_node].parent, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, false, false);
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
					found_at = recursive_context_dependency_checking(target, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, true, false);

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
					found_at = recursive_context_dependency_checking(target, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, true, true);

					visited_nodes.push_back(target);
				}

				if(found_at) break;
			}
		}

		if(found_at) {
			return found_at;
		} else {
			visited_nodes.push_back(current_node);

			return recursive_context_dependency_checking(mission_decomposition[current_node].parent, context_node, var_and_pred, instantiated_vars, semantic_mapping, visited_nodes, false, is_sequential);
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
			for(task t : path.decomposition) {
				for(literal eff : t.eff) {
					bool instantiated_eff = true;
					vector<variant<pair<string,string>,pair<string,vector<string>>>> arg_map;
					for(string arg : eff.arguments) {
						bool found_arg = false;
						variant<string,vector<string>> mapped_var;
						// Here one place where we have to expand collection related predicates
						for(pair<pair<variant<vector<string>,string>,string>,string> var_map : at.variable_mapping) {
							if(arg == var_map.second) {
								found_arg = true;
								if(holds_alternative<string>(var_map.first.first)) {
									mapped_var = std::get<string>(var_map.first.first);
								} else {
									mapped_var = std::get<vector<string>>(var_map.first.first);
								}
								break;
							}
						}

						if(!found_arg) {
							instantiated_eff = false;
							break;
						}

						if(holds_alternative<string>(mapped_var)) {
							arg_map.push_back(make_pair(arg,std::get<string>(mapped_var)));
						} else {
							arg_map.push_back(make_pair(arg,std::get<vector<string>>(mapped_var)));
						}
					}

					if(instantiated_eff) {
						vector<ground_literal> inst_eff;

						for(variant<pair<string,string>,pair<string,vector<string>>> arg_inst : arg_map) {
							if(inst_eff.size() == 0) {
								ground_literal e;
								e.positive = eff.positive;
								e.predicate = eff.predicate;

								inst_eff.push_back(e);
							}

							if(holds_alternative<pair<string,string>>(arg_inst)) {
								pair<string,string> inst = std::get<pair<string,string>>(arg_inst);
								for(ground_literal& e : inst_eff) {
									e.args.push_back(inst.second);
								}
							} else {
								pair<string,vector<string>> inst = std::get<pair<string,vector<string>>>(arg_inst);

								vector<ground_literal> new_inst_eff;
								for(ground_literal e : inst_eff) {
									for(unsigned int index = 0; index < inst.second.size(); index++) {
										ground_literal aux = e;
										aux.args.push_back(inst.second.at(index));

										new_inst_eff.push_back(aux);
									}
								}

								inst_eff = new_inst_eff;
							}
						}

						for(ground_literal e : inst_eff) {
							bool effect_applied = false;
							for(ground_literal& state : world_state_copy) {
								bool same_predicate = is_same_predicate(state, e);

								if(same_predicate && (eff.positive != state.positive)) {
									state.positive = eff.positive;
									effect_applied = true;
									break;
								}
							}

							if(!effect_applied) {
								world_state_copy.push_back(e);
							}
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
				bool same_predicate = is_same_predicate(state, context_pred);

				if(same_predicate && (context_pred.positive == state.positive)) {
					context_satisfied = true;
					break;
				}
			}

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

/*
    Function: create_execution_constraint_edges
    Objective: Create execution constraint edges with the current node (if they do not exist)

	@ Input 1: The ID of the node being evaluated
    @ Output: Void. The task graph is modified
*/
void MissionDecomposer::create_execution_constraint_edges(int node_id) {
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

			if(mission_decomposition[edge.first].edge_type == NORMALAND || mission_decomposition[edge.first].edge_type == NORMALOR) {
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

	recursive_at_graph_build(-1, gmannot, false, gm_vars_map, world_db, semantic_mapping, instantiated_vars);

	final_context_dependency_links_generation();

	if(is_unique_branch(mission_decomposition)) {
		vector<size_t> achieve_goals;
		
		ATGraph::vertex_iterator i, end;

		for(boost::tie(i,end) = vertices(mission_decomposition); i != end; ++i) {
			if(mission_decomposition[*i].is_achieve_type && mission_decomposition[*i].node_type == GOALNODE) {
				achieve_goals.push_back(*i);
			}
		}

		for(size_t goal : achieve_goals) {
			int parent = mission_decomposition[goal].parent;

			if(parent != -1) {
				ATNode aux;
				aux = mission_decomposition[goal];

				aux.node_type = OP;
				aux.content = "#";

				int aux_id = boost::add_vertex(aux, mission_decomposition);
				
				ATGraph::out_edge_iterator ei, ei_end;

				for(boost::tie(ei,ei_end) = out_edges(parent,mission_decomposition);ei != ei_end;++ei) {
					ATEdge e = mission_decomposition[*ei];

					if(e.target == goal && (e.edge_type == NORMALAND || e.edge_type == NORMALOR)) {
						auto edge = boost::edge(parent,goal,mission_decomposition).first;

						boost::remove_edge(edge, mission_decomposition);
					}	
				}

				ATEdge e;
				e.edge_type = NORMALAND;
				e.source = parent;
				e.target = aux_id;

				boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(aux_id, mission_decomposition), e, mission_decomposition);

				ATEdge e2;
				e2.edge_type = NORMALAND;
				e2.source = aux_id;
				e2.target = goal;

				boost::add_edge(boost::vertex(aux_id, mission_decomposition), boost::vertex(goal, mission_decomposition), e2, mission_decomposition);

				ATNode* v = &mission_decomposition[goal];
				v->parent = aux_id;
			} else {
				ATNode aux;
				aux = mission_decomposition[goal];

				aux.node_type = OP;
				aux.content = "#";

				int aux_id = boost::add_vertex(aux, mission_decomposition);

				ATEdge e;
				e.edge_type = NORMALAND;
				e.source = aux_id;
				e.target = goal;

				boost::add_edge(boost::vertex(aux_id, mission_decomposition), boost::vertex(goal, mission_decomposition), e, mission_decomposition);

				ATNode* v = &mission_decomposition[goal];
				v->parent = aux_id;
			}
		}
	}

	return mission_decomposition;
}

/*
    Function: recursive_at_graph_build
    Objective: Call the recursive Task Graph building structure, which generates an ATGraph object. This graph is the
	graph of all possible combinations of tasks

	@ Input 1: The ID of the parent of the current node
	@ Input 2: The current node annotation object
	@ Input 3: A boolean flag indicating if the current node is involved in execution constraints
	@ Input 4: The map between OCL goal model variables and HDDL variables
	@ Input 5: The world knowledge as a ptree object
	@ Input 6: The semantic mappings vector
	@ Input 7: A map of the instantiated OCL variables at this level of the recursion
    @ Output: Void. The ATGraph object is built
*/
void FileKnowledgeMissionDecomposer::recursive_at_graph_build(int parent, general_annot* rannot, bool non_coop, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_vars_map, 
                                                				pt::ptree world_db, vector<SemanticMapping> semantic_mapping, map<string, variant<string,vector<string>>> instantiated_vars) {
	ATNode node;
	int node_id;

	if(rannot->type == OPERATOR || rannot->type == MEANSEND) {
		bool active_context = true;
		Context context;

		VertexData gm_node;

		bool is_forAll = false;
		bool is_achieve = false;
		pair<string,string> monitored_var;
		pair<string,string> controlled_var;

		if(rannot->related_goal != "") { 
			string n_id = rannot->related_goal;
			int gm_node_id = find_gm_node_by_id(n_id, gm);

			gm_node = gm[gm_node_id];
			if(gm_node.custom_props.find(context_prop) != gm_node.custom_props.end()) {
				context = std::get<Context>(gm_node.custom_props[context_prop]);

				if(context.get_context_type() == condition_context_type) {
					active_context = check_context(context, world_state, semantic_mapping, instantiated_vars);
				}
			}

			if(std::get<string>(gm_node.custom_props[goal_type_prop]) == achieve_goal_type) {
				is_achieve = true;
			}
		} else {
			/*
				If we do not have a related goal this means that we have an operator generated by a forAll operator

				- variables are of the form <var_name,var_type>
			*/
			string n_id = rannot->children.at(0)->related_goal;
			int gm_node_id = find_gm_node_by_id(n_id, gm);

			gm_node = gm[gm_node_id];
	
			is_forAll = true;
			monitored_var = std::get<vector<pair<string,string>>>(gm_node.custom_props[monitors_prop]).at(0);
			controlled_var = std::get<vector<pair<string,string>>>(gm_node.custom_props[controls_prop]).at(0);
		}
		
		node.non_coop = rannot->non_coop;
		if(parent != -1) {
			if(!mission_decomposition[parent].group) {
				node.group = false;
				node.divisible = mission_decomposition[parent].divisible;
			} else {
				if(!mission_decomposition[parent].divisible) {
					if(rannot->group) {
						node.group = true;
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
		if(rannot->children.size() == 0 || rannot->type == MEANSEND) {
			node.node_type = GOALNODE;
		} else {
			node.node_type = OP;
		}
		node.content = rannot->content;
		node.parent = parent;
		node.is_achieve_type = is_forAll || is_achieve;

		int achieve_goalnode_id = -1;
		if(is_achieve && node.node_type == OP) {
			ATNode new_achieve_node = node;

			new_achieve_node.node_type = GOALNODE;
			new_achieve_node.content = rannot->related_goal;

			node.non_coop = false;
			node.group = true;
			node.divisible = true;

			achieve_goalnode_id = boost::add_vertex(new_achieve_node, mission_decomposition);

			node.parent = achieve_goalnode_id;
			node.is_achieve_type = false;
		}

		node_id = boost::add_vertex(node, mission_decomposition);

		if(achieve_goalnode_id == -1) {
			if(parent != -1) {
				ATEdge e;
				if(rannot->parent->or_decomposition) {
					e.edge_type = NORMALOR;
				} else {
					e.edge_type = NORMALAND;
				}
				e.source = parent;
				e.target = node_id;

				//mission_decomposition[node_id].parent = parent;
				boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(node_id, mission_decomposition), e, mission_decomposition);
			}
		} else {
			if(parent != -1) {
				ATEdge e;
				if(rannot->parent->or_decomposition) {
					e.edge_type = NORMALOR;
				} else {
					e.edge_type = NORMALAND;
				}
				e.source = parent;
				e.target = achieve_goalnode_id;

				//mission_decomposition[node_id].parent = parent;
				boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(achieve_goalnode_id, mission_decomposition), e, mission_decomposition);
			}

			ATEdge e;
			if(rannot->parent->or_decomposition) {
				e.edge_type = NORMALOR;
			} else {
				e.edge_type = NORMALAND;
			}
			e.source = achieve_goalnode_id;
			e.target = node_id;

			boost::add_edge(boost::vertex(achieve_goalnode_id, mission_decomposition), boost::vertex(node_id, mission_decomposition), e, mission_decomposition);
		}
		/*
			- Check if context is active
				- This requires a lot of thinking, how can we do that?
				- We cannot assume that the context will always be of the format [variable].[attribute], or can we?
					- Another valid way would be just a variable name for example
				- If it is an attribute, we have to check for the type of the variable
					- For now, the only valid types will be just one of the location types given in the configuration file
				- We will need to check a SemanticMapping that involves this attribute from this variable type
			- With this information, just search for all the ATASK type nodes and see if one of them leads to that context being active
		*/
		if(!active_context) {
			/*
				This will happen in two cases:
					- If we have a wrong model
					- If we have a parallel decomposition which is not completely parallel since we have a context dependency
			*/
			bool resolved_context = check_context_dependency(parent, node_id, context, instantiated_vars, semantic_mapping);

			if(!resolved_context) {
				string bad_context_err = "COULD NOT RESOLVE CONTEXT FOR NODE: " + gm_node.text;

				throw std::runtime_error(bad_context_err);
			}
		}

		if(!non_coop) {
			non_coop = node.non_coop;
		}
		/*
			If the node is non cooperative (non-group or non-divisible group) we create non coop links between AT's that are children of it.

			-> We just create these links at the last child AT
		*/
		unsigned int child_index = 0;
		if(is_forAll) {
			/*
				- Controlled variable in forAll needs to be of value type
				- Monitored variable in forAll needs to be of container type
			*/
			int value_index = 0;
			for(general_annot* child : rannot->children) {
				pair<vector<string>,string> var_map = std::get<pair<vector<string>,string>>(gm_vars_map[monitored_var.first]);

				instantiated_vars[controlled_var.first] = var_map.first.at(value_index);
				value_index++;

				if(child_index < rannot->children.size()-1) {
					recursive_at_graph_build(node_id, child, false, gm_vars_map, world_db, semantic_mapping, instantiated_vars);
				} else {
					recursive_at_graph_build(node_id, child, non_coop, gm_vars_map, world_db, semantic_mapping, instantiated_vars);
				}
				child_index++;
			}
		} else {
			for(general_annot* child : rannot->children) {
				if(child_index < rannot->children.size()-1) {
					recursive_at_graph_build(node_id, child, false, gm_vars_map, world_db, semantic_mapping, instantiated_vars);
				} else {
					recursive_at_graph_build(node_id, child, non_coop, gm_vars_map, world_db, semantic_mapping, instantiated_vars);
				}
				child_index++;
			}
		}
	} else if(rannot->type == TASK) {
		node.non_coop = true;
		node.node_type = ATASK;
		node.parent = parent;
		node.group = mission_decomposition[parent].group;
		node.divisible = mission_decomposition[parent].divisible;
		
		//Find AT instance that corresponds to this node and put it in the content
		bool found_at = false;
		map<string,vector<AbstractTask>>::iterator at_inst_it;
		for(at_inst_it = at_instances.begin();at_inst_it != at_instances.end();++at_inst_it) {
			string rannot_id, at_id;
			rannot_id = rannot->content.substr(0,rannot->content.find("_"));
			at_id = at_inst_it->second.at(0).id.substr(0,at_inst_it->second.at(0).id.find("_"));
			
			if(at_id == rannot_id) { //If we are dealing with the same task
				for(AbstractTask at : at_inst_it->second) {
					if(at.id == rannot->content) {
						node.content = at;
						found_at = true;
						break;
					}
				}
			}

			if(found_at) break;
		}

		if(!found_at) {
			string at_not_found_error = "Could not find AT " + rannot->content.substr(0,rannot->content.find("_")) + " definition";
			
			throw std::runtime_error(at_not_found_error);
		}

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

		node_id = boost::add_vertex(node, mission_decomposition);

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

		if(non_coop) {
			create_execution_constraint_edges(node_id);
		}

		AbstractTask at = std::get<AbstractTask>(node.content);

		int path_id = 1;
		for(DecompositionPath path : at_decomposition_paths[at.name]) {
			ATNode path_node;
			path_node.node_type = DECOMPOSITION;
			path_node.non_coop = true;
			path_node.parent = parent;

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