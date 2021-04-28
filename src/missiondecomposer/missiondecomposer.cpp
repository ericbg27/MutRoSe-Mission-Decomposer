#include "missiondecomposer.hpp"

#include <iostream>

#include <boost/foreach.hpp>

using namespace std;

void MissionDecomposer::set_mission_decomposer_type(mission_decomposer_type mdt) {
	md_type = mdt;
}

void MissionDecomposer::set_world_state(vector<ground_literal> ws) {
	world_state = ws;
}

void MissionDecomposer::set_at_decomposition_paths(map<string,vector<vector<task>>> atpaths) {
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
    Function: build_at_graph
    Objective: Call the recursive Task Graph building structure, which generates an ATGraph object. This graph is the
	graph of all possible combinations of tasks

    @ Input 1: The abstract tasks instances map
	@ Input 2: The abstract tasks decomposition paths map
	@ Input 3: The goal model runtime annotation
	@ Input 4: The goal model as a GMGraph object
	@ Input 5: The initial world state, initialized using the knowledge (robots and world)
	@ Input 6: The map between OCL goal model variables and HDDL variables
	@ Input 7: The world knowledge as a KnowledgeBase object
	@ Input 8: The semantic mappings vector
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

	final_context_dependency_links_generation(mission_decomposition);

	return mission_decomposition;
}

/*
    Function: recursive_at_graph_build
    Objective: Call the recursive Task Graph building structure, which generates an ATGraph object. This graph is the
	graph of all possible combinations of tasks

	@ Input 1: The partial Task Graph as an ATGraph object (this is the mission decomposition)
	@ Input 2: The world state in this level of the recursion
    @ Input 3: The abstract tasks instances map
	@ Input 4: The abstract tasks decomposition paths map
	@ Input 5: The goal model runtime annotation
	@ Input 6: The ID of the parent of the current node
	@ Input 7: The goal model as a GMGraph object
	@ Input 8: A boolean flag indicating if the current node is involved in execution constraints
	@ Input 9: The map between OCL goal model variables and HDDL variables
	@ Input 10: The world knowledge as a ptree object
	@ Input 11: The semantic mappings vector
	@ Input 12: A map of the instantiated OCL variables at this level of the recursion
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
		node.group = rannot->group;
		node.divisible = rannot->divisible;
		if(rannot->children.size() == 0 || rannot->type == MEANSEND) {
			node.node_type = GOALNODE;
		} else {
			node.node_type = OP;
		}
		node.content = rannot->content;
		node.parent = parent;
		node.is_achieve_type = is_forAll || is_achieve;

		node_id = boost::add_vertex(node, mission_decomposition);

		if(parent != -1) {
			ATEdge e;
			e.edge_type = NORMAL;
			e.source = parent;
			e.target = node_id;

			//mission_decomposition[node_id].parent = parent;
			boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(node_id, mission_decomposition), e, mission_decomposition);
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
			bool resolved_context = check_context_dependency(mission_decomposition, parent, node_id, context, rannot, world_state, instantiated_vars, at_decomposition_paths, semantic_mapping);

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

		node_id = boost::add_vertex(node, mission_decomposition);

		ATEdge e;
		e.edge_type = NORMAL;
		e.source = parent;
		e.target = node_id;

		mission_decomposition[node_id].parent = parent;
		
		boost::add_edge(boost::vertex(parent, mission_decomposition), boost::vertex(node_id, mission_decomposition), e, mission_decomposition);

		if(non_coop) {
			create_non_coop_edges(mission_decomposition,node_id);
		}

		AbstractTask at = std::get<AbstractTask>(node.content);

		int path_id = 1;
		for(vector<task> path : at_decomposition_paths[at.name]) {
			ATNode path_node;
			path_node.node_type = DECOMPOSITION;
			path_node.non_coop = true;
			path_node.parent = parent;

			Decomposition d;
			d.id = at.id + "|" + to_string(path_id);
			d.path = path;
			d.at = at;
			instantiate_decomposition_predicates(at,d,gm_vars_map);

			path_id++;

			path_node.content = d;

			int dnode_id = boost::add_vertex(path_node, mission_decomposition);

			ATEdge d_edge;
			d_edge.edge_type = NORMAL;
			d_edge.source = node_id;
			d_edge.target = dnode_id;

			mission_decomposition[dnode_id].parent = node_id;

			boost::add_edge(boost::vertex(node_id, mission_decomposition), boost::vertex(dnode_id, mission_decomposition), d_edge, mission_decomposition);
		}
	}
}

void FileKnowledgeMissionDecomposer::set_fk_manager(FileKnowledgeManager* manager) {
	fk_manager = manager;
}

shared_ptr<MissionDecomposer> MissionDecomposerFactory::create_mission_decomposer(shared_ptr<KnowledgeManager> k_manager, vector<ground_literal> ws, map<string,vector<vector<task>>> atpaths, 
																							map<string,vector<AbstractTask>> atinst, general_annot* gma, GMGraph g) {
	shared_ptr<MissionDecomposer> mission_decomposer;
	
	if(k_manager->get_knowledge_type() == FILEKNOWLEDGE) {
		mission_decomposer = std::make_shared<FileKnowledgeMissionDecomposer>();
		mission_decomposer->set_mission_decomposer_type(FILEMISSIONDECOMPOSER);
	} else {
		string unsupported_manager_type = "Unsupported manager type found";

		throw std::runtime_error(unsupported_manager_type);
	}

	mission_decomposer->set_world_state(ws);
	mission_decomposer->set_at_decomposition_paths(atpaths);
	mission_decomposer->set_at_instances(atinst);
	mission_decomposer->set_gm_annot(gma);
	mission_decomposer->set_gm(g);

	return mission_decomposer;
}