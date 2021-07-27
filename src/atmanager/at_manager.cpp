#include "at_manager.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "../annotmanager/annotmanager.hpp"

using namespace std;

void ATManager::set_at_manager_type(at_manager_type atm) {
	atm_type = atm;
}

void ATManager::set_abstract_tasks(vector<task> ats) {
	abstract_tasks = ats;
}

void ATManager::set_gm(GMGraph g) {
	gm = g;
}

void ATManager::set_high_level_loc_types(vector<string> hllt) {
	high_level_loc_types = hllt;
}

at_manager_type ATManager::get_at_manager_type() {
	return atm_type;
}

void ATManager::erase_invalid_structures(int depth) {
	map<int,AchieveCondition>::iterator cond_it;
	vector<int> to_erase;
	for(cond_it = valid_forAll_conditions.begin();cond_it != valid_forAll_conditions.end();++cond_it) { //Find invalid conditions
		if(cond_it->first >= depth) {
			to_erase.push_back(cond_it->first);
		}
	}
	for(auto e : to_erase) { //Erase invalid conditions
		valid_forAll_conditions.erase(valid_forAll_conditions.find(e));
		forAll_inst_id.erase(forAll_inst_id.find(e));
	}
	to_erase.clear();

	map<int,vector<string>>::iterator events_it;
	for(events_it = valid_events.begin();events_it != valid_events.end();events_it++) { //Find invalid events
		if(events_it->first >= depth) {
			to_erase.push_back(events_it->first);
		}
	}
	for(auto e : to_erase) { //Erase invalid events
		valid_events.erase(valid_events.find(e));
	}
}

void ATManager::at_id_instantiation(AbstractTask& at, pair<string,string> at_def) {
	at.id = at_def.first;
	if(!forAll_inst_id.empty()) {
		map<int,int>::iterator it;
		for(it = forAll_inst_id.begin(); it != forAll_inst_id.end(); ++it) {
			at.id += "_" + to_string(it->second);
		}
	} else {
		at.id += "_1";
	}
}

void ATManager::robotnum_prop_instantiation(AbstractTask& at, int current_node) {
	at.fixed_robot_num = gm[current_node].fixed_robot_num;
	if(holds_alternative<int>(gm[current_node].robot_num)) {
		at.robot_num = std::get<int>(gm[current_node].robot_num); 
	} else {
		at.robot_num = std::get<pair<int,int>>(gm[current_node].robot_num);
	}
}

void ATManager::location_prop_instantiation(AbstractTask& at, pair<string,string> at_def, int current_node, vector<VariableMapping> var_mapping, 
												map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map) {
	if(gm[current_node].custom_props.find(location_prop) != gm[current_node].custom_props.end()) {
		string location_var;
		location_var = std::get<string>(gm[current_node].custom_props[location_prop]);

		if(gm_var_map.find(location_var) != gm_var_map.end()) {
			variant<pair<string,string>,pair<vector<string>,string>> var_map = gm_var_map[location_var];

			variant<vector<string>,string> var_value;
			string var_type;
			if(holds_alternative<pair<string,string>>(var_map)) {
				pair<string,string> var_value_and_type = std::get<pair<string,string>>(var_map);

				var_value = var_value_and_type.first;
				var_type = var_value_and_type.second;
			} else {
				pair<vector<string>,string> var_value_and_type = std::get<pair<vector<string>,string>>(var_map);

				var_value = var_value_and_type.first;
				var_type = var_value_and_type.second;
			}
			
			at.location = make_pair(var_value, make_pair(location_var,var_type));

			for(VariableMapping var : var_mapping) {
				if(var.get_task_id() == at_def.first) {
					if(var.get_gm_var() == location_var) {
						at.variable_mapping.push_back(make_pair(make_pair(var_value,var_type),var.get_hddl_var()));
					}
				}
			}
		}
	}
}

void ATManager::params_prop_instantiation(AbstractTask& at, pair<string,string> at_def, int current_node, vector<VariableMapping> var_mapping, 
												map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map) {
	if(gm[current_node].custom_props.find(params_prop) != gm[current_node].custom_props.end()) {
		vector<string> params = std::get<vector<string>>(gm[current_node].custom_props["Params"]);

		for(string param : params) {
			if(gm_var_map.find(param) != gm_var_map.end()) {
				if(holds_alternative<pair<string,string>>(gm_var_map[param])) {
					string parameter = std::get<pair<string,string>>(gm_var_map[param]).first;
					at.params.push_back(parameter);
				} else {
					vector<string> parameter = std::get<pair<vector<string>,string>>(gm_var_map[param]).first;
					at.params.push_back(parameter);
				}
			} else {
				string param_not_found_error = "Could not find value for parameter [" + param + "] for task [" + at.name + "]"; 

				throw std::runtime_error(param_not_found_error);
			}
		}

		for(VariableMapping var : var_mapping) {
			if(var.get_task_id() == at_def.first) {
				vector<string>::iterator param_it = std::find(params.begin(), params.end(), var.get_gm_var());
				if(param_it != params.end()) {
					variant<vector<string>,string> var_value;
					string var_type;
					if(holds_alternative<pair<string,string>>(gm_var_map[*param_it])) {
						pair<string,string> var_value_and_type = std::get<pair<string,string>>(gm_var_map[*param_it]);

						var_value = var_value_and_type.first;
						var_type = var_value_and_type.second;
					} else {
						pair<vector<string>,string> var_value_and_type = std::get<pair<vector<string>,string>>(gm_var_map[*param_it]);

						var_value = var_value_and_type.first;
						var_type = var_value_and_type.second;
					}

					at.variable_mapping.push_back(make_pair(make_pair(var_value,var_type),var.get_hddl_var()));
				}
			}
		}
	}
}

void ATManager::events_prop_instantiation(AbstractTask& at, bool insert_events) {
	if(!valid_events.empty() && insert_events) {
		vector<string> events;
		map<int, vector<string>>::iterator events_it;
		for(events_it = valid_events.begin(); events_it != valid_events.end(); ++events_it) {
			for(string e : events_it->second) {
				events.push_back(e);
			}
		}

		at.triggering_events = events;
	}
}

bool ATManager::check_trigger_ctx(int current_node, int depth) {
	bool insert_events = false;

	if(gm[current_node].custom_props.find(context_prop) != gm[current_node].custom_props.end()) {
		Context c = std::get<Context>(gm[current_node].custom_props[context_prop]);
		
		if(c.get_context_type() == trigger_context_type) {
			valid_events[depth].push_back(c.get_condition());
			insert_events = true;
		}
	}

	return insert_events;
}

/*
    Function: generate_at_instances
    Objective: This function initializes variables and calls a recursive function which populates at_instances

	@ Input 1: The variable mapping of the GM
	@ Input 2: The variable mappings between HDDL and the Goal Model
    @ Output: The abstract task instances in a map format
*/
map<string,vector<AbstractTask>> FileKnowledgeATManager::generate_at_instances(map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map, vector<VariableMapping> var_mapping) {
	vector<int> vctr = get_dfs_gm_nodes(gm);

	/*
		Get the world knowledge ptree. We disconsider the root key, if any, since we expect it to be
		just a name like world_db or similar
	*/
	shared_ptr<FileKnowledgeBase> world_knowledge = fk_manager->get_world_knowledge();
	pt::ptree world_tree;
	
	if(world_knowledge->get_knowledge_file_type() == XML) {
		XMLKnowledgeBase* world_db = dynamic_cast<XMLKnowledgeBase*>(world_knowledge.get());

		if(world_db->get_root_key() == "") {
			world_tree = world_db->get_knowledge();
		} else {
			world_tree = world_db->get_knowledge().get_child(world_db->get_root_key());
		}
	}

	map<int,int> node_depths;

	recursive_at_instances_generation(0, 0, node_depths, world_tree, var_mapping, gm_var_map, false);

	return at_instances;
}

/*
    Function: recursive_at_instances_generation
    Objective: This function goes through the goal model in a depth-first search maaner. In this GM walk-through
	we use the world knowledge in order to instantiate variables and verify high-level location types. Also, in
	here we evaluate forAll condition, events, etc.
	Short Description: Go through the Goal Model and find out how many tasks must be created (and how many of each).

	@ Input 1: The ID of the current node
	@ Input 2: The depth of the current node
	@ Input 3: A map of the node depths
	@ Input 4: The world knowledge ptree
	@ Input 5: The variable mappings between HDDL and the Goal Model
	@ Input 6: The variable mapping of the GM
	@ Input 7: A flag which indicates if events need to be inserted
	
    @ Output: Void. The AT instances attribute is populated
*/
void FileKnowledgeATManager::recursive_at_instances_generation(int current, int depth, map<int,int>& node_depths, pt::ptree world_tree, vector<VariableMapping> var_mapping,
																map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map, bool insert_events) {	
	if(gm[current].parent != -1) {
		depth = node_depths[gm[current].parent] + 1;
	}

	erase_invalid_structures(depth);

	general_annot* parent_annot;
	int parent_id = gm[current].parent;

	string parent_text;
	if(parent_id != -1) {
		parent_text = gm[parent_id].text;
	} else {
		parent_text = gm[current].text;
	}

	parent_annot = retrieve_runtime_annot(parent_text);
	if(parent_annot->type != OPERATOR) {
		string except = "[AT_MANAGER] Invalid runtime annotation for node: " + gm[current].text;
			
		throw std::runtime_error(except);
	}
	if(parent_annot->content == sequential_op) {
		insert_events = false;
	} else if(parent_annot->content == parallel_op || parent_annot->content == "") {
		insert_events = true;
	}

	node_depths[current] = depth;

	if(gm[current].type == istar_goal) {
		insert_events = check_trigger_ctx(current, depth);

		string goal_type = std::get<string>(gm[current].custom_props[goal_type_prop]);
		if(goal_type == query_goal_type) {
			query_goal_resolution(current, world_tree, gm_var_map);
		} else if(goal_type == achieve_goal_type) {
			achieve_goal_resolution(current, depth, world_tree, insert_events, node_depths, var_mapping, gm_var_map);
		} else {
			for(int child : gm[current].children) {
				recursive_at_instances_generation(child, depth, node_depths, world_tree, var_mapping, gm_var_map, insert_events);
			}
		}
	} else if(gm[current].type == istar_task) {
		/*
			For tasks we don't need to check forAll conditions, just instantiate them, their variable mappings, etc

			-> All variables have been initialized at this point
		*/
		pair<string,string> at_def = parse_at_text(gm[current].text);

		task at_hddl_def;
		bool found_abstract_task = false;
		for(task abstract_task : abstract_tasks) {
			if(abstract_task.name == at_def.second) {
				at_hddl_def = abstract_task;
				found_abstract_task = true;

				break;
			}
		}
		if(!found_abstract_task) {
			string bad_at_error = "Could not find AT [" + at_def.second + "] in HDDL Domain definition.";

			throw std::runtime_error(bad_at_error);
		}

		AbstractTask at;

		at.name = at_def.second;
		at.at = at_hddl_def;

		at_id_instantiation(at, at_def);
		robotnum_prop_instantiation(at, current);
		location_prop_instantiation(at, at_def, current, var_mapping, gm_var_map);
		params_prop_instantiation(at, at_def, current, var_mapping, gm_var_map);
		events_prop_instantiation(at, insert_events);

		at_instances[at_def.second].push_back(at);
	}
}

void FileKnowledgeATManager::query_goal_resolution(int current_node, pt::ptree world_tree, map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map) {
	pt::ptree queried_tree = get_query_ptree(gm, current_node, valid_variables, valid_forAll_conditions, world_tree);
	QueriedProperty q = std::get<QueriedProperty>(gm[current_node].custom_props[queried_property_prop]);

	solve_query_statement(queried_tree, q, gm, current_node, valid_variables, gm_var_map);
}

void FileKnowledgeATManager::achieve_goal_resolution(int current_node, int depth, pt::ptree world_tree, bool insert_events, map<int,int>& node_depths,
														vector<VariableMapping> var_mapping, map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map) {
	AchieveCondition a = std::get<AchieveCondition>(gm[current_node].custom_props[achieve_condition_prop]);
	if(a.has_forAll_expr) {
		valid_forAll_conditions[depth] = a;
		forAll_inst_id[depth] = 1;

		vector<int> children = gm[current_node].children;
		
		string iterated_var_type;
		vector<pair<string,string>> controlled_vars = std::get<vector<pair<string,string>>>(gm[current_node].custom_props[controls_prop]);
		for(pair<string,string> var : controlled_vars) {
			if(var.first == a.get_iteration_var()) {
				iterated_var_type = var.second;
				break;
			}
		}

		vector<string> iterated_var_value = std::get<pair<vector<string>,string>>(gm_var_map[a.get_iterated_var()]).first;
		for(unsigned int i = 0; i < iterated_var_value.size(); i++) {
			gm_var_map[a.get_iteration_var()] = make_pair(iterated_var_value.at(i),iterated_var_type);
			for(pt::ptree aux : valid_variables[a.get_iterated_var()].second) {
				if(aux.get<string>("name") == iterated_var_value.at(i)) {
					vector<pt::ptree> iterated_var_ptree;
					iterated_var_ptree.push_back(aux);

					valid_variables[a.get_iteration_var()] = make_pair(iterated_var_type,iterated_var_ptree);
					break;
				}
			}
			
			for(int child : children) {
				recursive_at_instances_generation(child, depth, node_depths, world_tree, var_mapping, gm_var_map, insert_events);
			}
			forAll_inst_id[depth] += 1;
		}
	} else {
		for(int child : gm[current_node].children) {
			recursive_at_instances_generation(child, depth, node_depths, world_tree, var_mapping, gm_var_map, insert_events);
		}
	}
}

void FileKnowledgeATManager::set_fk_manager(FileKnowledgeManager* manager) {
	fk_manager = manager;
}

shared_ptr<ATManager> ATManagerFactory::create_at_manager(shared_ptr<KnowledgeManager> k_manager, vector<task> abstract_tasks, GMGraph gm, vector<string> high_level_loc_types) {
	shared_ptr<ATManager> at_manager;
	
	if(k_manager->get_knowledge_type() == FILEKNOWLEDGE) {
		at_manager = std::make_shared<FileKnowledgeATManager>();
		at_manager->set_at_manager_type(ATFILE);
	} else {
		string unsupported_manager_type = "Unsupported manager type found";

		throw std::runtime_error(unsupported_manager_type);
	}

	at_manager->set_abstract_tasks(abstract_tasks);
	at_manager->set_gm(gm);
	at_manager->set_high_level_loc_types(high_level_loc_types);

	return at_manager;
}