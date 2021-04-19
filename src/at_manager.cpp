#include "at_manager.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "annotmanager.hpp"

using namespace std;

const string world_db_query_var = "location_db";

void ATManager::set_at_manager_type(at_manager_type atm) {
	atm_type = atm;
}

at_manager_type ATManager::get_at_manager_type() {
	return atm_type;
}

/*
    Function: generate_at_instances
    Objective: This function goes through the goal model in a depth-first search maaner. In this GM walk-through
	we use the world knowledge in order to instantiate variables and verify high-level location types. Also, in
	here we evaluate forAll condition, events, etc.
	Short Description: Go through the Goal Model and find out how many tasks must be created (and how many of each).

    @ Input 1: The set of abstract tasks, taken from the HDDL definition
	@ Input 2: The GMGraph that represents the GM
	@ Input 3: The high-level location type (for now only one is accepted)
	@ Input 4: The variable mapping of the GM
	@ Input 5: The variable mappings between HDDL and the Goal Model
    @ Output: The abstract task instances in a map format
*/
map<string,vector<AbstractTask>> FileKnowledgeATManager::generate_at_instances(vector<task> abstract_tasks, GMGraph gm, vector<string> high_level_loc_types,
															map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map,
																vector<VariableMapping> var_mapping) {
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

	/*
		Get the high-level location types
	*/
	set<string> dsl_locations; //Locations that must be declared in the DSL
	BOOST_FOREACH(pt::ptree::value_type& child, world_tree) {
		if(std::find(high_level_loc_types.begin(), high_level_loc_types.end(), child.first) != high_level_loc_types.end()) {
			dsl_locations.insert(child.second.get<string>("name"));
		}
	}

	/*
		Variable initializations
	*/
	map<int,AchieveCondition> valid_forAll_conditions;
	map<string,pair<string,vector<pt::ptree>>> valid_variables;
	map<string,vector<AbstractTask>> at_instances;
	map<string,int> at_ids;
	map<int,vector<string>> valid_events;

	int depth = 0;
	int last_visited = 0;
	int current = 0;
	map<int,int> node_depths;

	bool insert_events = true;

	// This is related to queries that query iteration variables of forAll conditions
	map<string,pair<int,QueriedProperty>> props_to_query;

	for(int v : vctr) {
		current = v;

		/*
			If the last visited vertex is not the same as the current we verify:

			-> Is the last visited vertex the parent of the current node?
				- If so, increase the depth counter, since we are one level deeper

			-> If the last visited vertex is not the node parent we:
				
				- Set the depth as the depth of the parent vertex + 1
					* We know the depth of the parent since it must already have been visited (DFS)
				
				- Erase forAll conditions and events which are not valid anymore
					* These are the ones which depth is greater or equal as the current node depth
				
				- Check if the parent node runtime annotation. If the higher-level node is an operator we:
					* Do not insert events if it is a parallel
					* Insert events if it is a sequential
					* If it isn't, we throw an exception
		*/

		if(last_visited != current) {
			if(gm[current].parent == last_visited) {
				depth++;
			} else {
				depth = node_depths[gm[current].parent] + 1;

				map<int,AchieveCondition>::iterator cond_it;
				vector<int> to_erase;
				for(cond_it = valid_forAll_conditions.begin();cond_it != valid_forAll_conditions.end();++cond_it) { //Find invalid conditions
					if(cond_it->first >= depth) {
						to_erase.push_back(cond_it->first);
					}
				}
				for(auto e : to_erase) { //Erase invalid conditions
					valid_forAll_conditions.erase(valid_forAll_conditions.find(e));
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

				int parent_id = gm[current].parent;
				general_annot* parent_annot;
				string parent_text = gm[parent_id].text;

				parent_annot = retrieve_runtime_annot(parent_text);
				if(parent_annot->type != OPERATOR) {
					string except = "[AT_MANAGER] Invalid runtime annotation for node: " + gm[current].text;
					throw std::runtime_error(except);
				}
				if(parent_annot->content == ";") {
					insert_events = false;
				} else if(parent_annot->content == "#") {
					insert_events = true;
				}
			}
		}

		if(gm[v].type == "istar.Goal") { 
			/*
				If current vertex is a goal in the GM we check its type

				*************************************** QUERY GOALS ***************************************
				If we have a Query goal we:

				-> Get the QueriedProperty obtained from the parsing of the select statement

				-> With the QueriedProperty we go through the world knowledge variables:
					
					- If the type of query var equals type of the variable in the database, check 
					condition (if any)
						
						* If condition has size 1, we have a boolean property or nothing: [prop] or ![prop]
							** If we have nothing, just insert the variable in the aux vector
							** If we have a property, it must be an attribute of the variable in the 
							database
								*** If this property holds for this variable, insert it into the aux vector
						
						* If condition is not of size one we have an expression: [val1] == [val2] or 
						[val1] != [val2]
							* We assume [val1] is an attribute of the variable being considered
							* [val2] must be a constant (at least for now) 
							* If the condition evaluates to true, add the variable to the aux vector
						
					- After the aux vector is filled, we check if it the query_var is of a container type
					in OCL or if it is a single value
						* For container types we only accept Sequence for now
						* With this variable we fill the gm_var_map
				*******************************************************************************************

				************************************** ACHIEVE GOALS **************************************
				If we have an Achieve goal we:

				-> Get its AchieveCondition and add it to valid_forAll conditions if it has a forAll
				statement
				*******************************************************************************************

				After checking the type we verify trigger types CreationCondition attributes. If we find one, 
				we add the event to the valid_events list of the current depth
			*/

			if(std::get<string>(gm[v].custom_props["GoalType"]) == "Query") {
				pt::ptree queried_tree = get_query_ptree(gm, v, valid_variables, valid_forAll_conditions, props_to_query, gm_var_map, world_tree);
				QueriedProperty q = std::get<QueriedProperty>(gm[v].custom_props["QueriedProperty"]);

				solve_query_statement(queried_tree, q, gm, v, valid_variables, gm_var_map);
			} else if(std::get<string>(gm[v].custom_props["GoalType"]) == "Achieve") {
				AchieveCondition a = std::get<AchieveCondition>(gm[v].custom_props["AchieveCondition"]);
				if(a.has_forAll_expr) {
					valid_forAll_conditions[depth] = a;
				}

				vector<pair<string,string>> controlled_vars = std::get<vector<pair<string,string>>>(gm[v].custom_props["Controls"]);
				for(pair<string,string> var : controlled_vars) {
					if(var.first == a.get_iteration_var()) {
						gm_var_map[var.first] = make_pair("",var.second);
					}
				}
			}

			if(gm[v].custom_props.find("CreationCondition") != gm[v].custom_props.end()) {
				Context c = get<Context>(gm[v].custom_props["CreationCondition"]);
				if(c.type == "trigger") {
					valid_events[depth].push_back(c.condition);
					insert_events = true;
				}
			}
		} else if(gm[v].type == "istar.Task") {

			/*
				If we are dealing with a task vertex we:

				1. Get its user-defined ID and text
				2. Verify if its text (name) matches an abstract task in HDDL
				3. Verify if we have valid forAll conditions*
					3.1. If we have valid forAll conditions:
						3.1.1. If the task location_var is equal to the forAll_iteration_var we can create
						multiple instances of the task in multiple locations. With this in mind:
							3.1.1.1. We create the AT with the data from HDDL and the GM. 
						3.1.2. If the task location_var is not equal to the forAll_ieration_var we can also
						create multiple instances but only in one location. With this in mind:
							3.1.2.1. We create the AT with the data from HDDL and the GM
					3.2. If we do not have valid forAll conditions we simply create an AT instance in the
					location defined by the Location attribute of the task (if any)
				4. Verify valid events and insert all of them in the triggering events list of all the generated
				instances for the AT

				* For now: Assume only one forAll condition is allowed for each AT
				** VERIFY IF VARIABLE MAPPINGS ARE STILL USEFUL!
			*/

			pair<string,string> at_def = parse_at_text(gm[v].text);

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

			set<string> mapped_gm_vars;
			map<string,vector<pair<string,pt::ptree>>> query_vals;

			/*
				If we have valid forAll conditions, the AT must have the iteration variable of each of them

				-> The only exception for this is the case where the AT has a location var which is of same type of the unsatisfied forAll condition
			*/
			if(!valid_forAll_conditions.empty()) { // We have valid forAll conditions
				string location_var;
				if(gm[v].custom_props.find("Location") != gm[v].custom_props.end()) {
					location_var = std::get<string>(gm[v].custom_props["Location"]);
				} else {
					location_var = "";
				}

				set<string> solved_forAll_iteration_vars;
				vector<AchieveCondition> unsolved_forAll_conditions;

				map<int,AchieveCondition>::iterator a_it;
				for(a_it = valid_forAll_conditions.begin();a_it != valid_forAll_conditions.end();++a_it) {
					AchieveCondition current_condition = a_it->second;

					if(solved_forAll_iteration_vars.find(current_condition.get_iteration_var()) != solved_forAll_iteration_vars.end()) {
						string duplicated_variable_error = "Could not solve forAll conditions: Duplicate variable " + current_condition.get_iteration_var() + " in task " + at_def.first;

						throw std::runtime_error(duplicated_variable_error); 
					}

					map<string,pair<int,QueriedProperty>>::iterator props_it = props_to_query.find(current_condition.get_iteration_var());
					if(props_it != props_to_query.end()) {
						for(pt::ptree current_val : valid_variables[current_condition.get_iterated_var()].second) {
							vector<string> query_attrs;
						
							string queried_var = props_it->second.second.queried_var;
							std::replace(queried_var.begin(), queried_var.end(),'.',' ');

							stringstream ss(queried_var);
							string temp;
							while(ss >> temp) {
								query_attrs.push_back(temp);
							}	

							pt::ptree query_ptree = current_val;
							for(unsigned int i = 1;i < query_attrs.size();i++) {
								boost::optional<pt::ptree&> attr_ptree = query_ptree.get_child_optional(query_attrs.at(i));
								if(attr_ptree) {
									query_ptree = attr_ptree.get();
 								} else {
									string query_statement_error = "Cannot solve query in Goal" + get_node_name(gm[props_it->second.first].text);

									throw std::runtime_error(query_statement_error);
								}
							}
							query_vals[current_condition.get_iteration_var()].push_back(make_pair(current_val.get<string>("name"),query_ptree));
						}
					}

					if(current_condition.get_iteration_var() == location_var) {
						if(at_instances[at_def.second].size() == 0) {
							AbstractTask at;

							at.id = at_def.first;
							at.name = at_def.second;

							at_instances[at.name].push_back(at);
						}

						vector<AbstractTask> old_instances = at_instances[at_def.second];
						at_instances[at_def.second].clear();

						for(AbstractTask at_inst : old_instances) {
							for(VariableMapping vm : var_mapping) {
								if(vm.get_task_id() == at_def.first && vm.get_gm_var() != current_condition.get_iteration_var()) {
									map<string,vector<pair<string,pt::ptree>>>::iterator qvals_it = query_vals.find(vm.get_gm_var());
									if(qvals_it != query_vals.end()) {
										string q_value;
										for(auto at_vm : at_inst.variable_mapping) {
											if(at_vm.second == vm.get_hddl_var()) {
												if(holds_alternative<string>(at_vm.first.first)) {
													q_value = std::get<string>(at_vm.first.first);
												} else{
													//Error! iteration variables should not be of collection type
												}
												break;
											}
										}
										for(pair<string,pt::ptree> val_tree : qvals_it->second) {
											if(val_tree.first == q_value) {
												map<string,pair<int,QueriedProperty>>::iterator p_it = props_to_query.find(vm.get_gm_var());
												solve_query_statement(val_tree.second, p_it->second.second, gm, p_it->second.first, valid_variables, gm_var_map);
												break;
											}
										}
									}
								}
							}

							for(pt::ptree current_val : valid_variables[current_condition.get_iterated_var()].second) {
								AbstractTask at;
								
								if(at_ids.find(at_inst.id) != at_ids.end()) {
									at_ids[at_inst.id]++;
								} else {
									at_ids[at_inst.id] = 1;
								}

								at.id = at_inst.id + "_" + to_string(at_ids[at_inst.id]);
								at.name = at_inst.name;

								variant<vector<string>,string> loc = current_val.get<string>("name");
								string var_type = get<pair<string,string>>(gm_var_map[location_var]).second;
								at.location = make_pair(loc,make_pair(location_var,var_type));

								at.at = at_hddl_def;
								at.params = at_inst.params;
								at.fixed_robot_num = gm[v].fixed_robot_num;
								if(holds_alternative<int>(gm[v].robot_num)) {
									at.robot_num = get<int>(gm[v].robot_num); 
								} else {
									at.robot_num = get<pair<int,int>>(gm[v].robot_num);
								}
								
								for(VariableMapping var : var_mapping) {
									if(var.get_task_id() == at_def.first) {
										if(var.get_gm_var() == location_var) {
											string var_type = valid_variables[location_var].first;
											string var_value = current_val.get<string>("name");
											at.variable_mapping.push_back(make_pair(make_pair(var_value,var_type),var.get_hddl_var()));

											mapped_gm_vars.insert(var.get_gm_var());
											break;
										} 
									}
								}

								at_instances[at.name].push_back(at);
							}
						}

						solved_forAll_iteration_vars.insert(current_condition.get_iteration_var());
					} else {
						vector<string> params;

						if(gm[v].custom_props.find("Params") != gm[v].custom_props.end()) {
							params = std::get<vector<string>>(gm[v].custom_props["Params"]);
						}

						if(at_instances[at_def.second].size() == 0) {
							AbstractTask at;

							at.id = at_def.first;
							at.name = at_def.second;

							at_instances[at.name].push_back(at);
						}

						if(std::find(params.begin(), params.end(), current_condition.get_iteration_var()) != params.end()) {
							vector<AbstractTask> old_instances = at_instances[at_def.second];
							at_instances[at_def.second].clear();

							for(AbstractTask at_inst : old_instances) {
								for(VariableMapping vm : var_mapping) {
									if(vm.get_task_id() == at_def.first && vm.get_gm_var() != current_condition.get_iteration_var()) {
										map<string,vector<pair<string,pt::ptree>>>::iterator qvals_it = query_vals.find(vm.get_gm_var());
										if(qvals_it != query_vals.end()) {
											string q_value;
											for(auto at_vm : at_inst.variable_mapping) {
												if(at_vm.second == vm.get_hddl_var()) {
													if(holds_alternative<string>(at_vm.first.first)) {
														q_value = std::get<string>(at_vm.first.first);
													} else{
														//Error! iteration variables should not be of collection type
													}
													break;
												}
											}
											for(pair<string,pt::ptree> val_tree : qvals_it->second) {
												if(val_tree.first == q_value) {
													map<string,pair<int,QueriedProperty>>::iterator p_it = props_to_query.find(vm.get_gm_var());
													solve_query_statement(val_tree.second, p_it->second.second, gm, p_it->second.first, valid_variables, gm_var_map);
													break;
												}
											}
										}
									}
								}

								for(pt::ptree current_val : valid_variables[current_condition.get_iterated_var()].second) {
									AbstractTask at;
								
									if(at_ids.find(at_inst.id) != at_ids.end()) {
										at_ids[at_inst.id]++;
									} else {
										at_ids[at_inst.id] = 1;
									}

									at.id = at_inst.id + "_" + to_string(at_ids[at_inst.id]);
									at.name = at_inst.name;
									at.location = at_inst.location;

									at.at = at_hddl_def;
									at.fixed_robot_num = gm[v].fixed_robot_num;
									if(holds_alternative<int>(gm[v].robot_num)) {
										at.robot_num = get<int>(gm[v].robot_num); 
									} else {
										at.robot_num = get<pair<int,int>>(gm[v].robot_num);
									}

									string param_value = current_val.get<string>("name");
									at.params.push_back(param_value);

									
									for(VariableMapping var : var_mapping) {
										if(var.get_task_id() == at_def.first) {
											if(var.get_gm_var() == current_condition.get_iteration_var()) {
												string var_type = valid_variables[current_condition.get_iteration_var()].first;
												string var_value = current_val.get<string>("name");
												at.variable_mapping.push_back(make_pair(make_pair(var_value,var_type),var.get_hddl_var()));

												mapped_gm_vars.insert(var.get_gm_var());
												break;
											} 
										}
									}

									at_instances[at.name].push_back(at);
								}
							}

							solved_forAll_iteration_vars.insert(current_condition.get_iteration_var());
						} else {
							if(unsolved_forAll_conditions.size() == 0) {
								unsolved_forAll_conditions.push_back(current_condition);
							} else {
								string unsolved_forAll_conditions_error = "Cannot solve all forAll conditions in task " + at_def.first;

								throw std::runtime_error(unsolved_forAll_conditions_error);
							}
						}
					}
				}

				assert(unsolved_forAll_conditions.size() <= 1);

				/*
					Solve unsolved forAll condition (if possible). This condition will be related to a location variable that isn't
					an iteration variable of any forAll condition
				*/
				if(unsolved_forAll_conditions.size() > 0) {
					AchieveCondition unsolved_condition = unsolved_forAll_conditions.at(0);
					bool solved_condition = true;

					if(location_var == "") {
						solved_condition = false;
					} else {
						string iteration_var_type = std::get<pair<string,string>>(gm_var_map[unsolved_condition.get_iteration_var()]).second;
						if(std::find(high_level_loc_types.begin(),high_level_loc_types.end(),iteration_var_type) == high_level_loc_types.end()) {
							solved_condition = false;
						}
					}

					if(solved_condition) {
						bool found_location_type_condition = false;
						map<int,AchieveCondition>::iterator forAll_it;
						for(forAll_it = valid_forAll_conditions.begin();forAll_it != valid_forAll_conditions.end();++forAll_it) {
							string iteration_var_type = std::get<pair<string,string>>(gm_var_map[forAll_it->second.get_iteration_var()]).second;
							if(std::find(high_level_loc_types.begin(),high_level_loc_types.end(),iteration_var_type) != high_level_loc_types.end()) {
								if(!found_location_type_condition) {
									found_location_type_condition = true;
								} else {
									string multiple_forall_conditions_error = "Conflict between forAll conditions in task " + at_def.first;

									throw std::runtime_error(multiple_forall_conditions_error);
								}
							}
						}

						if(!found_location_type_condition) {
							solved_condition = false;
						}
					}

					if(!solved_condition) {
						string forAll_condition_not_found_error = "Could not solve forAll condition expansion in task " + at_def.first;

						throw std::runtime_error(forAll_condition_not_found_error);
					} else {
						if(at_instances[at_def.second].size() == 0) {
							AbstractTask at;

							at.id = at_def.first;
							at.name = at_def.second;

							at_instances[at.name].push_back(at);
						}

						vector<AbstractTask> old_instances = at_instances[at_def.second];
						at_instances[at_def.second].clear();

						for(AbstractTask at_inst : old_instances) {
							for(unsigned int i = 0;i < valid_variables[unsolved_condition.get_iterated_var()].second.size();i++) {
								AbstractTask at;
								
								if(at_ids.find(at_inst.id) != at_ids.end()) {
									at_ids[at_inst.id]++;
								} else {
									at_ids[at_inst.id] = 1;
								}

								at.id = at_inst.id + "_" + to_string(at_ids[at_inst.id]);
								at.name = at_inst.name;

								string var_value = std::get<pair<string,string>>(gm_var_map[location_var]).first;
								string var_type = std::get<pair<string,string>>(gm_var_map[location_var]).second;
								at.location = make_pair(var_value,make_pair(location_var,var_type));

								at.at = at_hddl_def;
								at.params = at_inst.params;
								at.fixed_robot_num = gm[v].fixed_robot_num;
								if(holds_alternative<int>(gm[v].robot_num)) {
									at.robot_num = get<int>(gm[v].robot_num); 
								} else {
									at.robot_num = get<pair<int,int>>(gm[v].robot_num);
								}

								for(VariableMapping var : var_mapping) {
									if(var.get_task_id() == at_def.first) {
										if(var.get_gm_var() == location_var) {
											at.variable_mapping.push_back(make_pair(make_pair(var_value,var_type),var.get_hddl_var()));

											mapped_gm_vars.insert(var.get_gm_var());
											break;
										} 
									}
								}

								at_instances[at.name].push_back(at);
							}
						}
					}
				}
				
				vector<pair<pair<variant<vector<string>,string>,string>,string>> additional_var_mappings;
				for(VariableMapping var : var_mapping) {
					if(mapped_gm_vars.find(var.get_gm_var()) == mapped_gm_vars.end()) {
						if(var.get_task_id() == at_def.first) {
							pair<pair<variant<vector<string>,string>,string>,string> new_var_mapping;
							if(valid_variables.find(var.get_gm_var()) != valid_variables.end()) {
								string var_type = valid_variables[var.get_gm_var()].first;
									if(parse_gm_var_type(var_type) == "COLLECTION") {
										vector<string> var_values;
										for(pt::ptree v : valid_variables[var.get_gm_var()].second) {
											var_values.push_back(v.get<string>("name"));
										}
										new_var_mapping = make_pair(make_pair(var_values,var_type),var.get_hddl_var());
									} else {
										string var_value = valid_variables[var.get_gm_var()].second.at(0).get<string>("name");
										new_var_mapping = make_pair(make_pair(var_value,var_type),var.get_hddl_var());
									}
									additional_var_mappings.push_back(new_var_mapping);
							} else { 
								string var_mapping_error = "Could not find variable mapping for task " + at_def.second;

								throw std::runtime_error(var_mapping_error);
							}
						}
					}
				}

				for(AbstractTask at : at_instances[at_def.second]) {
					for(pair<pair<variant<vector<string>,string>,string>,string> var_mapping : additional_var_mappings) {
						at.variable_mapping.push_back(var_mapping);
					}
				}
			} else { // We don't have valid forAll statements
				string location_var;

				if(gm[v].custom_props.find("Location") != gm[v].custom_props.end()) {
					location_var = get<string>(gm[v].custom_props["Location"]);
				} else {
					location_var = "";
				}

				AbstractTask at;
						
				if(at_ids.find(at_def.first) != at_ids.end()) {
					at_ids[at_def.first]++;
				} else {
					at_ids[at_def.first] = 1;
				}

				at.id = at_def.first + "_" + to_string(at_ids[at_def.first]);
				at.name = at_def.second;

				variant<vector<string>,string> loc;
				string var_type;
				if(valid_variables[location_var].second.size() > 1) {
					vector<string> aux;
					for(pt::ptree l : valid_variables[location_var].second) {
						aux.push_back(l.get<string>("name"));
					}
					loc = aux;
					var_type = get<pair<vector<string>,string>>(gm_var_map[location_var]).second;
				} else {
					if(valid_variables[location_var].second.size() == 1) {
						loc = valid_variables[location_var].second.at(0).get<string>("name");
					} else {
						loc = "";
					}
					var_type = get<pair<string,string>>(gm_var_map[location_var]).second;
				}

				at.location = make_pair(loc, make_pair(location_var,var_type));
				at.at = at_hddl_def;
				at.fixed_robot_num = gm[v].fixed_robot_num;
				if(holds_alternative<int>(gm[v].robot_num)) {
					at.robot_num = get<int>(gm[v].robot_num); 
				} else {
					at.robot_num = get<pair<int,int>>(gm[v].robot_num);
				}

				for(VariableMapping var : var_mapping) {
					if(var.get_task_id() == at_def.first) {
						if(valid_variables.find(var.get_gm_var()) != valid_variables.end()) {
							pair<pair<variant<vector<string>,string>,string>,string> new_var_mapping;
							string var_type = valid_variables[var.get_gm_var()].first;
							if(parse_gm_var_type(var_type) == "COLLECTION") {
								vector<string> var_values;
								for(pt::ptree v : valid_variables[var.get_gm_var()].second) {
									var_values.push_back(v.get<string>("name"));
								}
								new_var_mapping = make_pair(make_pair(var_values,var_type),var.get_hddl_var());
							} else {
								string var_value = valid_variables[var.get_gm_var()].second.at(0).get<string>("name");
								new_var_mapping = make_pair(make_pair(var_value,var_type),var.get_hddl_var());
							}
							at.variable_mapping.push_back(new_var_mapping);
						} else {
							string var_mapping_error = "Could not find variable mapping for task " + at.name;
							throw std::runtime_error(var_mapping_error);
						}
					}
				}

				at_instances[at.name].push_back(at);
			}

			if(!valid_events.empty() && insert_events) {
				vector<string> events;
				map<int, vector<string>>::iterator events_it;
				for(events_it = valid_events.begin(); events_it != valid_events.end(); ++events_it) {
					for(string e : events_it->second) {
						events.push_back(e);
					}
				}

				for(AbstractTask& inst : at_instances[at_def.second]) {
					inst.triggering_events = events;
				}
			}
		}

		node_depths[v] = depth;
		last_visited = v;
	}

	return at_instances;
}

void FileKnowledgeATManager::set_fk_manager(FileKnowledgeManager* manager) {
	fk_manager = manager;
}

shared_ptr<ATManager> ATManagerFactory::create_at_manager(shared_ptr<KnowledgeManager> k_manager) {
	if(k_manager->get_knowledge_type() == FILEKNOWLEDGE) {
		shared_ptr<ATManager> f_at_manager = std::make_shared<FileKnowledgeATManager>();
		f_at_manager->set_at_manager_type(ATFILE);

		return f_at_manager;
	} else {
		string unsupported_manager_type = "Unsupported manager type found";

		throw std::runtime_error(unsupported_manager_type);
	}
}

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
void print_at_paths_info(map<string,vector<vector<task>>> at_decomposition_paths) {
	map<string,vector<vector<task>>>::iterator at_paths_it;

	for(at_paths_it = at_decomposition_paths.begin();at_paths_it != at_decomposition_paths.end();++at_paths_it) {
		cout << "Abstract task " << at_paths_it->first << " decomposition paths:" << endl;

		for(auto path : at_paths_it->second) {
			cout << "#####################################" << endl;
			cout << "Path: ";
			for(auto t : path) {
				cout << t.name;
				if(t.name != path.back().name) {
					cout << " -> ";
				} else {
					cout << endl;
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
								string relation_type = std::get<string>(sm.get_prop("relation"));
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

		string var_name = std::get<vector<pair<string,string>>>(gm[node_id].custom_props["Controls"]).at(0).first;
		string var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props["Controls"]).at(0).second;

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
	QueriedProperty q = std::get<QueriedProperty>(gm[node_id].custom_props["QueriedProperty"]);

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
						string queried_var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props["Controls"]).at(0).second;
										
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