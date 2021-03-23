#include "at_manager.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "annotmanager.hpp"


/*
    Function: generate_at_instances
    Objective: This function goes through the goal model in a depth-first search maaner. In this GM walk-through
	we use the world knowledge in order to instantiate variables and verify high-level location types. Also, in
	here we evaluate forAll condition, events, etc.
	Short Description: Go through the Goal Model and find out how many tasks must be created (and how many of each).

    @ Input 1: The set of abstract tasks, taken from the HDDL definition
	@ Input 2: The GMGraph that represents the GM
	@ Input 3: The high-level location type (for now only one is accepted)
	@ Input 4: The KnowledgeBase object representing the world knowledge 
	@ Input 5: The variable mapping of the GM
	@ Input 6: The variable mappings between HDDL and the Goal Model
    @ Output: The abstract task instances in a map format
*/
map<string,vector<AbstractTask>> generate_at_instances(vector<task> abstract_tasks, GMGraph gm, string location_type,
														KnowledgeBase world_db, map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map,
															vector<VariableMapping> var_mapping) {
	/*	
		-> HERE WE ARE ASSUMING THE FIRST MONITORED VARIABLE REPRESENTS THE LOCATION. DEAL WITH THIS LATER!
	*/

	/*
		Ideas:
			- Search through nodes in the order given by depth-first search and verify valid
			variables and valid conditions. If the goal we are visiting is in the same or in 
			higher depth than the goal which holds some condition, than the condition is not
			valid anymore. Variables just need to be seen somewhere in order for them to be
			valid. One note about variables is that if they aren't controlled by some other
			goal, we have to search in the local database (which is the default database). In
			a real-world case study, this types of variables would only be worth something if
			some goal was already assigned to a robot, so in this initial parsing they are not
			used (but are considered). Conditions, aside from context conditions, must be evaluated
			on variables that are controlled by some goal (in a higher-level than the goal which uses
			it or in the same level and the left)
	*/
	std::vector<int> vctr = get_dfs_gm_nodes(gm);

	/*
		Get the world knowledge ptree. We disconsider the root key, if any, since we expect it to be
		just a name like world_db or similar
	*/
	pt::ptree world_tree;
	if(world_db.get_root_key() == "") {
		world_tree = world_db.get_knowledge();
	} else {
		world_tree = world_db.get_knowledge().get_child(world_db.get_root_key());
	}

	/*
		Get the high-level location type

		-> TODO: We declare a set of locations since we expect to be able to have multiple
		high-level location types
	*/
	set<string> dsl_locations; //Locations that must be declared in the DSL
	BOOST_FOREACH(pt::ptree::value_type& child, world_tree) {
		if(child.first == location_type) {
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
				std::string parent_text = gm[parent_id].text;

				parent_annot = retrieve_runtime_annot(parent_text);
				if(parent_annot->type != OPERATOR) {
					std::string except = "[AT_MANAGER] Invalid runtime annotation for node: " + gm[current].text;
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

				*******************************************************************************************
			*/
			if(std::get<string>(gm[v].custom_props["GoalType"]) == "Query") {
				vector<pt::ptree> aux;
				QueriedProperty q = std::get<QueriedProperty>(gm[v].custom_props["QueriedProperty"]);

				BOOST_FOREACH(pt::ptree::value_type& child, world_tree) {
					if(child.first == q.query_var.second) {
						if(q.query.size() == 1) {
							if(q.query.at(0) != "") {
								string prop = q.query.at(0).substr(q.query.at(0).find('.')+1);
								bool prop_val;
								istringstream(boost::to_lower_copy(child.second.get<string>(prop))) >> std::boolalpha >> prop_val;
								if(q.query.at(0).find('!') != std::string::npos) {
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
								std::string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + gm[v].text; 
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

				string var_name = std::get<vector<pair<string,string>>>(gm[v].custom_props["Controls"]).at(0).first;
				string var_type = std::get<vector<pair<string,string>>>(gm[v].custom_props["Controls"]).at(0).second;

				valid_variables[var_name] = make_pair(q.query_var.second,aux);
				
				string gm_var_type = parse_gm_var_type(var_type);
				if(gm_var_type == "VALUE") {
					//We assume everything has a name attribute
					gm_var_map[var_name] = make_pair(aux.at(0).get<string>("name"),var_type); 
				} else if(gm_var_type == "SEQUENCE") {
					vector<string> var_value;
					for(pt::ptree t : aux) {
						var_value.push_back(t.get<string>("name"));
					}

					gm_var_map[var_name] = make_pair(var_value,var_type);
				}
			} else if(std::get<string>(gm[v].custom_props["GoalType"]) == "Achieve") {
				AchieveCondition a = std::get<AchieveCondition>(gm[v].custom_props["AchieveCondition"]);
				if(a.has_forAll_expr) {
					valid_forAll_conditions[depth] = a;
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
				Populate AT instances
				Steps:
				- Improve select statement parsing
				- For every AT, check if the parent monitored variable belongs to a forAll condition or not
					- This check will result in an if-else statement, depending on the result
				- We can just verify to which forAll condition the variable belongs to, because then each forAll condition in a higher level is
				  its parent
				- Think about how is the work of a forAll inside a forAll
					- If this gets too complex, deal with the case where a forAll can't be inside another forAll
					- In the future, a forAll can be inside a forAll and also a Loop type goal with its IterationRule
			
				- Not all abstract tasks will be under a forAll condition
				- Not all abstract tasks that are under a forAll condition happen in a condition defined by it. There can be tasks which happen in
				  static positions
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
				cout << "Could not find AT [" << at_def.second << "] in HDDL Domain definition." << endl;
				exit(1);
			}

			/*
				For now: Assume only one forAll condition is allowed for each AT
			*/
			if(!valid_forAll_conditions.empty()) {
				string location_var, forAll_iterated_var, forAll_iteration_var;
				map<int,AchieveCondition>::iterator a_it;

				for(a_it = valid_forAll_conditions.begin();a_it != valid_forAll_conditions.end();++a_it) {
					//Since we are assuming one valid forAll condition, this will work properly
					forAll_iterated_var = a_it->second.get_iterated_var();
					forAll_iteration_var = a_it->second.get_iteration_var();
				}

				if(gm[v].custom_props.find("Location") != gm[v].custom_props.end()) {
					location_var = get<string>(gm[v].custom_props["Location"]);
				} else {
					location_var = "";
				}

				if(location_var == forAll_iteration_var) {
					for(pt::ptree current_val : valid_variables[forAll_iterated_var].second) {
						AbstractTask at;
						
						if(at_ids.find(at_def.first) != at_ids.end()) {
							at_ids[at_def.first]++;
						} else {
							at_ids[at_def.first] = 1;
						}

						at.id = at_def.first + "_" + to_string(at_ids[at_def.first]);
						at.name = at_def.second;
						at.location = make_pair(current_val.get<string>("name"),location_var);
						at.at = at_hddl_def;
						at.fixed_robot_num = gm[v].fixed_robot_num;
						if(holds_alternative<int>(gm[v].robot_num)) {
							at.robot_num = get<int>(gm[v].robot_num); 
						} else {
							at.robot_num = get<pair<int,int>>(gm[v].robot_num);
						}
						
						for(VariableMapping var : var_mapping) {
							if(var.get_task_id() == at_def.first) {
								if(var.get_gm_var() == forAll_iteration_var) {
									at.variable_mapping.push_back(make_pair(current_val.get<string>("name"),var.get_hddl_var()));
								} else {
									if(valid_variables.find(var.get_gm_var()) != valid_variables.end()) {
										at.variable_mapping.push_back(make_pair(valid_variables[var.get_gm_var()].second.at(0).get<string>("name"),var.get_hddl_var()));
									} else { 
										cout << "Could not find variable mapping..." << endl;
										exit(1);
									}
								}
							}
						}

						at_instances[at.name].push_back(at);
					}
				} else {
					/*
						Here we are assuming that the location variable is not the iteration variable

						-> We create multiple instances with the same value
					*/
					for(unsigned int i = 0;i < valid_variables[forAll_iterated_var].second.size();i++) {
						AbstractTask at;

						if(at_ids.find(at_def.first) != at_ids.end()) {
							at_ids[at_def.first]++;
						} else {
							at_ids[at_def.first] = 1;
						}

						at.id = at_def.first + "_" + to_string(at_ids[at_def.first]);
						at.name = at_def.second;
						at.location = make_pair(valid_variables[location_var].second.at(0).get<string>("name"), location_var);
						at.at = at_hddl_def;
						at.fixed_robot_num = gm[v].fixed_robot_num;
						if(holds_alternative<int>(gm[v].robot_num)) {
							at.robot_num = get<int>(gm[v].robot_num); 
						} else {
							at.robot_num = get<pair<int,int>>(gm[v].robot_num);
						}

						for(VariableMapping var : var_mapping) {
							if(var.get_task_id() == at_def.first) {
								if(var.get_gm_var() == forAll_iteration_var) {
									at.variable_mapping.push_back(make_pair(valid_variables[forAll_iteration_var].second.at(0).get<string>("name"),var.get_hddl_var()));
								} else {
									if(valid_variables.find(var.get_gm_var()) != valid_variables.end()) {
										at.variable_mapping.push_back(make_pair(valid_variables[var.get_gm_var()].second.at(0).get<string>("name"),var.get_hddl_var()));
									} else { 
										cout << "Could not find variable mapping..." << endl;
										exit(1);
									}
								}
							}
						}

						at_instances[at.name].push_back(at);
					}
				}
			} else {
				/*
					Here is the case where we don't have a forAll statement
				*/
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
				at.location = make_pair(valid_variables[location_var].second.at(0).get<string>("name"),location_var);
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
							at.variable_mapping.push_back(make_pair(valid_variables[var.get_gm_var()].second.at(0).get<string>("name"),var.get_hddl_var()));
						} else {
							cout << "Could not find variable mapping..." << endl;
							exit(1);
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
				cout << var_map.second << ": " << var_map.first << endl;
			}
			cout << "Triggering Events:" << endl;
			for(string event : inst.triggering_events) {
				cout << event << ", ";
			}
			cout << endl;
		}
	}
}

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