#include "annotmanagerutils.hpp"

#include <iostream>
#include <regex>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "../rannot/rannot.hpp"
#include "math_utils.hpp"

using namespace std;

int parse_string(const char* in);

map<string,general_annot*> goals_and_rannots;

/*
    Function: retrieve_runtime_annot
    Objective: Retrieve the runtime annotation of some given node. We need to have goals_and_rannots initialized

    @ Input: The text of the desired node in the goal model
    @ Output: The runtime annotation of the given node
*/ 
general_annot* retrieve_runtime_annot(string id) {
    parse_string(id.c_str());

    string node_name = get_node_name(id);

    return goals_and_rannots[node_name];
}

/*
    Function: recursive_fill_up_runtime_annot
    Objective: Based on a current node, fill up execution constraints related attributes of runtime annotation nodes

    @ Input 1: The runtime annotation to be filled
    @ Input 2: The vertex of the Goal Model
    @ Output: None. The runtime annotation nodes are filled
*/
void recursive_fill_up_runtime_annot(general_annot* rannot, VertexData gm_node) {
    if(!gm_node.group || (gm_node.group && !gm_node.divisible)) {
        rannot->non_coop = true;
    } else {
        rannot->non_coop = false;
    }

    if(!gm_node.group) {
        rannot->group = false;
    }
    if(!gm_node.divisible) {
        rannot->divisible = false;
    }

    rannot->related_goal = get_node_name(gm_node.text);

    for(general_annot* child : rannot->children) {
        if(child->type == OPERATOR) {
            recursive_fill_up_runtime_annot(child, gm_node);
        }
    }
}

/*
    Function: recursive_child_replacement
    Objective: Replacing the children nodes of a given annotation, since we are dealing with references.
    This is needed in order to deal with forAll statements runtime annotations

    @ Input 1: A reference to the copy runtime annotation to be created
    @ Input 2: A reference to the original runtime annotation
    @ Output: Void. The copy runtime annotation is initialized with the values of the original one
*/ 
void recursive_child_replacement(general_annot* copy, general_annot* original) {
    if(original->children.size() > 0) {
        for(general_annot* original_child : original->children) {
            general_annot* child_copy = new general_annot();

            child_copy->content = original_child->content;
            child_copy->type = original_child->type;
            child_copy->related_goal = original_child->related_goal;
            child_copy->non_coop = original_child->non_coop;
            child_copy->group = original_child->group;
            child_copy->divisible = original_child->divisible;
            recursive_child_replacement(child_copy, original_child);

            child_copy->parent = copy;
            copy->children.push_back(child_copy);
        }
    }
}

/*
    Function: rename_at_instances_in_runtime_annot
    Objective: Rename AT instances in goal model runtime annotation

    @ Input 1: The goal model runtime annotation
    @ Input 2: The at instances map
    @ Input 3: The goal model as a GMGraph object
    @ Output: Void. The goal model runtime annotation has the AT's instances renamed
*/ 
void rename_at_instances_in_runtime_annot(general_annot* gmannot, map<string,vector<AbstractTask>> at_instances, GMGraph gm) {
    map<string,int> at_instances_counter;

    map<string,vector<AbstractTask>>::iterator at_inst_it;
    for(at_inst_it = at_instances.begin();at_inst_it != at_instances.end();at_inst_it++) {
        string task_id;
        if(at_inst_it->second.at(0).id.find("_") != string::npos) {
            task_id = at_inst_it->second.at(0).id.substr(0,at_inst_it->second.at(0).id.find("_"));
        } else {
            task_id = at_inst_it->second.at(0).id;
        }

        at_instances_counter[task_id] = 0;
    }

    if(gmannot->content == parallel_op && gmannot->related_goal == "") {
        //Dealing with a forAll in the root
        for(general_annot* child : gmannot->children) {
            recursive_at_instances_renaming(child, at_instances_counter, true, at_instances, gm);
        }
    } else {
        for(general_annot* child : gmannot->children) {
            recursive_at_instances_renaming(child, at_instances_counter, false, at_instances, gm);
        }
    }
}

/*
    Function: recursive_at_instances_renaming
    Objective: Rename AT instances in the runtime annotation object and recursive call this renaming step

    @ Input 1: The runtime annotation being considered
    @ Input 2: The counter in order to know which instance we left
    @ Input 3: Boolean flag to know if we have a forAll generated node in the root
    @ Input 4: The at instances map
    @ Input 5: The goal model as a GMGraph object
    @ Output: Void. The runtime goal model annotation is renamed
*/ 
void recursive_at_instances_renaming(general_annot* rannot, map<string,int>& at_instances_counter, bool in_forAll, map<string,vector<AbstractTask>> at_instances, GMGraph gm) {
    set<string> operators {sequential_op,parallel_op,fallback_op,"OPT","|"};

    set<string>::iterator op_it;

    op_it = operators.find(rannot->content);

    if(op_it == operators.end()) { //If we have a task
        if(rannot->type == TASK) {
            int gm_id = find_gm_node_by_id(rannot->content.substr(0,rannot->content.find("_")), gm);
            pair<string,string> at_id_name = parse_at_text(gm[gm_id].text);

            rannot->content = at_instances[at_id_name.second].at(at_instances_counter[at_id_name.first]).id;

            at_instances_counter[at_id_name.first]++;
        } else {
            if(rannot->type == MEANSEND) {
                general_annot* child = rannot->children.at(0);
                recursive_at_instances_renaming(child, at_instances_counter, in_forAll, at_instances, gm);
            }
        }
    } else {
        if(rannot->content == parallel_op && rannot->related_goal == "") {
            for(general_annot* child : rannot->children) {
                recursive_at_instances_renaming(child, at_instances_counter, true, at_instances, gm);
            }
        } else {
            for(general_annot* child : rannot->children) {
                recursive_at_instances_renaming(child, at_instances_counter, in_forAll, at_instances, gm);
            }
        }
    }
}   

/*
    Function: print_runtime_annot_from_general_annot
    Objective: Print the given runtime annotation in the terminal

    @ Input: The runtime annotation to be printed 
    @ Output: Void. The runtime annotation is printed
*/ 
void print_runtime_annot_from_general_annot(general_annot* rt) {
    string rt_annot = "";

    rt_annot = recursive_rt_annot_build(rt);

    cout << rt_annot << endl;
}

/*
    Function: recursive_rt_annot_build
    Objective: Build the runtime annotation string recursively

    @ Input: The current runtime annotation being considered
    @ Output: The current runtime annotation string
*/ 
string recursive_rt_annot_build(general_annot* rt) {
    set<string> operators {sequential_op,parallel_op,fallback_op};

    set<string>::iterator op_it;

    op_it = operators.find(rt->content);

    string annot = "";
    if(rt->non_coop) {
        annot += "NC(";
    }
    if(op_it != operators.end()) {
        if(rt->content == fallback_op) {
            vector<string> children;
            for(general_annot* child : rt->children) {
                children.push_back(recursive_rt_annot_build(child));
            }

            annot += "FALLBACK(";
            int cnt = 0;
            for(string c : children) {
                annot += c;
                cnt++;
                if(cnt == 1) {
                    annot += ",";
                } else {
                    annot += ")";
                }
            }
        } else {
            vector<string> children;
            for(general_annot* child : rt->children) {
                children.push_back(recursive_rt_annot_build(child));
            }

            if(!rt->non_coop) {
                annot += "(";
            }
            unsigned int cnt = 0;
            for(string c : children) {
                annot += c;
                if(cnt < rt->children.size()-1) {
                    annot += rt->content;
                } else {
                    annot += ")";
                }
                cnt++;
            }
        }
    } else {
        if(rt->type == MEANSEND) {
            annot += recursive_rt_annot_build(rt->children.at(0));
        } else {
            annot += rt->content;
        }
    }

    if(rt->non_coop) {
        annot += ")";
    }

    return annot;
}

/*
    Function: solve_query_statement
    Objective: Solve a query statement filling up the valid variables at the end

    @ Input 1: The ptree where the query is being performed
    @ Input 2: The QueriedProperty itself
    @ Input 3: The Goal Model as a GMGraph object
    @ Input 4: The node ID
    @ Input 5: The valid variables map
    @ Input 6: The knowledge base unique ID name
    @ Output: Void. The valid variables map is filled with new variables
*/
pair<vector<pt::ptree>,set<string>> solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>>& valid_variables, string unique_id) {
    if(holds_alternative<pair<Query*,Query*>>(q.query->query)) {
        pair<Query*,Query*> query_items = std::get<pair<Query*,Query*>>(q.query->query);

        QueriedProperty aux1;
        aux1.queried_var = q.queried_var;
        aux1.query_var = q.query_var;
        aux1.query = query_items.first;

        pair<vector<pt::ptree>,set<string>> valid_query1 = solve_query_statement(queried_tree, aux1, gm, node_id, valid_variables, unique_id);

        QueriedProperty aux2;
        aux2.queried_var = q.queried_var;
        aux2.query_var = q.query_var;
        aux2.query = query_items.second;

        pair<vector<pt::ptree>,set<string>> valid_query2 = solve_query_statement(queried_tree, aux2, gm, node_id, valid_variables, unique_id);

        pair<vector<pt::ptree>,set<string>> final_result = valid_query1;

        if(q.query->is_and) {
            set<string> aux;

            std::set_difference(valid_query1.second.begin(), valid_query1.second.end(), valid_query2.second.begin(), valid_query2.second.end(), std::inserter(aux, aux.end()));

            for(string elem : aux) {
                set<string>::iterator pos = final_result.second.find(elem);
                if(pos != final_result.second.end()) {
                    final_result.second.erase(pos);
                }
            }

            vector<pt::ptree>::iterator result_it;
            for(result_it = final_result.first.begin(); result_it != final_result.first.end(); ) {
                string result_val = result_it->get<string>(unique_id);

                if(final_result.second.find(result_val) == final_result.second.end()) {
                    final_result.first.erase(result_it);
                } else {
                    result_it++;
                }
            }
        } else {
            set<string> aux;

            std::set_difference(valid_query1.second.begin(), valid_query1.second.end(), valid_query2.second.begin(), valid_query2.second.end(), std::inserter(aux, aux.end()));

            for(string elem : aux) {
                set<string>::iterator pos = final_result.second.find(elem);
                if(pos == final_result.second.end()) {
                    final_result.second.insert(elem);
                }
            }

			for(pt::ptree res : valid_query2.first) {
				if(aux.find(res.get<string>(unique_id)) != aux.end()) {
					final_result.first.push_back(res);
				}
			}
        }

        return final_result;
    } else {
        vector<pt::ptree> aux;
        set<string> accepted_records;

        vector<string> query_item = std::get<vector<string>>(q.query->query);
                    
        if(!queried_tree.empty()) {
            BOOST_FOREACH(pt::ptree::value_type& child, queried_tree) {
                if(child.first == q.query_var.second) {
                    if(query_item.size() == 1) {
                        if(query_item.at(0) != "") {
                            string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

                            boost::optional prop_val_opt = child.second.get_optional<string>(prop);

                            if(prop_val_opt) {
                                bool prop_val;

                                istringstream(boost::to_lower_copy(prop_val_opt.get())) >> std::boolalpha >> prop_val;
                                if(query_item.at(0).find('!') != string::npos) {
                                    prop_val = !prop_val;
                                }
                                if(prop_val) {
                                    aux.push_back(child.second);
                                    accepted_records.insert(child.second.get<string>(unique_id));
                                }
                            }
                        } else {
                            aux.push_back(child.second);
                        }
                    } else {
                        if(query_item.at(1) == ocl_equal || query_item.at(1) == ocl_different) {
                            string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

                            boost::optional prop_val_opt = child.second.get_optional<string>(prop);

                            if(prop_val_opt) {
                                string prop_val = prop_val_opt.get();

                                bool result;
                                if(query_item.at(1) == ocl_equal) {
                                    result = (prop_val == query_item.at(2));
                                } else {
                                    result = (prop_val != query_item.at(2));
                                }

                                if(result) {
                                    aux.push_back(child.second);
                                    accepted_records.insert(child.second.get<string>(unique_id));
                                }
                            }
                        } else if(query_item.at(1) == ocl_in) {
                            string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

                            boost::optional prop_val_opt = child.second.get_optional<string>(prop);

                            if(prop_val_opt) {
                                string prop_val = prop_val_opt.get();
                                
                                string attr_to_search = query_item.at(2);
                                std::replace(attr_to_search.begin(), attr_to_search.end(), '.', ' ');
                                
                                vector<string> split_attr;
                                
                                stringstream ss(attr_to_search);
                                string tmp;
                                while(ss >> tmp) {
                                    split_attr.push_back(tmp);
                                }

                                /*
                                    If we have [VAR].[ATTR] in [VAR].[ÁTTR] we search in the ptree

                                    If we have [VAR].[ATTR] in [VAR], where VAR is a collection variable, we search in the variable value
                                */
                                if(split_attr.size() == 1) {
                                    vector<pt::ptree> var_value = valid_variables[split_attr.at(0)].second;
                                    
                                    bool found_attr = false;
                                    for(pt::ptree val : var_value) {
                                        if(val.get<string>(unique_id) == prop_val) {
                                            found_attr = true;
                                            
                                            break;
                                        }
                                    }

                                    if(found_attr) {
                                        aux.push_back(child.second);
                                        accepted_records.insert(child.second.get<string>(unique_id));
                                    }
                                } else if(split_attr.size() == 2) {
                                    // Here we need to get the query ptree for the second attribute
                                    pt::ptree attr_tree = valid_variables[split_attr.at(0)].second.at(0).get_child(split_attr.at(1));

                                    string attr_data = attr_tree.data();
                                    boost::trim(attr_data);
                                    if(attr_tree.empty() && attr_data != "") {
                                        vector<string> attr_values;

                                        stringstream ss(attr_data);
                                        string tmp;
                                        while(ss >> tmp) {
                                            attr_values.push_back(tmp);
                                        }

                                        if(std::find(attr_values.begin(), attr_values.end(), prop_val) != attr_values.end()) {
                                            aux.push_back(child.second);
                                            accepted_records.insert(child.second.get<string>(unique_id));
                                        }
                                    } else if(!attr_tree.empty() && attr_data == "") {
                                        BOOST_FOREACH(pt::ptree::value_type val, attr_tree) {
                                            if(prop_val == val.second.data()) {
                                                aux.push_back(child.second);

                                                break;
                                            }
                                        }
                                    } else {
                                        string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

                                        throw std::runtime_error(bad_condition);
                                    }
                                } else {
                                    string bad_condition = "Cannot solve condition in QueriedProperty of Goal " + get_node_name(gm[node_id].text); 

                                    throw std::runtime_error(bad_condition);
                                }
                            }
                        } else if(query_item.at(1) == ocl_gt || query_item.at(1) == ocl_lt || query_item.at(1) == ocl_geq || query_item.at(1) == ocl_leq) {
                            string prop = query_item.at(0).substr(query_item.at(0).find('.')+1);

                            boost::optional prop_val_opt = child.second.get_optional<string>(prop);

                            if(prop_val_opt) {
                                string prop_val = prop_val_opt.get();

                                std::regex integer("[0-9]+");

                                bool result = false;
                                if(std::regex_match(query_item.at(2), integer)) {
                                    int q_val = stoi(query_item.at(2));

                                    if(prop_val.find(".") == string::npos) {
                                        int val = stoi(prop_val);

                                        if(query_item.at(1) == ocl_gt) {
                                            result = (val > q_val);
                                        } else if(query_item.at(1) == ocl_lt) {
                                            result = (val < q_val);
                                        } else if(query_item.at(1) == ocl_geq) {
                                            result = (val >= q_val);
                                        } else if(query_item.at(1) == ocl_leq) {
                                            result = (val <= q_val);
                                        }
                                    } else {
                                        float val = static_cast<float>(::atof(prop_val.c_str()));

                                        if(query_item.at(1) == ocl_gt) {
                                            result = greater_than_float_and_int(q_val, val);
                                        } else if(query_item.at(1) == ocl_lt) {
                                            result = greater_than_int_and_float(q_val, val);
                                        } else if(query_item.at(1) == ocl_geq) {
                                            result = !greater_than_int_and_float(q_val, val);
                                        } else if(query_item.at(1) == ocl_leq) {
                                            result = !greater_than_float_and_int(q_val, val);
                                        }
                                    }
                                } else {
                                    float q_val = static_cast<float>(::atof(query_item.at(2).c_str()));

                                    if(prop_val.find(".") == string::npos) {
                                        int val = stoi(prop_val);

                                        if(query_item.at(1) == ocl_gt) {
                                            result = greater_than_int_and_float(val, q_val);
                                        } else if(query_item.at(1) == ocl_lt) {
                                            result = greater_than_float_and_int(val, q_val);
                                        } else if(query_item.at(1) == ocl_geq) {
                                            result = !greater_than_float_and_int(val, q_val);
                                        } else if(query_item.at(1) == ocl_leq) {
                                            result = !greater_than_int_and_float(val, q_val);
                                        }
                                    } else {
                                        float val = static_cast<float>(::atof(prop_val.c_str()));

                                        if(query_item.at(1) == ocl_gt) {
                                            result = greater_than_floats(val, q_val);
                                        } else if(query_item.at(1) == ocl_lt) {
                                            result = greater_than_floats(q_val, val);
                                        } else if(query_item.at(1) == ocl_geq) {
                                            result = !greater_than_floats(q_val, val);
                                        } else if(query_item.at(1) == ocl_leq) {
                                            result = !greater_than_floats(val, q_val);
                                        }
                                    }
                                }

                                if(result) {
                                    aux.push_back(child.second);
                                    accepted_records.insert(child.second.get<string>(unique_id));
                                }
                            }
                        }
                    }
                }
            }

            /*string var_name = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).first;
            string var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).second;

            valid_variables[var_name] = make_pair(var_type,aux);*/
        }

        return make_pair(aux,accepted_records);
    }
}

/*
    Function: get_query_ptree
    Objective: Based on a queried property, find the ptree that corresponds to the object being queried

    @ Input 1: The Goal Model as a GMGraph object
    @ Input 2: The Query goal node ID
    @ Input 3: The valid variables map
    @ Input 4: The valid forAll conditions map
    @ Input 5: The world knowledge ptree
    @ Input 6: The knowledge unique ID name
    @ Output: The ptree corresponding to the queried object
*/
pt::ptree get_query_ptree(GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>> valid_variables, map<int,AchieveCondition> valid_forAll_conditions, pt::ptree world_tree, string unique_id) {
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
				valid_query = false;
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

			pt::ptree var_to_query;
			string var_type;

			bool found_var = false;
			int current_attr = 0;

			if(valid_variables.find(query_attrs.at(0)) != valid_variables.end()) {
				var_type = valid_variables[query_attrs.at(0)].first;
				found_var = true;
			}

			if(!found_var) {
				valid_query = false;
			}			
							
			if(valid_query) {
				BOOST_FOREACH(pt::ptree::value_type& child, world_tree) {
					if(child.first == var_type) {	
						if(child.second.get<string>(unique_id) == valid_variables[query_attrs.at(0)].second.at(0).get<string>(unique_id)) { //Doesn't work for collection variables
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
                        size_t begin = queried_var_type.find("(")+1;
                        size_t end = queried_var_type.find(")");
						queried_var_type = queried_var_type.substr(begin,end-begin);
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

	return queried_tree;
}