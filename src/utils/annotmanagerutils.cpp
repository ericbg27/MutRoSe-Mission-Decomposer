#include "annotmanagerutils.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "../rannot/rannot.hpp"

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

    if(gmannot->content == "#" && gmannot->related_goal == "") {
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
    set<string> operators {";","#","FALLBACK","OPT","|"};

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
        if(rannot->content == "#" && rannot->related_goal == "") {
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
    set<string> operators {";","#","FALLBACK","OPT","|"};

    set<string>::iterator op_it;

    op_it = operators.find(rt->content);

    string annot = "";
    if(rt->non_coop) {
        annot += "NC(";
    }
    if(op_it != operators.end()) {
        if(rt->content == "OPT") {
            string child = recursive_rt_annot_build(rt->children.at(0));

            annot += "OPT(" + child + ")";
        } else if(rt->content == "FALLBACK") {
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
    @ Output: Void. The valid variables map is filled with new variables
*/
void solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>>& valid_variables) {
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

		string var_name = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).first;
		string var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).second;

		valid_variables[var_name] = make_pair(var_type,aux);
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
    @ Output: The ptree corresponding to the queried object
*/
pt::ptree get_query_ptree(GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>> valid_variables, map<int,AchieveCondition> valid_forAll_conditions, pt::ptree world_tree) {
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