#include "annotmanager.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <regex>
#include <sstream>
#include <set>

#include "rannot.hpp"

int parse_string(const char* in);

map<string,general_annot*> goals_and_rannots;

general_annot* retrieve_runtime_annot(string id) {
    parse_string(id.c_str());

    string node_name = get_node_name(id);

    return goals_and_rannots[node_name];
}

general_annot* retrieve_gm_annot(GMGraph gm, pt::ptree worlddb, string location_type, map<string,vector<AbstractTask>> at_instances) {
    auto indexmap = boost::get(boost::vertex_index, gm);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    DFSVisitor vis;
    boost::depth_first_search(gm, vis, colormap, 0);

    std::vector<int> vctr = vis.GetVector();
    
    VertexData root = gm[vctr.at(0)];

    general_annot* root_annot = retrieve_runtime_annot(root.text);

    if(!gm[vctr.at(0)].group || (gm[vctr.at(0)].group && !gm[vctr.at(0)].divisible)) {
        root_annot->non_coop = true;
    } else {
        root_annot->non_coop = false;
    }

    if(!gm[vctr.at(0)].group) {
        root_annot->group = false;
    }
    if(!gm[vctr.at(0)].divisible) {
        root_annot->divisible = false;
    }

    map<string,pair<string,vector<pt::ptree>>> valid_variables;

    map<int,int> node_depths;
    map<int,AchieveCondition> valid_forAll_conditions;

    node_depths[vctr.at(0)] = 0;

    int current_node = vctr.at(0);

    vctr.erase(vctr.begin());

    general_annot* empty_annot = new general_annot();
    empty_annot->content = "";
    empty_annot->related_goal = "";

    root_annot->parent = empty_annot;

    recursive_gm_annot_generation(root_annot, vctr, gm, worlddb, location_type, current_node, valid_variables, valid_forAll_conditions, node_depths);

    return root_annot;
}

/*
    -> We have to check for the forAll operators
        - Create the different instances of the goal that contains a forAll as an achieve condition (if it generates multiple instances)
        - Put the different instances as parallel annotations
*/
void recursive_gm_annot_generation(general_annot* node_annot, vector<int>& vctr, GMGraph gm, pt::ptree worlddb, string location_type, int current_node,
                                        map<string,pair<string,vector<pt::ptree>>>& valid_variables, map<int,AchieveCondition> valid_forAll_conditions, 
                                        map<int,int>& node_depths) {
    set<string> operators {";","#","FALLBACK","OPT","|"};

    set<string>::iterator op_it;

    op_it = operators.find(node_annot->content);

    int depth;
    if(gm[current_node].parent != -1) {
        depth = node_depths[gm[current_node].parent] + 1;
        node_depths[current_node] = depth;
    } else {
        depth = node_depths[current_node];
    }

    bool is_forAll_goal = false;
    if(gm[current_node].type == "istar.Goal") {
		if(std::get<string>(gm[current_node].custom_props["GoalType"]) == "Query") {
			vector<pt::ptree> aux;
			QueriedProperty q = std::get<QueriedProperty>(gm[current_node].custom_props["QueriedProperty"]);
			BOOST_FOREACH(pt::ptree::value_type& child, worlddb.get_child("world_db")) {
				if(child.first == q.query_var.second) { //If type of queried var equals type of the variable in the database, check condition (if any)
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
						string prop_val = child.second.get<string>(prop);
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
			string var_name = std::get<vector<pair<string,string>>>(gm[current_node].custom_props["Controls"]).at(0).first; //.second would be for type assertion
			valid_variables[var_name] = make_pair(q.query_var.second,aux);
		} else if(std::get<string>(gm[current_node].custom_props["GoalType"]) == "Achieve") {
            is_forAll_goal = true;
			AchieveCondition a = std::get<AchieveCondition>(gm[current_node].custom_props["AchieveCondition"]);
			if(a.has_forAll_expr) {
				valid_forAll_conditions[depth] = a;
			}
		}
    }

    /*
        -> If we have an operator, simply check its children.

        -> If we have a Goal/Task we have two cases:
            - We may be dealing with a leaf node, in which case we simply finish the execution or
            - We may be dealing with a non-leaf node, in which case we expand it and substitute it for its extension in the parent's children
    */
    if(op_it != operators.end()) { //GM root goal    
        for(general_annot* child : node_annot->children) {
            int c_node = vctr.at(0);
            child->parent = node_annot;
            recursive_gm_annot_generation(child, vctr, gm, worlddb, location_type, c_node, valid_variables, valid_forAll_conditions, node_depths);
        }

        if(is_forAll_goal) {
            string iterated_var = valid_forAll_conditions[depth].get_iterated_var();
            int generated_instances = valid_variables[iterated_var].second.size();

            /*
                Generation of multiple instances of the forAll goal and its children
            */
            if(generated_instances > 1) {
                general_annot* aux = new general_annot();

                aux->content = node_annot->content;
                aux->type = node_annot->type;
                aux->children = node_annot->children;
                aux->related_goal = node_annot->related_goal;
                
                node_annot->content = "#";
                node_annot->type = OPERATOR;
                node_annot->children.clear();
                node_annot->related_goal = "";
                for(int i = 0;i < generated_instances;i++) {
                    general_annot* child = new general_annot();
                    
                    child->content = aux->content;
                    child->type = aux->type;
                    child->children = aux->children;
                    child->related_goal = aux->related_goal;

                    node_annot->children.push_back(child);
                }
            }
        }
    } else {
        if((!gm[vctr.at(0)].group) || (gm[vctr.at(0)].group && !gm[vctr.at(0)].divisible)) {
            node_annot->non_coop = true;
        } else {
            node_annot->non_coop = false;
        }

        if(!gm[vctr.at(0)].group) {
            node_annot->group = false;
        }
        if(!gm[vctr.at(0)].divisible) {
            node_annot->divisible = false;
        }
        if(gm[vctr.at(0)].children.size() == 0) { //Leaf Node
            vctr.erase(vctr.begin());
            return;
        } else {
            general_annot* expanded_annot = retrieve_runtime_annot(gm[vctr.at(0)].text);

            if(gm[vctr.at(0)].children.size() > 1) {
                if(expanded_annot->content == "") {
                    expanded_annot->content = ";";
                    for(int child : gm[vctr.at(0)].children) {
                        general_annot* aux = new general_annot();

                        string node_name = get_node_name(gm[child].text);

                        aux->content = node_name;
                        if(node_name.front() == 'G') {
                            aux->type = GOAL;
                        } else {
                            aux->type = TASK;
                        }

                        expanded_annot->children.push_back(aux);
                    }
                }
            } else { //Means-end decomposition
                int only_child = gm[vctr.at(0)].children.at(0);
                expanded_annot->content = get_node_name(gm[vctr.at(0)].text);
                expanded_annot->type = MEANSEND;

                general_annot* aux = new general_annot();

                string node_name = get_node_name(gm[only_child].text);
                
                aux->content = node_name;
                if(node_name.front() == 'G') {
                    aux->type = GOAL;
                } else {
                    aux->type = TASK;
                }

                if(aux->type == TASK) {
                    pair<string,string> node_name_id = parse_at_text(gm[only_child].text);
                    aux->content = node_name_id.first;
                }

                expanded_annot->children.push_back(aux);
            }

            node_annot->content = expanded_annot->content;
            node_annot->type = expanded_annot->type;
            node_annot->children = expanded_annot->children;
            node_annot->related_goal = expanded_annot->related_goal;
            
            vctr.erase(vctr.begin());
            for(general_annot* child : node_annot->children) {
                int c_node = vctr.at(0);
                child->parent = node_annot;
                recursive_gm_annot_generation(child, vctr, gm, worlddb, location_type, c_node, valid_variables, valid_forAll_conditions, node_depths);
            }

            if(is_forAll_goal) {
                string iterated_var = valid_forAll_conditions[depth].get_iterated_var();
                int generated_instances = valid_variables[iterated_var].second.size();

                /*
                    Generation of multiple instances of the forAll goal and its children
                */
                if(generated_instances > 1) {
                    general_annot* aux = new general_annot();

                    aux->content = node_annot->content;
                    aux->type = node_annot->type;
                    aux->children = node_annot->children;
                    aux->related_goal = node_annot->related_goal;
                    aux->non_coop = node_annot->non_coop;
                    aux->group = node_annot->group;
                    aux->divisible = node_annot->divisible;

                    node_annot->content = "#"; //Do we need to define a custom operator for instances generated by a forAll expression?
                    node_annot->type = OPERATOR;
                    node_annot->children.clear();
                    node_annot->related_goal = "";
                    node_annot->group = true;
                    node_annot->divisible = true;
                    for(int i = 0;i < generated_instances;i++) {
                        general_annot* child = new general_annot();
                        
                        child->content = aux->content;
                        child->type = aux->type;
                        child->non_coop = aux->non_coop;
                        child->group = aux->group;
                        child->divisible = aux->divisible;
                        for(general_annot* ch : aux->children) {
                            general_annot* copy = new general_annot();

                            copy->content = ch->content;
                            copy->type = ch->type;
                            copy->related_goal = ch->related_goal;
                            copy->non_coop = ch->non_coop;
                            copy->group = ch->group;
                            copy->divisible = ch->divisible;
                            recursive_child_replacement(copy, ch);
                            
                            copy->parent = child;
                            child->children.push_back(copy);
                        }
                        child->related_goal = aux->related_goal;

                        child->parent = node_annot;
                        node_annot->children.push_back(child);
                    }
                }
            }
        }
    }
}

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

void rename_at_instances_in_runtime_annot(general_annot* gmannot, map<string,vector<AbstractTask>> at_instances) {
    map<string,int> at_instances_counter;

    map<string,vector<AbstractTask>>::iterator at_inst_it;
    for(at_inst_it = at_instances.begin();at_inst_it != at_instances.end();at_inst_it++) {
        string task_id;
        if(at_inst_it->second.at(0).id.find("_") != std::string::npos) {
            task_id = at_inst_it->second.at(0).id.substr(0,at_inst_it->second.at(0).id.find("_"));
        } else {
            task_id = at_inst_it->second.at(0).id;
        }

        at_instances_counter[task_id] = 1;
    }

    if(gmannot->content == "#" && gmannot->related_goal == "") {
        //Dealing with a forAll in the root
        for(general_annot* child : gmannot->children) {
            recursive_at_instances_renaming(child, at_instances_counter, true);
        }
    } else {
        for(general_annot* child : gmannot->children) {
            recursive_at_instances_renaming(child, at_instances_counter, false);
        }
    }
}

void recursive_at_instances_renaming(general_annot* rannot, map<string,int>& at_instances_counter, bool in_forAll) {
    set<string> operators {";","#","FALLBACK","OPT","|"};

    set<string>::iterator op_it;

    op_it = operators.find(rannot->content);

    if(op_it == operators.end()) { //If we have a task
        if(rannot->type == TASK) {
            if(in_forAll) {
                string aux = rannot->content;
                rannot->content = rannot->content + "_" + to_string(at_instances_counter[rannot->content]);

                at_instances_counter[aux]++;
            }
        } else {
            if(rannot->type == MEANSEND) {
                general_annot* child = rannot->children.at(0);
                recursive_at_instances_renaming(child, at_instances_counter, in_forAll);
            }
        }
    } else {
        if(rannot->content == "#" && rannot->related_goal == "") {
            for(general_annot* child : rannot->children) {
                recursive_at_instances_renaming(child, at_instances_counter, true);
            }
        } else {
            for(general_annot* child : rannot->children) {
                recursive_at_instances_renaming(child, at_instances_counter, in_forAll);
            }
        }
    }
}   

void print_runtime_annot_from_general_annot(general_annot* rt) {
    string rt_annot = "";

    rt_annot = recursive_rt_annot_build(rt);

    cout << rt_annot << endl;
}

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