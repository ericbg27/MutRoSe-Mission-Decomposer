#include "querysolver.hpp"

#include <regex>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "../utils/math_utils.hpp"

using namespace std;

QuerySolver::QuerySolver(GMGraph gm, QueriedProperty q, pt::ptree queried_tree, int node_id) {
    this->gm = gm;
    this->q = q;
    this->queried_tree = queried_tree;
    this->node_id = node_id;
}

void QuerySolver::solve_query_statement(map<string,pair<string,vector<pt::ptree>>>& valid_variables, map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map) {
	if(holds_alternative<pair<Query*,Query*>>(q.query->query)) {
        solve_query_statement(valid_variables, gm_var_map);
    } else {
        vector<string> query_item = std::get<vector<string>>(q.query->query);

        vector<pt::ptree> aux;
                    
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
                                if(prop_val) aux.push_back(child.second);
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
                                if(result) aux.push_back(child.second);
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
                                    if(holds_alternative<pair<vector<string>,string>>(gm_var_map[split_attr.at(0)])) {
                                        pair<vector<string>,string> var_value_and_type = std::get<pair<vector<string>,string>>(gm_var_map[split_attr.at(0)]);

                                        if(std::find(var_value_and_type.first.begin(), var_value_and_type.first.end(), prop_val) != var_value_and_type.first.end()) {
                                            aux.push_back(child.second);
                                        }
                                    } else {
                                        string query_statement_non_collection_var_error = "Wrong query statement in Goal " + get_node_name(gm[node_id].text) + ". Usage of in statement in non-collection variable.";

                                        throw std::runtime_error(query_statement_non_collection_var_error);
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

                                if(result) aux.push_back(child.second);
                            }
                        }
                    }
                }
            }

            string var_name = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).first;
            string var_type = std::get<vector<pair<string,string>>>(gm[node_id].custom_props[controls_prop]).at(0).second;

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
}