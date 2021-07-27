#include "gm.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <regex>
#include <sstream>
#include <set>

using namespace std;

/*
    Function: get_dfs_gm_nodes
    Objective: Go through the GM using DFS and return nodes in the order they are visited

    @ Input 1: The GMGraph representing the GM
    @ Output: The vertices indexes based on DFS visit
*/  
vector<int> get_dfs_gm_nodes(GMGraph gm) {
    auto indexmap = boost::get(boost::vertex_index, gm);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    DFSVisitor vis;
    boost::depth_first_search(gm, vis, colormap, 0);

    return vis.GetVector();
}

/*
    Function: check_gm_validity
    Objective: Go through the GMGraph representing the goal model and verify if it is valid. Here we mainly check
    if AchieveConditions are correctly constructed (if variables are declared). If invalid
    we throw an error at the first invalid construct we find.

    @ Input 1: The GMGraph representing the GM
    @ Output: Void. We only throw errors
*/ 
void check_gm_validity(GMGraph gm) {
    vector<int> vctr = get_dfs_gm_nodes(gm);

    map<string,string> declared_vars;

    for(int v : vctr) {
        string goal_type = get<string>(gm[v].custom_props[goal_type_prop]);

        vector<pair<string,string>> monitored_vars;
        if(holds_alternative<vector<pair<string,string>>>(gm[v].custom_props[monitors_prop])) {
            monitored_vars = std::get<vector<pair<string,string>>>(gm[v].custom_props[monitors_prop]);
        }

        pair<bool,pair<string,string>> undeclared_var;
        undeclared_var.first = false;
        
        for(pair<string,string> var : monitored_vars) {
            if(declared_vars.find(var.first) == declared_vars.end()) {
                undeclared_var.first = true;
                undeclared_var.second = var;
            }
        }

        if(undeclared_var.first) {
            string undeclared_variable_error = "Undeclared variable [" + undeclared_var.second.first + "] of type [" + undeclared_var.second.second + "] in goal " + get_node_name(gm[v].text);

            throw std::runtime_error(undeclared_variable_error);
        }

        vector<pair<string,string>> controlled_vars;
        if(holds_alternative<vector<pair<string,string>>>(gm[v].custom_props[controls_prop])) {
            controlled_vars = std::get<vector<pair<string,string>>>(gm[v].custom_props[controls_prop]);
        }

        for(pair<string,string> var : controlled_vars) {
            if(declared_vars.find(var.first) == declared_vars.end()) {
                declared_vars[var.first] = var.second;
            } else {
                string var_redeclaration_error = "Redeclaration of variable [" + var.first + "] in goal " + get_node_name(gm[v].text);

                throw std::runtime_error(var_redeclaration_error); 
            }
        }

        if(goal_type == achieve_goal_type) {
            AchieveCondition ac = std::get<AchieveCondition>(gm[v].custom_props[achieve_condition_prop]);
            
            bool found_iterated_var = false;
            for(auto monitored : monitored_vars) {
                if(monitored.first == ac.get_iterated_var()) {
                    found_iterated_var = true;
                    break;
                }
            }
            if(!found_iterated_var) {
                string iterated_var_err = "Did not find iterated variable " + ac.get_iterated_var() + " in " + get_node_name(gm[v].text) + "'s controlled variables list";
                throw std::runtime_error(iterated_var_err);
            }

            bool found_iteration_var = false;
            for(auto controlled : controlled_vars) {
                if(controlled.first == ac.get_iteration_var()) {
                    found_iteration_var = true;
                    break;
                }
            }
            if(!found_iteration_var) {
                string iteration_var_err = "Did not find iteration variable " + ac.get_iteration_var() + " in " + get_node_name(gm[v].text) + "'s monitored variables list";
                throw std::runtime_error(iteration_var_err);
            }
        } else if(goal_type == query_goal_type) {
            // TODO: verify correctness of queried property
        }

        if(goal_type != achieve_goal_type) {
            if(gm[v].custom_props.find(achieve_condition_prop) != gm[v].custom_props.end()) {
                string achieve_condition_error = "Goal of type " + goal_type + " cannot contain an Achieve Condition";

                throw std::runtime_error(achieve_condition_error);
            }
        }

        if(goal_type != query_goal_type) {
            if(gm[v].custom_props.find(queried_property_prop) != gm[v].custom_props.end()) {
                string queried_property_error = "Goal of type " + goal_type + " cannot contain a Queried Property";

                throw std::runtime_error(queried_property_error);
            }
        }
    }
}

/*
    Function: find_gm_node_by_id
    Objective: Find a vertex in the GMGraph given its user-defined ID

    @ Input 1: The user-defined ID in a string format
    @ Input 2: The GMGraph
    @ Output: The ID of the vertex in the GMGraph
*/ 
int find_gm_node_by_id(string id, GMGraph gm) {
    GMGraph::vertex_iterator v, vend;

    for(boost::tie(v,vend) = vertices(gm);v != vend;++v) {
        string node_id;
        if(gm[*v].type == istar_goal) {
            node_id = parse_goal_text(gm[*v].text).first;
        } else {
            node_id = parse_at_text(gm[*v].text).first;
        }

        if(id == node_id) {
            int n_id = *v;
            return n_id;
        }
    }

    return -1;
}


/*
    Function: analyze_custom_props
    Objective: Here we analyze all kinds of custom properties on a specific Goal Model vertex and
    attribute them to the custom properties attribute of this vertex

    @ Input 1: A map representing the custom properties
    @ Input 2: The reference to the Goal Model vertex
    @ Output: Void
*/ 
void analyze_custom_props(map<string,string> custom_props, VertexData& v) {
    v.periodic = false;
    v.group = true;
    v.divisible = true;

    string s = "";

    map<string,string>::iterator cp_it;
    for(cp_it = custom_props.begin();cp_it != custom_props.end();++cp_it) {
        if(cp_it->first == "Periodic") {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            //istringstream(boost::to_lower_copy(cp_it->second)) >> std::boolalpha >> v.periodic;
            istringstream(aux) >> std::boolalpha >> v.periodic;
        } else if(cp_it->first == "Deadline") { //Problem if deadline is symbolic (solve this later!)
            stringstream ss(cp_it->second);
            ss >> v.deadline;
        } else if(cp_it->first == goal_type_prop) {
            v.custom_props[goal_type_prop] = cp_it->second;
        } else if(cp_it->first == "Period") {
            stringstream ss(cp_it->second);
            ss >> v.period;
        } else if(cp_it->first == "Group") {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            istringstream(aux) >> std::boolalpha >> v.group;
        } else if(cp_it->first == "Divisible") {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            istringstream(aux) >> std::boolalpha >> v.divisible;
        } else if(cp_it->first == controls_prop || cp_it->first == monitors_prop) {
            v.custom_props[cp_it->first] = parse_vars(cp_it->second);
        } else if(cp_it->first == context_prop) {
            Context c;
            size_t pos1 = cp_it->second.find('\"');
            size_t pos2 = cp_it->second.find('\"',pos1+1);
            string cond = cp_it->second.substr(pos1+1,pos2);
            c.set_condition(cond.substr(0,cond.size()-1)); 

            string aux = cp_it->second;
            std::transform(aux.begin(),aux.end(),aux.begin(),::tolower);  
            if(aux.find(trigger_context_type) != string::npos) {
                c.set_context_type(trigger_context_type);
            } else if(aux.find(condition_context_type) != string::npos) {
                c.set_context_type(condition_context_type);
            }
            v.custom_props[cp_it->first] = c;
        } else if(cp_it->first == location_prop) {
            v.custom_props[cp_it->first] = cp_it->second;
        } else if(cp_it->first == robot_number_prop) {
            v.fixed_robot_num = false;
            v.robot_num = parse_robot_number(cp_it->second);
        } else if(cp_it->first == params_prop) {
            vector<string> params;

            string params_str = cp_it->second;

            stringstream ss(params_str);
            string param;
            while(std::getline(ss,param,',')) {
                boost::trim(param);
                params.push_back(param);
            }

            v.custom_props[cp_it->first] = params;
        } else {
            if(default_props.find(cp_it->first) == default_props.end()) {
                string error_str = "Invalid property " + cp_it->first + " in vertex " + v.text;

                throw std::runtime_error(error_str);
            }
        }
    }

    if(v.custom_props.find(goal_type_prop) == v.custom_props.end()) {
        v.custom_props[goal_type_prop] = perform_goal_type;
    }

    if(std::get<string>(v.custom_props[goal_type_prop]) == achieve_goal_type) {
        AchieveCondition a;
        a = parse_achieve_condition(custom_props[achieve_condition_prop]);
        v.custom_props[achieve_condition_prop] = a;
        if(custom_props.find(failure_condition_prop) != custom_props.end()) {
            FailureCondition f;
            f.set_condition(custom_props[failure_condition_prop]);
            v.custom_props[failure_condition_prop] = f;
        }
    } else if(std::get<string>(v.custom_props[goal_type_prop]) == query_goal_type) {
        string aux = custom_props[queried_property_prop];
        std::transform(aux.begin(),aux.end(),aux.begin(),::tolower);
        if(aux.find("select")) {
            v.custom_props[queried_property_prop] = parse_select_expr(custom_props[queried_property_prop]);
        } else {
            //v.custom_props[queried_property_prop] = custom_props[queried_property_prop];
            string missing_select_statement_error = "Missing select statement in Query Goal [" + parse_goal_text(v.text).first + "]";

            throw std::runtime_error(missing_select_statement_error);
        }
    } else if(std::get<string>(v.custom_props[goal_type_prop]) == perform_goal_type) {
        if(custom_props.find(failure_condition_prop) != custom_props.end()) {
            FailureCondition f;
            f.set_condition(custom_props[failure_condition_prop]);
            v.custom_props[failure_condition_prop] = f;
        }
    } else if(std::get<string>(v.custom_props[goal_type_prop]) == loop_goal_type) {
        string not_implemented_loop_goal_error = "Current version of the tool does not support Loop goals yet.";

        throw std::runtime_error(not_implemented_loop_goal_error);
        //v.custom_props["IterationRule"] = parse_iterate_expr(custom_props["IterationRule"]);
    }
}

/*
    Function: parse_gm_nodes
    Objective: Parse GM nodes and attribute values to vertices in the GMGraph. We do not add them in
    the GMGraph structure, only return the vertices and their user-defined ID's.

    @ Input: A ptree representing the nodes
    @ Output: A vector of vertices and their ids
*/ 
vector<pair<int,VertexData>> parse_gm_nodes(pt::ptree nodes) {
    vector<pair<int,VertexData>> vertex;

    int cnt = 0;
    BOOST_FOREACH(pt::ptree::value_type& node, nodes) {
        VertexData v;
        v.id = node.second.get<string>("id");
        v.text = node.second.get<string>("text");
        v.type = node.second.get<string>("type");
        v.x = node.second.get<int>("x");
        v.y = node.second.get<int>("y");

        pt::ptree c_props = node.second.get_child("customProperties");
        map<string,string> custom_props;
        BOOST_FOREACH(pt::ptree::value_type& prop, c_props) {
            custom_props[prop.first] = prop.second.get_value<string>();
        }

        analyze_custom_props(custom_props, v);

        if(v.type == istar_goal || v.type == istar_task) {
            string name = v.text.substr(0,v.text.find(':'));
            smatch m;
            regex e("[0-9]+");

            regex_search(name,m,e);
            
            stringstream ss(m[0]);
            int id = 0;
            ss >> id;

            vertex.push_back(make_pair(id,v));
        } else {
            vertex.push_back(make_pair(cnt,v));
            cnt++;
        }
    }

    std::sort(vertex.begin(), vertex.end(), sort_by_id());

    return vertex;
}

/*
    Function: parse_gm_edges
    Objective: Parse GM edges and update vertices parent and children attributes based on them. Also, create
    a vector of the edges and return them alongside the vertices linked by them

    @ Input: A ptree representing the nodes
    @ Output: A vector of vertices and their ids
*/ 
vector<pair<pair<int,int>, EdgeData>> parse_gm_edges(pt::ptree links, GMGraph& gm, vector<pair<int,VertexData>> vertex) {
    vector<pair<int, VertexData>>::iterator vertex_it;
    for(vertex_it = vertex.begin();vertex_it != vertex.end();++vertex_it) {
        boost::add_vertex(vertex_it->second, gm);
    }

    auto vertex_idMap = get(boost::vertex_index, gm);

    vector<pair<pair<int,int>, EdgeData>> edges;

    BOOST_FOREACH(pt::ptree::value_type& link, links) {
        EdgeData e;
        e.id = link.second.get<string>("id");
        e.type = link.second.get<string>("type");
        e.source = link.second.get<string>("source");
        e.target = link.second.get<string>("target");

        boost::graph_traits <GMGraph>::vertex_iterator i, end;

        int s = 0, t = 0;

        for(boost::tie(i, end) = vertices(gm); i != end; ++i) {
            if(gm[*i].id == e.source) {
                s = vertex_idMap[*i];
            }
            if(gm[*i].id == e.target) {
                t = vertex_idMap[*i];
            }
        }

        gm[boost::vertex(s,gm)].parent = t;
        gm[boost::vertex(t,gm)].children.push_back(s);

        edges.push_back(make_pair(make_pair(s,t),e));
    }

    std::sort(edges.begin(), edges.end(), sort_edges());

    return edges;
}

/*
    Function: graph_from_property_tree
    Objective: Here we generate the GMGraph from the property tree representing the Goal Model.

    @ Input: A ptree representing the Goal Model 
    @ Output: The GMGraph representing the Goal Model
*/ 
GMGraph graph_from_property_tree(pt::ptree root) {
    GMGraph gm;

    pt::ptree nodes;
    pt::ptree links;
	
    /*
        Retrieve nodes from Goal Model

        -> Later work out how to deal with multiple actors (we want only one)
    */
	BOOST_FOREACH(pt::ptree::value_type& child, root.get_child("actors")) { 
		pt::ptree subtree = child.second;
		BOOST_FOREACH(pt::ptree::value_type& c, subtree) {
			if(c.first == "nodes") {
				nodes = c.second;
			}
		}
	}

    vector<pair<int,VertexData>> vertices;
    vertices = parse_gm_nodes(nodes);

    //Retrieve edges from Goal Model
    links = root.get_child("links");

    vector<pair<pair<int,int>, EdgeData>> edges;
    edges = parse_gm_edges(links, gm, vertices);

    vector<pair<pair<int,int>, EdgeData>>::iterator edges_it;
    for(edges_it = edges.begin();edges_it != edges.end();++edges_it) {
        int s = edges_it->first.first;
        int t = edges_it->first.second;
        EdgeData e = edges_it->second;

        boost::add_edge(boost::vertex(t, gm), boost::vertex(s, gm), e, gm);
    }

    return gm;
}

/*
    Function: check_undefined_number_of_robots
    Objective: Here we infer the number of robots in tasks that do not have the RobotNumber attribute.
    This inferring occurs based on the HDDL Domain definition, since we assume that we can count the
    number of robots in the HDDL definition for tasks that do not have the RobotNumber attribute.

    @ Input 1: The reference to the GMGraph representing the Goal Model
    @ Input 2: The vector of the abstract tasks
    @ Input 3: The sort definitions, in order to infer types that are derived from the robot type 
    @ Output: Void
*/ 
void check_undefined_number_of_robots(GMGraph& gm, vector<task> abstract_tasks, vector<sort_definition> sort_definitions) {
    auto nodes = vertices(gm);

    int graph_size = *nodes.second - *nodes.first;

    for(int gm_index = 0;gm_index < graph_size;gm_index++) {
        VertexData vertex = gm[gm_index];
        if(vertex.type == istar_task && vertex.fixed_robot_num) {
            pair<string,string> at_id_name = parse_at_text(vertex.text);

            int robot_number = 0;
            for(task at : abstract_tasks) {
                if(at.name == at_id_name.second) {
                    for(int var_index = 0;var_index < at.number_of_original_vars;var_index++) {
                        string var_type = at.vars.at(var_index).second;
                        if(var_type == hddl_robot_type) {
                            robot_number++;
                        } else {
                            if(var_type == hddl_robotteam_type) { //Maybe later change this to deal with subtypes of robotteam
                                string prohibited_robotteam_error = "Tasks without RobotNumber attribute cannot have robotteam variable!";

                                throw std::runtime_error(prohibited_robotteam_error);
                            }
                            bool is_robot_subtype = false;
                            for(sort_definition def : sort_definitions) {
                                if(std::find(def.declared_sorts.begin(),def.declared_sorts.end(),var_type) != def.declared_sorts.end()) {
                                    if(def.has_parent_sort) {
                                        if(def.parent_sort == hddl_robot_type) {
                                            is_robot_subtype = true;
                                            break;
                                        }
                                    }
                                }
                            }

                            if(is_robot_subtype) {
                                robot_number++;
                            }
                        }
                    }

                    break;
                }
            }

            gm[gm_index].robot_num = robot_number;
        }
    }
}

/*
    Function: print_gm_nodes_info
    Objective: Print information about the vertices of the Goal Model

    @ Input: The GMGraph representing the goal model
    @ Output: void. There is only printing to a terminal
*/ 
void print_gm_nodes_info(GMGraph gm) {
    GMGraph::vertex_iterator i1, end1;
	GMGraph::adjacency_iterator ai1, a_end1;

	for(boost::tie(i1,end1) = vertices(gm); i1 != end1; ++i1) {
		VertexData node = gm[*i1];

		std::cout << "Node: " << node.text << std::endl;
		std::cout << "Context: " << std::endl;

		Context c;
			
		if(node.custom_props.find(context_prop) != node.custom_props.end()) {
			c = get<Context>(node.custom_props[context_prop]);

			std::cout << "\tType: " << c.get_context_type() << std::endl;
			std::cout << "\tCondition: " << c.get_condition() << std::endl;
		} else {
			std::cout << "\tNo Context" << std::endl;
		}

        std::cout << "Parameters: " << std::endl;

        if(node.custom_props.find(params_prop) != node.custom_props.end()) {
            for(string param : get<vector<string>>(node.custom_props[params_prop])) {
                std::cout << "\tParam: " << param << std::endl;
            }
        }

        std::cout << "Group? " << node.group << std::endl;
        std::cout << "Divisible? " << node.divisible << std::endl;
			
		std::cout << std::endl;
	}
}

/*
    Function: print_gm_var_map_info
    Objective: Print information variable mappings

    @ Input: The existing variable mappings
    @ Output: void. There is only printing to a terminal
*/ 
void print_gm_var_map_info(map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map) {
    map<string, variant<pair<string,string>,pair<vector<string>,string>>>::iterator gm_var_it;
	for(gm_var_it = gm_var_map.begin();gm_var_it != gm_var_map.end();++gm_var_it) {
		cout << "Var name: " << gm_var_it->first << endl;
		if(holds_alternative<pair<vector<string>,string>>(gm_var_it->second)) {
            cout << "Var type: " << get<pair<vector<string>,string>>(gm_var_it->second).second << endl;
			cout << "Mapping: [";
			unsigned int cnt = 0;
			vector<string> val_vec = get<pair<vector<string>,string>>(gm_var_it->second).first;
			for(string val : val_vec) {
				if(cnt == val_vec.size()-1) {
					cout << val << "]" << endl << endl;
				} else {
					cout << val << ",";
				}
				cnt++;
			} 
		} else {
            cout << "Var type: " << get<pair<string,string>>(gm_var_it->second).second << endl;
			cout << "Mapping: " << get<pair<string,string>>(gm_var_it->second).first << endl << endl;
		}
	}
}

void print_gm(GMGraph gm) {
    GMGraph::vertex_iterator i, end;
	GMGraph::adjacency_iterator ai, a_end;

	for(boost::tie(i,end) = vertices(gm); i != end; ++i) {
		VertexData node = gm[*i];
		std::cout << get_node_name(node.text) << "(" << *i << ") " << " --> ";

		for(boost::tie(ai,a_end) = adjacent_vertices(*i,gm); ai != a_end;++ai) {
			VertexData a_node = gm[*ai];
			std::cout << get_node_name(a_node.text) << "(" << *ai << ")" << " ";
		}	
		std::cout << std::endl;
	}

	std::cout << std::endl;
}