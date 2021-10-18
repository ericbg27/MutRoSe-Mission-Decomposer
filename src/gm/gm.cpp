#include "gm.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <iostream>
#include <regex>
#include <sstream>
#include <set>

typedef boost::graph_traits<GMGraph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<GMGraph>::edge_descriptor edge_descriptor;

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

bool exists_path(int source, int target, GMGraph gm) {
    int vertices_number = boost::num_vertices(gm);

    vector<bool> visited(vertices_number, false);
    visited[source] = true;
    
    std::stack<int> next;
    next.push(source);

    while(!next.empty()) {
        int current_vertex = next.top();
        next.pop();

        for(int next_vertex = 0; next_vertex < vertices_number; next_vertex++) {
            if(!visited[next_vertex] && std::find(gm[current_vertex].children.begin(), gm[current_vertex].children.end(), next_vertex) != gm[current_vertex].children.end()) {
                visited[next_vertex] = true;
                next.push(next_vertex);
            }
        }
    }

    return visited[target];
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
            
            if(ac.has_forAll_expr) {
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
            }
        } else if(goal_type == query_goal_type) {
            QueriedProperty qp = std::get<QueriedProperty>(gm[v].custom_props[queried_property_prop]);

            if(controlled_vars.size() == 0) {
                string no_controlled_variable_error = "No controlled variable was declared for Query goal [" + get_node_name(gm[v].text) + "]";

		        throw std::runtime_error(no_controlled_variable_error);
            }

            string first_controlled_var_type = controlled_vars.at(0).second;
            if(parse_gm_var_type(first_controlled_var_type) == "COLLECTION") {
                first_controlled_var_type = first_controlled_var_type.substr(first_controlled_var_type.find("(")+1,first_controlled_var_type.find(")")-first_controlled_var_type.find("(")-1);
            }

            if(qp.query_var.second != first_controlled_var_type) {
                string wrong_type_controlled_var = "Query variable [" + qp.query_var.first + "] type + [" + qp.query_var.second + "] is different than the base type of the first controlled variable [" + controlled_vars.at(0).first + "] ([" + first_controlled_var_type + "])";

                throw std::runtime_error(wrong_type_controlled_var);
            }
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
    v.group = true;
    v.divisible = true;

    string s = "";

    map<string,string>::iterator cp_it;
    for(cp_it = custom_props.begin();cp_it != custom_props.end();++cp_it) {
        if(cp_it->first == goal_type_prop) {
            v.custom_props[goal_type_prop] = cp_it->second;
        } else if(cp_it->first == group_prop) {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            if(aux == "true" || aux == "false") {
                istringstream(aux) >> std::boolalpha >> v.group;
            } else {
                v.group = true;
            }
        } else if(cp_it->first == divisible_prop) {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            if(aux == "true" || aux == "false") {
                istringstream(aux) >> std::boolalpha >> v.divisible;
            } else {
                v.divisible = true;
            }
        } else if(cp_it->first == controls_prop || cp_it->first == monitors_prop) {
            boost::trim(cp_it->second);
            if(cp_it->second != "") {
                v.custom_props[cp_it->first] = parse_vars(cp_it->second);
            }
        } else if(cp_it->first == context_prop) {
            v.custom_props[cp_it->first] = cp_it->second;
        } else if(cp_it->first == context_trigger_prop) {
            Context c = parse_context_condition(cp_it->second, context_trigger_prop);

            v.custom_props[cp_it->first] = c;
        } else if(cp_it->first == context_condition_prop) {
            Context c = parse_context_condition(cp_it->second, context_condition_prop);
            
            v.custom_props[cp_it->first] = c;
        } else if(cp_it->first == location_prop) {
            boost::trim(cp_it->second);
            if(cp_it->second != "") {
                v.custom_props[cp_it->first] = cp_it->second;
            }
        } else if(cp_it->first == robot_number_prop) {
            boost::trim(cp_it->second);
            if(cp_it->second != "") {
                v.robot_num = parse_robot_number(cp_it->second);

                if(holds_alternative<int>(v.robot_num)) {
                    v.fixed_robot_num = true;
                } else {
                    v.fixed_robot_num = false;
                }

                v.custom_props[robot_number_prop] = cp_it->second;
            }
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

    if(v.custom_props.find(goal_type_prop) != v.custom_props.end()) {
        if(std::get<string>(v.custom_props[goal_type_prop]) == achieve_goal_type) {
            AchieveCondition a;
            a = parse_achieve_condition(custom_props[achieve_condition_prop]);
            v.custom_props[achieve_condition_prop] = a;
            /*if(custom_props.find(failure_condition_prop) != custom_props.end()) {
                FailureCondition f;
                f.set_condition(custom_props[failure_condition_prop]);
                v.custom_props[failure_condition_prop] = f;
            }*/
        } else if(std::get<string>(v.custom_props[goal_type_prop]) == query_goal_type) {
            string aux = custom_props[queried_property_prop];
            std::transform(aux.begin(),aux.end(),aux.begin(),::tolower);
            if(aux.find("select")) {
                v.custom_props[queried_property_prop] = parse_select_expr(custom_props[queried_property_prop]);
            } else {
                string missing_select_statement_error = "Missing select statement in Query Goal [" + parse_goal_text(v.text).first + "]";

                throw std::runtime_error(missing_select_statement_error);
            }
        } else if(std::get<string>(v.custom_props[goal_type_prop]) == perform_goal_type) {
            /*if(custom_props.find(failure_condition_prop) != custom_props.end()) {
                FailureCondition f;
                f.set_condition(custom_props[failure_condition_prop]);
                v.custom_props[failure_condition_prop] = f;
            }*/
        } else {
            string goal_type_warning = "Invalid goal type on node [" + get_node_name(v.text) + "]: [" + std::get<string>(v.custom_props[goal_type_prop]) + "]. Defaulting to Perform type.";
            
            std::cout << std::endl;
            std::cout << "------------------------ WARNING ------------------------" << std::endl;
            std::cout << goal_type_warning << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl << std::endl;
        }
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
        v.x = node.second.get<float>("x");
        v.y = node.second.get<float>("y");

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

        boost::add_edge(boost::vertex(t,gm), boost::vertex(s,gm), e, gm);

        edges.push_back(make_pair(make_pair(s,t),e));
    }

    std::sort(edges.begin(), edges.end(), sort_edges(gm));

    return edges;
}

/*
    Function: graph_from_property_tree
    Objective: Here we generate the GMGraph from the property tree representing the Goal Model.

    @ Input: A ptree representing the Goal Model 
    @ Output: The GMGraph representing the Goal Model
*/ 
GMGraph graph_from_property_tree(pt::ptree root) {
    GMGraph gm, aux;

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

    vector<pair<int,VertexData>> gm_vertices = parse_gm_nodes(nodes);

    //Retrieve edges from Goal Model
    links = root.get_child("links");

    vector<pair<pair<int,int>, EdgeData>> edges;
    edges = parse_gm_edges(links, aux, gm_vertices);

    map<int,int> node_id_map;
    
    vector<pair<pair<int,int>, EdgeData>>::iterator edges_it;
    for(edges_it = edges.begin();edges_it != edges.end();++edges_it) {
        int source_id;
        if(node_id_map.find(edges_it->first.second) == node_id_map.end()) {
            source_id = boost::add_vertex(aux[edges_it->first.second], gm);

            node_id_map[edges_it->first.second] = source_id;
        } else {
            source_id = node_id_map[edges_it->first.second];
        }

        int target_id;
        if(node_id_map.find(edges_it->first.first) == node_id_map.end()) {
            target_id = boost::add_vertex(aux[edges_it->first.first], gm);

            node_id_map[edges_it->first.first] = target_id;
        } else {
            target_id = node_id_map[edges_it->first.first];
        }

        EdgeData gm_edge;
        gm_edge.id = edges_it->second.id;
        gm_edge.type = edges_it->second.type;
        gm_edge.source = edges_it->second.source;
        gm_edge.target = edges_it->second.target;

        boost::add_edge(boost::vertex(source_id, gm), boost::vertex(target_id, gm), gm_edge, gm);

        gm[source_id].children.push_back(target_id);
        gm[target_id].parent = source_id;
    }

    GMGraph::vertex_iterator i, e;
    for(boost::tie(i,e) = vertices(gm); i != e; ++i) {
        std::sort(gm[*i].children.begin(),gm[*i].children.end());
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
        if(vertex.type == istar_task && vertex.custom_props.find(robot_number_prop) == vertex.custom_props.end()) {
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

/*
    Function: print_gm
    Objective: Print GMGraph to cout

    @ Input: The GMGraph representing a Goal Model
    @ Output: void. There is only printing to a terminal
*/ 
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