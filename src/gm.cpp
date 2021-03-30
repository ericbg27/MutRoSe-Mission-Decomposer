#include "gm.hpp"

#include <boost/foreach.hpp>
#include <iostream>
#include <regex>
#include <sstream>
#include <set>

const std::set<std::string> default_props{"Description", "QueriedProperty", "FailureCondition", "AchieveCondition"};

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
    std::vector<int> vctr = get_dfs_gm_nodes(gm);

    for(int v : vctr) {
        std::string goal_type = get<std::string>(gm[v].custom_props["GoalType"]);
        std::vector<std::pair<std::string,std::string>> controlled_vars;
        if(holds_alternative<std::vector<std::pair<std::string,std::string>>>(gm[v].custom_props["Controls"])) {
            controlled_vars = std::get<std::vector<std::pair<std::string,std::string>>>(gm[v].custom_props["Controls"]);
        }
        if(goal_type == "Achieve") {
            std::vector<std::pair<std::string,std::string>> monitored_vars;
            if(holds_alternative<std::vector<std::pair<std::string,std::string>>>(gm[v].custom_props["Monitors"])) {
                monitored_vars = std::get<std::vector<std::pair<std::string,std::string>>>(gm[v].custom_props["Monitors"]);
            }

            AchieveCondition ac = std::get<AchieveCondition>(gm[v].custom_props["AchieveCondition"]);
            
            bool found_iterated_var = false;
            for(auto monitored : monitored_vars) {
                if(monitored.first == ac.get_iterated_var()) {
                    found_iterated_var = true;
                    break;
                }
            }
            if(!found_iterated_var) {
                std::string iterated_var_err = "Did not find iterated variable " + ac.get_iterated_var() + " in " + get_node_name(gm[v].text) + "'s controlled variables list";
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
                std::string iteration_var_err = "Did not find iteration variable " + ac.get_iteration_var() + " in " + get_node_name(gm[v].text) + "'s monitored variables list";
                throw std::runtime_error(iteration_var_err);
            }
        }
    }
}

/*
    Function: get_node_name
    Objective: Return the user-defined ID of some node given its text

    @ Input: The node text
    @ Output: The node name
*/ 
string get_node_name(string node_text) {
    size_t pos = node_text.find(":");
    string node_name;
    if(pos != std::string::npos) {
        node_name = node_text.substr(0,pos);
    }

    return node_name;
}

/*
    Function: parse_gm_var_type
    Objective: Verify the OCL type represented by the input string and return a standard string

    @ Input 1: The OCL type in string format
    @ Output: A standard string representing the given type
*/ 
string parse_gm_var_type(string var_type) {
    std::transform(var_type.begin(), var_type.end(), var_type.begin(), ::toupper);
    if(var_type.find("SEQUENCE") != std::string::npos) {
        return "COLLECTION";
    }

    return "VALUE";
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
        string node_id = parse_goal_text(gm[*v].text).first;

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
    //v.non_cooperative = false;
    std::string s = "";

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
        } else if(cp_it->first == "GoalType") {
            v.custom_props["GoalType"] = cp_it->second;
        } else if(cp_it->first == "Period") {
            stringstream ss(cp_it->second);
            ss >> v.period;
        } else if(cp_it->first == "Group") {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            //istringstream(boost::to_lower_copy(cp_it->second)) >> std::boolalpha >> v.non_cooperative;
            istringstream(aux) >> std::boolalpha >> v.group;
        } else if(cp_it->first == "Divisible") {
            string aux = cp_it->second;
            transform(aux.begin(), aux.end(), aux.begin(), ::tolower);
            //istringstream(boost::to_lower_copy(cp_it->second)) >> std::boolalpha >> v.non_cooperative;
            istringstream(aux) >> std::boolalpha >> v.divisible;
        } else if(cp_it->first == "Controls" || cp_it->first == "Monitors") {
            v.custom_props[cp_it->first] = parse_vars(cp_it->second);
        } else if(cp_it->first == "CreationCondition") {
            Context c;
            size_t pos1 = cp_it->second.find('\"');
            size_t pos2 = cp_it->second.find('\"',pos1+1);
            c.condition = cp_it->second.substr(pos1+1,pos2);
            c.condition = c.condition.substr(0,c.condition.size()-1);   
            if(cp_it->second.find("trigger") != std::string::npos) {
                c.type = "trigger";
            } else if(cp_it->second.find("condition") != std::string::npos) {
                c.type = "condition";
            }
            v.custom_props[cp_it->first] = c;
        } else if(cp_it->first == "Location") {
            v.custom_props[cp_it->first] = cp_it->second;
        } else if(cp_it->first == "RobotNumber") {
            v.fixed_robot_num = false;
            v.robot_num = parse_robot_number(cp_it->second);
        } else {
            if(default_props.find(cp_it->first) == default_props.end()) {
                std::string error_str = "Invalid property " + cp_it->first + " in vertex " + v.text;
                throw std::runtime_error(error_str);
            }
        }
    }

    if(v.custom_props.find("GoalType") == v.custom_props.end()) {
        v.custom_props["GoalType"] = "Perform";
    }

    if(std::get<string>(v.custom_props["GoalType"]) == "Achieve") {
        AchieveCondition a;
        a = parse_achieve_condition(custom_props["AchieveCondition"]);
        v.custom_props["AchieveCondition"] = a;
        if(custom_props.find("FailureCondition") != custom_props.end()) {
            FailureCondition f;
            f.condition = custom_props["FailureCondition"];
            v.custom_props["FailureCondition"] = f;
        }
    } else if(std::get<string>(v.custom_props["GoalType"]) == "Query") {
        if(custom_props["QueriedProperty"].find("select")) {
            v.custom_props["QueriedProperty"] = parse_select_expr(custom_props["QueriedProperty"]);
        } else {
            v.custom_props["QueriedProperty"] = custom_props["QueriedProperty"];
        }
    } else if(std::get<string>(v.custom_props["GoalType"]) == "Perform") {
        if(custom_props.find("FailureCondition") != custom_props.end()) {
            FailureCondition f;
            f.condition = custom_props["FailureCondition"];
            v.custom_props["FailureCondition"] = f;
        }
    } /*else if(std::get<string>(v.custom_props["GoalType"]) == "Loop") {
        v.custom_props["IterationRule"] = parse_iterate_expr(custom_props["IterationRule"]);
    } else if(std::get<string>(v.custom_props["GoalType"]) == "Trigger") {
        v.custom_props["TriggeredEvent"] = custom_props["TriggeredEvent"];
    } */
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

        if(v.type == "istar.Goal" || v.type == "istar.Task") {
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

        int s, t;

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

    vector<pair<int,VertexData>> vertex;

    vertex = parse_gm_nodes(nodes);

    //Retrieve edges from Goal Model
    links = root.get_child("links");

    vector<pair<pair<int,int>, EdgeData>> edges;

    edges = parse_gm_edges(links, gm, vertex);

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
        if(vertex.type == "istar.Task" && vertex.fixed_robot_num) {
            pair<string,string> at_id_name = parse_at_text(vertex.text);

            int robot_number = 0;
            for(task at : abstract_tasks) {
                if(at.name == at_id_name.second) {
                    for(int var_index = 0;var_index < at.number_of_original_vars;var_index++) {
                        string var_type = at.vars.at(var_index).second;
                        if(var_type == "robot") {
                            robot_number++;
                        } else {
                            if(var_type == "robotteam") { //Maybe later change this to deal with subtypes of robotteam
                                throw std::runtime_error("Tasks without RobotNumber attribute cannot have robotteam variable!");
                            }
                            bool is_robot_subtype = false;
                            for(sort_definition def : sort_definitions) {
                                if(std::find(def.declared_sorts.begin(),def.declared_sorts.end(),var_type) != def.declared_sorts.end()) {
                                    if(def.has_parent_sort) {
                                        if(def.parent_sort == "robot") {
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
    Function: parse_vars
    Objective: Parse vars in OCL declaration syntax

    @ Input: The string representing the var declaration
    @ Output: The vector with variable names and OCL types
*/ 
vector<pair<string,string>> parse_vars(string var_decl) {
    stringstream ss(var_decl);
    vector<pair<string,string>> vars;

    while(ss.good()) {
        string substr;

        smatch m;
        regex e1("[a-zA-z]+");
        regex e2("[a-zA-z]+([(][a-zA-z]+[)])?");

        getline(ss, substr, ',');

        stringstream aux(substr);
        string var_name, var_type;

        getline(aux, var_name, ':');
        aux >> var_type;

        regex_search(var_name,m,e1);
        var_name = m[0];

        regex_search(var_type,m,e2);
        var_type = m[0];

        vars.push_back(make_pair(var_name,var_type));

        if(var_name == "") {
            std::string var_err = "Invalid variable declaration " + substr + " in GM.";
            throw std::runtime_error(var_err);
        }
    }

    return vars;
}

/*
    Function: parse_forAll_expr
    Objective: Parse OCL forAll expression

    @ Input: The string representing the forAll expression
    @ Output: A vector with the iterated var at the first position, iteration var at the second
    position and condition at the third position
*/ 
vector<string> parse_forAll_expr(string expr) {
    bool error = false;

    vector<string> res;

    std::regex forall_reg("[a-zA-Z]+[a-zA-z_.0-9]*(->forAll)[(][a-zA-Z]+[a-zA-z_.0-9]*[ ]?[|][ ]?([a-zA-Z]+[a-zA-z_.0-9]*)?[)]");

    if(!std::regex_match(expr, forall_reg)) {
        error = true;
    }

    if(!error) {
        try {
            stringstream ss(expr);
            string aux;

            regex e1("[a-zA-Z]+[a-zA-z_.0-9]*");
            smatch m;

            getline(ss, aux, '>');
            regex_search(aux,m,e1);
            res.push_back(m[0]);

            getline(ss, aux, '|');
            aux = aux.substr(aux.find('(')+1);
            regex_search(aux,m,e1);
            res.push_back(m[0]);

            getline(ss, aux, ')');
            regex_search(aux,m,e1);
            res.push_back(m[0]);
        } catch(...) {
            error = true;
        }
    }

    if(res.at(0) == "" || res.at(1) == "" || error) {
        std::string forAll_err = "Invalid forAll statement " + expr + " in GM.";
        throw std::runtime_error(forAll_err);
    }

    return res;
}

/*
    Function: parse_achieve_condition
    Objective: Parse AchieveCondition, which must be a forAll statement. With this we create an
    AchieveCondition object and return it

    @ Input: The string representing the achieve condition
    @ Output: The generated achieve condition
*/ 
AchieveCondition parse_achieve_condition(string cond) {
    AchieveCondition a;
    if(cond.find("forAll") != std::string::npos) {
        a.has_forAll_expr = true;
    } else {
        a.has_forAll_expr = false;
    }

    if(a.has_forAll_expr) {
        vector<string> forAll_vars = parse_forAll_expr(cond);

        a.set_iterated_var(forAll_vars.at(0));
        a.set_iteration_var(forAll_vars.at(1));
        a.set_forAll_condition(forAll_vars.at(2));
    } else {
        a.set_condition(cond);
    }

    return a;
}

IterationRule parse_iterate_expr(string expr) {
    IterationRule it;
    stringstream ss(expr);
    string aux;

    regex e1("[a-zA-Z]+[a-zA-z_.0-9]*");
    smatch m;

    getline(ss, aux, '>');
    regex_search(aux,m,e1);
    it.iterated_var = m[0];

    getline(ss, aux, ';');
    aux.substr(aux.find('(')+1);
    regex_search(aux,m,e1);
    it.iteration_var = m[0];

    if(ss.str().find(":") == std::string::npos) {
        getline(ss, aux, '=');
        regex_search(aux,m,e1);
        it.result_var.first = m[0];
        it.result_var.second = "";
    } else {
        getline(ss, aux, ':');
        regex_search(aux,m,e1);
        it.result_var.first = m[0];

        getline(ss, aux, '=');
        regex_search(aux,m,e1);
        it.result_var.second = m[0];
    }

    regex e2("[a-zA-Z]+[a-zA-Z_.0-9]*");
    getline(ss, aux, '|');
    regex_search(aux,m,e2);
    it.result_init = m[0];

    regex e3("[a-zA-Z]{1}[a-zA-z0-9_]*(->([a-zA-z]+)[(]{1}[a-zA-Z]{1}[a-zA-z0-9]*[)]{1})?");
    getline(ss, aux, ')');
    string end_str;
    getline(ss,end_str);

    if(count(end_str.begin(),end_str.end(),')') > 0) {
        aux += ')';
    }
    regex_search(aux,m,e3);
    it.end_loop = m[0];

    return it;
}

/*
    Function: parse_select_expr
    Objective: Parse OCL select expression, returning a QueriedProperty object. This is done due to
    the fact that select statements are used in Query goals

    @ Input: The string representing the select expression
    @ Output: A QueriedProperty object generated from the select statement
*/ 
QueriedProperty parse_select_expr(string expr) {
    bool error = false;

    std::regex select_reg("[a-zA-Z]+[a-zA-z_.0-9]*(->select)[(][a-zA-Z]+[a-zA-z_.0-9]*[:][a-zA-z]+[a-zA-Z0-9]+[ ]?[|][ ]?([!]?[a-zA-Z]+[a-zA-z_.0-9]*|[a-zA-Z]+[a-zA-z_.0-9]*[ ]((==)|(!=))[ ]([a-zA-z]+[a-zA-Z0-9]+|\"[a-zA-z]+[a-zA-Z0-9]+\"))[)]");
    
    if(!std::regex_match(expr, select_reg)) {
        error = true;
    }

    QueriedProperty q;
    stringstream ss(expr);
    string aux;

    regex e1("[a-zA-Z]+[a-zA-z_.0-9]*");
    smatch m;

    getline(ss, aux, '>');
    regex_search(aux,m,e1);
    q.queried_var = m[0];

    getline(ss, aux, ':');
    aux = aux.substr(aux.find('(')+1);
    regex_search(aux,m,e1);
    q.query_var.first = m[0];

    if(ss.str().find(":") == std::string::npos) {
        q.query_var.second = "";
    } else {
        getline(ss, aux, '|');
        regex_search(aux,m,e1);
        q.query_var.second = m[0];
    }

    regex e2("[!a-zA-Z]{1}[a-zA-Z_.0-9]*");
    regex e3("[a-zA-Z]{1}[a-zA-Z_.0-9]*");
    if((ss.str().find("==") == std::string::npos) && (ss.str().find("!=") == std::string::npos)) {
        getline(ss, aux, ')');
        regex_search(aux,m,e2);
        q.query.push_back(m[0]);
    } else {
        getline(ss,aux,'=');
        regex_search(aux,m,e3);
        q.query.push_back(m[0]);
        
        if(ss.str().find("==") != std::string::npos) {
            q.query.push_back("==");
        } else {
            q.query.push_back("!=");
        }

        getline(ss,aux,')');
        regex_search(aux,m,e3);
        q.query.push_back(m[0]);
    }

    if(error == true) {
        std::string select_err = "Invalid select statement " + expr + " in GM.";
        throw std::runtime_error(select_err);
    }

    return q;
}

/*
    Function: parse_at_text
    Objective: Parse the text of an Abstract Task in the goal model

    @ Input: The string representing the text of the abstract task
    @ Output: A pair which contains the user-defined ID of the task and its description
*/ 
pair<string,string> parse_at_text(string text) {
    regex id("[AT]{2}[0-9]+");
    regex name("[a-zA-Z]+");
    smatch m;

    stringstream ss(text);
    pair<string,string> at;

    string aux;
    getline(ss, aux, ':');
    regex_search(aux,m,id);
    at.first = m[0];

    getline(ss, aux);
    regex_search(aux,m,name);
    at.second = m[0];

    return at;
}

/*
    Function: parse_goal_text
    Objective: Parse the text of a Goal in the goal model

    @ Input: The string representing the text of the goal
    @ Output: A pair which contains the user-defined ID of the goal and its description
*/ 
pair<string,string> parse_goal_text(string text) {
    regex id("[G]{1}[0-9]+");
    regex name("[a-zA-Z]+");
    smatch m;

    stringstream ss(text);
    pair<string,string> g;

    string aux;
    getline(ss, aux, ':');
    regex_search(aux,m,id);
    g.first = m[0];

    getline(ss, aux);
    regex_search(aux,m,name);
    g.second = m[0];

    return g;
}

/*vector<pair<string,string>> parse_var_mapping(string text) {
    vector<string> mappings;
    stringstream ss(text);

    while(ss.good()) {
        string mapping;
        getline(ss,mapping,',');
        mappings.push_back(mapping);
    }

    vector<pair<string,string>> parsed_mappings;

    regex hddl_varname("[\?]{1}[a-zA-Z]{1}[a-zA-Z_-]+"); //Here we are assuming HDDL variables can contain - and _
    regex varname("[a-zA-Z]{1}[a-zA-Z_0-9]+");
    smatch m;

    for(string map : mappings) {
        ss.clear();
        
        int cnt = 0;
        pair<string,string> splitted_mapping;
        std::replace(map.begin(),map.end(),':',' ');
        string var;
        stringstream aux(map);
        while(aux >> var) {
            if(cnt == 0) {
                regex_search(var,m,hddl_varname);
                splitted_mapping.first = m[0];
            } else {
                regex_search(var,m,varname);
                splitted_mapping.second = m[0];
            }
            
            cnt++;
        }

        parsed_mappings.push_back(splitted_mapping);
    }

    return parsed_mappings;
}*/

/*
    Function: parse_robot_number
    Objective: Parse the RobotNumber attribute from a task

    @ Input: The string representing the text of the RobotNumber attribute in the form "[n1,n2]"
    @ Output: A pair representing the lower and upper bounds
*/ 
pair<int,int> parse_robot_number(string text) {
    size_t begin, sep, end;

    begin = text.find("[");
    sep = text.find(",");
    end = text.find("]");

    int lower_bound, upper_bound;

    stringstream ss;
    ss << text.substr(begin+1,sep);
    ss >> lower_bound;
    ss.str("");
    ss << text.substr(sep+1,end-sep-1);
    ss >> upper_bound;

    return make_pair(lower_bound, upper_bound);
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
			
		if(node.custom_props.find("CreationCondition") != node.custom_props.end()) {
			c = get<Context>(node.custom_props["CreationCondition"]);

			std::cout << "\tType: " << c.type << std::endl;
			std::cout << "\tCondition: " << c.condition << std::endl;
		} else {
			std::cout << "\tNo Context" << std::endl;
		}
			
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
			cout << "Mapping: " << get<pair<string,string>>(gm_var_it->second).first << endl << endl;
		}
	}

}