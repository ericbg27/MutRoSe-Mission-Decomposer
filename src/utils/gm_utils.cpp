#include "gm_utils.hpp"

#include <sstream>
#include <regex>

using namespace std;

/*
    Function: get_node_name
    Objective: Return the user-defined ID of some node given its text

    @ Input: The node text
    @ Output: The node name
*/ 
string get_node_name(string node_text) {
    size_t pos = node_text.find(":");
    string node_name;
    if(pos != string::npos) {
        node_name = node_text.substr(0,pos);
    }

    std::transform(node_name.begin(),node_name.end(),node_name.begin(),::toupper);

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
    if(var_type.find("SEQUENCE") != string::npos) {
        return "COLLECTION";
    }

    return "VALUE";
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
        //regex e2("[a-zA-z]+([(][a-zA-z]+[)])?");
        regex e2("[a-zA-z0-9]+([(][a-zA-z]+[)])?");

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
            string var_err = "Invalid variable declaration " + substr + " in GM.";
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
        string forAll_err = "Invalid forAll statement " + expr + " in GM.";
        throw std::runtime_error(forAll_err);
    }

    return res;
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
