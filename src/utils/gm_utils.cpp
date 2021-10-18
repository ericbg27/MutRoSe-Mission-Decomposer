#include "gm_utils.hpp"

#include <sstream>
#include <regex>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include "../queryparser/queryparser.hpp"
#include "../conditionparser/conditionparser.hpp"

using namespace std;

int parse_query(const char* in);
int parse_condition(const char* in);

extern Query* parsed_query;
extern Condition* parsed_condition;

string AchieveCondition::get_iterated_var() {
    if(has_forAll_expr) {
        return iterated_var;
    } else {
        return "";
    }
}
        
string AchieveCondition::get_iteration_var() {
    if(has_forAll_expr) {
        return iteration_var;
    } else {
        return "";
    }
}

void AchieveCondition::set_iterated_var(std::string ivar) {
    iterated_var = ivar;
}

void AchieveCondition::set_iteration_var(std::string itvar) {
    iteration_var = itvar;
}

ConditionEvaluation* AchieveCondition::evaluate_condition(vector<SemanticMapping> semantic_mapping, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map) {
    if(has_forAll_expr) {
        string var_attr_regex = "([!]?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";
        string var_attr_regex2 = "(((\\bnot\\b)[ ]+){1}[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";
        string number_compare_regex = "[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((=)|(<>)){1}[ ]+([0-9]*[.])?[0-9]+){1}"; 
        string number_compare_regex2 = "[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+){1}";

        set<string> accepted_regex_patterns = {var_attr_regex, var_attr_regex2, number_compare_regex, number_compare_regex2};

        ConditionEvaluation* evaluation = Condition::evaluate_condition(gm_var_map, semantic_mapping, accepted_regex_patterns);

        return evaluation;
    } else {
        string var_attr_regex = "([!]?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";
        string var_attr_regex2 = "(((\\bnot\\b)[ ]+){1}[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";
        string number_compare_regex = "[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((=)|(<>)){1}[ ]+([0-9]*[.])?[0-9]+){1}"; 
        string number_compare_regex2 = "[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+){1}";

        set<string> accepted_regex_patterns = {var_attr_regex, var_attr_regex2, number_compare_regex, number_compare_regex2};

        ConditionEvaluation* evaluation = Condition::evaluate_condition(gm_var_map,semantic_mapping, accepted_regex_patterns);

        return evaluation;
    }
}

/*
    Function: parse_achieve_condition
    Objective: Parse AchieveCondition, which must be a forAll statement. With this we create an
    AchieveCondition object and return it

    @ Input: The string representing the achieve condition
    @ Output: The generated achieve condition
*/ 
AchieveCondition parse_achieve_condition(string cond, bool universal) {
    AchieveCondition a;
    if(universal) {
        if(cond.find("forAll") == string::npos) {
            string universal_achieve_condition_error = "Universal achieve condition must contain forAll statement!";

            throw std::runtime_error(universal_achieve_condition_error);
        }

        a.has_forAll_expr = true;
    } else {
        if(cond.find("forAll") != string::npos) {
            string achieve_condition_error = "Non-universal achieve condition must not contain forAll statement!";

            throw std::runtime_error(achieve_condition_error);
        }

        a.has_forAll_expr = false;
    }

    if(a.has_forAll_expr) {
        vector<string> forAll_vars = parse_forAll_expr(cond);

        a.set_iterated_var(forAll_vars.at(0));
        a.set_iteration_var(forAll_vars.at(1));

        parse_condition(forAll_vars.at(2).c_str()); //In parser we are considering leading and trailing spaces!

        a.set_condition(parsed_condition->get_condition());
        a.set_is_and(parsed_condition->get_is_and());
    } else {
        parse_condition(cond.c_str());

        a.set_condition(parsed_condition->get_condition());
        a.set_is_and(parsed_condition->get_is_and());
    }

    return a;
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

    std::regex select_reg(select_regex_exp);

    if(!std::regex_match(expr, select_reg)) {
        error = true;
    }

    QueriedProperty q;
    stringstream ss(expr);
    string aux;

    regex e1("[a-zA-Z]+[\\w.]*");
    smatch m;

    getline(ss, aux, '>');
    regex_search(aux,m,e1);
    q.queried_var = m[0];

    getline(ss, aux, ':');
    aux = aux.substr(aux.find('(')+1);
    regex_search(aux,m,e1);
    q.query_var.first = m[0];

    if(ss.str().find(":") == string::npos) {
        q.query_var.second = "";
    } else {
        getline(ss, aux, '|');
        regex_search(aux,m,e1);
        q.query_var.second = m[0];
    }

    getline(ss,aux,')');
    parse_query(aux.c_str());

    q.query = parsed_query;

    if(error == true) {
        string select_err = "Invalid select statement " + expr + " in GM.";

        throw std::runtime_error(select_err);
    }

    return q;
}

Context parse_context_condition(string condition, string context_type) {
    Context c;

    string cond = condition;
    if(context_type == context_trigger_prop) {
        c.set_context_type(trigger_context_type);
        c.set_condition(cond);
    } else if(context_type == context_condition_prop) {
        c.set_context_type(condition_context_type);

        parse_condition(cond.c_str());

        c.set_condition(parsed_condition->get_condition());
        c.set_is_and(parsed_condition->get_is_and());
    }

    return c;
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
        regex e1("[a-zA-Z]{1}[a-zA-z0-9]*");
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

    string overall_forall_regex = forall_regex_exp + "(.)*" + end_forall_regex_exp;
    std::regex general_forall(overall_forall_regex);
    
    /*if(!std::regex_match(expr, forall_reg1) && !std::regex_match(expr, forall_reg2) && !std::regex_match(expr, forall_reg3)) {
        error = true;
    }*/

    if(!std::regex_match(expr, general_forall)) {
        error = true;
    }

    if(!error) {
        try {
            stringstream ss(expr);
            string aux;

            regex e1("[a-zA-Z]+[\\w.]*(([ ]+((=)|(<>)){1}[ ]+[0-9]+)|([ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+))?");
            smatch m;

            getline(ss, aux, '>');
            regex_search(aux,m,e1);
            res.push_back(m[0]);

            getline(ss, aux, '|');
            aux = aux.substr(aux.find('(')+1);
            regex_search(aux,m,e1);
            res.push_back(m[0]);

            getline(ss, aux, ')');
            //regex_search(aux,m,e1);
            //res.push_back(m[0]);
            res.push_back(aux);
        } catch(...) {
            error = true;
        }
    }

    if(error) {
        string forAll_err = "Invalid forAll statement " + expr + " in GM.";

        throw std::runtime_error(forAll_err);
    }

    if(res.at(0) == "" || res.at(1) == "") {
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
variant<int,pair<int,int>> parse_robot_number(string text) {
    regex robot_num_range("\\[\\d+,\\d+\\]");
    regex robot_num_fixed("\\d+");

    if(std::regex_match(text,robot_num_range)) {
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
    } else if(std::regex_match(text,robot_num_fixed)) {
        stringstream ss(text);

        int robot_num;
        ss >> robot_num;

        return robot_num;
    } else {
        string invalid_robot_num_err = "Invalid declaration of robot number: " + text;

        throw std::runtime_error(invalid_robot_num_err);
    }
}