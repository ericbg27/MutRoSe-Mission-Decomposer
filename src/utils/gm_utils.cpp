#include "gm_utils.hpp"

#include <sstream>
#include <regex>
#include <iostream>

using namespace std;

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

variant<pair<pair<predicate_definition,vector<string>>,bool>,pair<pair<predicate_definition,vector<string>>,pair<int,variant<bool,string>>>,bool> AchieveCondition::evaluate_condition(vector<SemanticMapping> semantic_mapping, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map) {
    if(has_forAll_expr) {
        string iteration_var_type;
        if(holds_alternative<pair<string,string>>(gm_var_map[iteration_var])) { // For now, the only valid condition
            iteration_var_type = std::get<pair<string,string>>(gm_var_map[iteration_var]).second;
        }

        vector<string> iterated_var_values = std::get<pair<vector<string>,string>>(gm_var_map[iterated_var]).first;

        variant<pair<pair<predicate_definition,vector<string>>,bool>,pair<pair<predicate_definition,vector<string>>,pair<int,variant<bool,string>>>,bool> evaluation = Condition::evaluate_condition(make_pair(iterated_var_values,iteration_var_type), semantic_mapping);

        return evaluation;
    } else {
        string cond = condition;

        std::replace(cond.begin(), cond.end(), '.', ' ');

        vector<string> split_cond;
                                
        stringstream ss(cond);
        string temp;
        while(ss >> temp) {
            split_cond.push_back(temp);
        }

        string variable;
        if(split_cond.at(0) == "not") {
            variable = split_cond.at(1);
        } else {
            variable = split_cond.at(0);
        }

        variant<pair<string,string>,pair<vector<string>,string>> var_value_and_type = get_var_value_and_type(gm_var_map, variable);

        variant<pair<pair<predicate_definition,vector<string>>,bool>,pair<pair<predicate_definition,vector<string>>,pair<int,variant<bool,string>>>,bool> evaluation = Condition::evaluate_condition(var_value_and_type,semantic_mapping);

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
AchieveCondition parse_achieve_condition(string cond) {
    AchieveCondition a;
    if(cond.find("forAll") != string::npos) {
        a.has_forAll_expr = true;
    } else {
        a.has_forAll_expr = false;
    }

    if(a.has_forAll_expr) {
        vector<string> forAll_vars = parse_forAll_expr(cond);

        a.set_iterated_var(forAll_vars.at(0));
        a.set_iteration_var(forAll_vars.at(1));
        a.set_condition(forAll_vars.at(2));
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

    if(ss.str().find(":") == string::npos) {
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

    std::regex select_reg1("[a-zA-Z]{1}[a-zA-z_.0-9]*(->select)[(][a-zA-Z]{1}[a-zA-z_.0-9]*[:][a-zA-z]+[a-zA-Z0-9]+[ ]*[|][ ]*([!]?[a-zA-Z]+[a-zA-z_.0-9]*)[)]");
    std::regex select_reg2("[a-zA-Z]{1}[a-zA-z_.0-9]*(->select)[(][a-zA-Z]{1}[a-zA-z_.0-9]*[:][a-zA-z]+[a-zA-Z0-9]+[ ]*[|][ ]*([a-zA-Z]+[a-zA-z_.0-9]*[ ]+((=)|(<>)){1}[ ]+([a-zA-z]+[a-zA-Z0-9]+|\"[a-zA-z]+[a-zA-Z0-9]+\"|([0-9]*[.])?[0-9]+))[)]");
    std::regex select_reg3("[a-zA-Z]{1}[a-zA-z_.0-9]*(->select)[(][a-zA-Z]{1}[a-zA-z_.0-9]*[:][a-zA-z]+[a-zA-Z0-9]+[ ]*[|][ ]*([a-zA-Z]+[a-zA-z_.0-9]*[ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+)[)]");
    std::regex select_reg4("[a-zA-Z]{1}[a-zA-z_.0-9]*(->select)[(][a-zA-Z]{1}[a-zA-z_.0-9]*[:][a-zA-z]+[a-zA-Z0-9]+[ ]*[|][ ]*([a-zA-Z]+[a-zA-z_.0-9]*[ ]+(in)[ ]+[a-zA-Z]+[a-zA-z_.0-9]*)[)]");

    if(!std::regex_match(expr, select_reg1) && !std::regex_match(expr, select_reg2) && !std::regex_match(expr, select_reg3) && !std::regex_match(expr, select_reg4)) {
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

    if(ss.str().find(":") == string::npos) {
        q.query_var.second = "";
    } else {
        getline(ss, aux, '|');
        regex_search(aux,m,e1);
        q.query_var.second = m[0];
    }

    regex e2("[!a-zA-Z]{1}[a-zA-Z_.0-9]*");
    regex e3("[a-zA-Z]{1}[a-zA-Z_.0-9]*");
    regex num("[0-9]+");
    if((ss.str().find(ocl_equal) == string::npos) && (ss.str().find(ocl_different) == string::npos) && (ss.str().find(spaced_ocl_in) == string::npos) && 
        (ss.str().substr(ss.str().find("(")).find(ocl_gt) == string::npos) && (ss.str().find(ocl_lt) == string::npos) && (ss.str().find(ocl_geq) == string::npos) && (ss.str().find(ocl_leq) == string::npos)) {
        getline(ss, aux, ')');
        if(regex_search(aux,m,e2)) {
            q.query.push_back(m[0]);
        } else {
            regex_search(aux,m,e3);
            q.query.push_back(m[0]);
        }
    } else {
        if(ss.str().find(ocl_equal) != string::npos || ss.str().find(ocl_different) != string::npos) {
            getline(ss,aux,'=');
            regex_search(aux,m,e3);
            q.query.push_back(m[0]);
            
            if(ss.str().find(ocl_equal) != string::npos) {
                q.query.push_back(ocl_equal);
            } else {
                q.query.push_back(ocl_different);
            }

            getline(ss,aux,')');
            regex_search(aux,m,e3);
            q.query.push_back(m[0]);
        } else if(ss.str().find(spaced_ocl_in) != string::npos) {
            vector<string> split_query;

            string temp;
            while(ss >> temp) {
                split_query.push_back(temp);
            }

            regex_search(split_query.at(0),m,e3);
            q.query.push_back(m[0]);

            q.query.push_back(split_query.at(1));

            regex_search(split_query.at(2),m,e3);
            q.query.push_back(m[0]);
        } else {
            getline(ss,aux,'(');
            char op;

            if(ss.str().substr(ss.str().find("(")).find(ocl_gt) != string::npos) {
                op = ocl_gt[0];
            } else if(ss.str().find(ocl_lt) != string::npos) {
                op = ocl_lt[0];
            } else if(ss.str().find(ocl_geq) != string::npos) {
                op = ocl_geq[0];
            } else if(ss.str().find(ocl_leq) != string::npos) {
                op = ocl_leq[0];
            }

            getline(ss,aux,op);
            regex_search(aux,m,e3);
            q.query.push_back(m[0]);

            string op_str(1, op);
            q.query.push_back(op_str);
            
            getline(ss,aux,')');
            regex_search(aux,m,num);
            q.query.push_back(m[0]);
        }
    }

    if(error == true) {
        string select_err = "Invalid select statement " + expr + " in GM.";

        throw std::runtime_error(select_err);
    }

    return q;
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

    std::regex forall_reg1("[a-zA-Z]+[a-zA-Z_.0-9]*(->forAll)[(][a-zA-Z]+[a-zA-z_.0-9]*[ ]*[|][ ]*([a-zA-Z]+[a-zA-z_.0-9]*)?[)]");
    std::regex forall_reg2("[a-zA-Z]+[a-zA-Z_.0-9]*(->forAll)[(][a-zA-Z]+[a-zA-z_.0-9]*[ ]*[|][ ]*([A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((=)|(<>)){1}[ ]+([0-9]*[.])?[0-9]+))[)]");
    std::regex forall_reg3("[a-zA-Z]+[a-zA-Z_.0-9]*(->forAll)[(][a-zA-Z]+[a-zA-z_.0-9]*[ ]*[|][ ]*([a-zA-Z]+[a-zA-z_.0-9]*[ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+)[)]");
    
    if(!std::regex_match(expr, forall_reg1) && !std::regex_match(expr, forall_reg2) && !std::regex_match(expr, forall_reg3)) {
        error = true;
    }

    if(!error) {
        try {
            stringstream ss(expr);
            string aux;

            regex e1("[a-zA-Z]+[a-zA-z_.0-9]*(([ ]+((=)|(<>)){1}[ ]+[0-9]+)|([ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+))?");
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