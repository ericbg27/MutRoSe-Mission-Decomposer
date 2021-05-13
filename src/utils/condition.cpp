#include "condition.hpp"

#include <regex>
#include <sstream>
#include <iostream>

#include "../utils/gm_utils.hpp"

using namespace std;

string Condition::get_condition() {
    return condition;
}

void Condition::set_condition(string cond) {
    condition = cond;
}

variant<pair<pair<predicate_definition,vector<string>>,bool>,pair<pair<predicate_definition,vector<string>>,pair<int,bool>>,bool> Condition::evaluate_condition(variant<pair<string,string>,pair<vector<string>,string>> var_value_and_type, vector<SemanticMapping> semantic_mapping) {
    regex r1("(((\\bnot\\b)[ ]+)?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}"); // (not) [VAR].[ATTR]
    regex r2("[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((==)|(!=)){1}[ ]+[\"][A-Za-z0-9]*[\"]){1}"); // [VAR].[ATTR] == "[VALUE]" || [VAR].[ATTR] != "[VALUE]"
    regex r3("[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((==)|(!=)){1}[ ]+[0-9]+){1}"); // [VAR].[ATTR] == [INTVALUE] || [VAR].[ATTR] != [INTVALUE]
    
    bool invalid_condition = false; 

    string cond = condition;
    if(regex_match(condition, r1)) {
        predicate_definition map_pred;
        vector<string> pred_args;

        std::replace(cond.begin(), cond.end(), '.', ' ');

        vector<string> split_cond;
                                
        stringstream ss(cond);
        string temp;
        while(ss >> temp) {
            split_cond.push_back(temp);
        }

        bool negative_condition = false;
        string variable, attribute;
        if(split_cond.at(0) == "not") {
            negative_condition = true;
            variable = split_cond.at(1);
            attribute = split_cond.at(2);
        } else {
            variable = split_cond.at(0);
            attribute = split_cond.at(1);
        }

        if(holds_alternative<pair<vector<string>,string>>(var_value_and_type)) { // Collection type variables
            pair<vector<string>,string> value_and_type = std::get<pair<vector<string>,string>>(var_value_and_type);

            string type = value_and_type.second;
            std::transform(type.begin(),type.end(),type.begin(),::toupper);

            if(parse_gm_var_type(type) == "COLLECTION") {
                size_t type_begin = value_and_type.second.find("(")+1;
                size_t type_end = value_and_type.second.find(")",type_begin);

                value_and_type.second = value_and_type.second.substr(type_begin,type_end-type_begin);
            }

            for(SemanticMapping sm : semantic_mapping) {
                if(sm.get_mapping_type() == "attribute") {
                    if(sm.get_mapped_type() == "predicate") {
                        string relation = std::get<string>(sm.get_prop("relation"));
                        if(relation == value_and_type.second) {
                            if(std::get<string>(sm.get_prop("name")) == attribute) {
                                map_pred = std::get<predicate_definition>(sm.get_prop("map"));
                                break;
                            }
                        }
                    }
                }
            }

            pred_args = value_and_type.first;
        } else {
            pair<string,string> value_and_type = std::get<pair<string,string>>(var_value_and_type);

            for(SemanticMapping sm : semantic_mapping) {
                if(sm.get_mapping_type() == "attribute") {
                    if(sm.get_mapped_type() == "predicate") {
                        string relation = std::get<string>(sm.get_prop("relation"));
                        if(relation == value_and_type.second) {
                            if(std::get<string>(sm.get_prop("name")) == attribute) {
                                map_pred = std::get<predicate_definition>(sm.get_prop("map"));
                                break;
                            }
                        }
                    }
                }
            }
        
            pred_args.push_back(value_and_type.first);
        }

        if(!invalid_condition) {
            return make_pair(make_pair(map_pred,pred_args), negative_condition);
        }
    } else if(regex_match(condition, r2)) {
        // TO CHANGE
        /*std::replace(cond.begin(), cond.end(), '.', ' ');

        vector<string> split_cond;
                                
        stringstream ss(cond);
        string temp;
        while(ss >> temp) {
            split_cond.push_back(temp);
        }

        bool negative_condition = false;
        string variable, attribute;
        if(split_cond.at(2) == "!=") {
            negative_condition = true;
        } 
        variable = split_cond.at(0);
        attribute = split_cond.at(1);

        pair<string,string> value_and_type = std::get<pair<string,string>>(var_value_and_type);

        return true;*/
        invalid_condition = true;
    } else if(regex_match(condition, r3)) {
        std::replace(cond.begin(), cond.end(), '.', ' ');

        vector<string> split_cond;

        stringstream ss(cond);
        string temp;
        while(ss >> temp) {
            split_cond.push_back(temp);
        }

        bool negative_condition = false;
        string variable, attribute;
        if(split_cond.at(2) == "!=") {
            negative_condition = true;
        } 
        variable = split_cond.at(0);
        attribute = split_cond.at(1);

        vector<string> pred_args;
        predicate_definition map_pred;

        if(holds_alternative<pair<vector<string>,string>>(var_value_and_type)) {
            pair<vector<string>,string> value_and_type = std::get<pair<vector<string>,string>>(var_value_and_type);

            string type = value_and_type.second;
            std::transform(type.begin(),type.end(),type.begin(),::toupper);

            if(parse_gm_var_type(type) == "COLLECTION") {
                size_t type_begin = value_and_type.second.find("(")+1;
                size_t type_end = value_and_type.second.find(")",type_begin);

                value_and_type.second = value_and_type.second.substr(type_begin,type_end-type_begin);
            }

            for(SemanticMapping sm : semantic_mapping) {
                if(sm.get_mapping_type() == "attribute") {
                    if(sm.get_mapped_type() == "function") {
                        string relation = std::get<string>(sm.get_prop("relation"));
                        if(relation == value_and_type.second) {
                            if(std::get<string>(sm.get_prop("name")) == attribute) {
                                map_pred = std::get<predicate_definition>(sm.get_prop("map"));
                                break;
                            }
                        }
                    }
                }
            }

            pred_args = value_and_type.first;
        } else {
            pair<string,string> value_and_type = std::get<pair<string,string>>(var_value_and_type);

            for(SemanticMapping sm : semantic_mapping) {
                if(sm.get_mapping_type() == "attribute") {
                    if(sm.get_mapped_type() == "function") {
                        string relation = std::get<string>(sm.get_prop("relation"));
                        if(relation == value_and_type.second) {
                            if(std::get<string>(sm.get_prop("name")) == attribute) {
                                map_pred = std::get<predicate_definition>(sm.get_prop("map"));
                                break;
                            }
                        }
                    }
                }
            }
        
            pred_args.push_back(value_and_type.first);
        }

        int pred_value = stoi(split_cond.at(3));

        return make_pair(make_pair(map_pred,pred_args),make_pair(pred_value,negative_condition));
    } else if(condition == "") {
        return true;
    } else {
        invalid_condition = true;
    }

    if(invalid_condition) {
        string invalid_condition_pattern_error = "Invalid condition: " + condition;

        throw std::runtime_error(invalid_condition_pattern_error);
    }

    return true;
}

variant<pair<string,string>,pair<vector<string>,string>> Condition::get_var_value_and_type(map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, string variable) {
    return gm_var_map[variable];
}