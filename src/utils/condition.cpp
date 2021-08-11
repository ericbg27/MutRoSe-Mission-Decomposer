#include "condition.hpp"

#include <regex>
#include <sstream>
#include <iostream>

#include "../utils/gm_utils.hpp"
#include "../utils/math_utils.hpp"
#include "../utils/predicate_utils.hpp"

using namespace std;

ConditionExpression::ConditionExpression() {
    ConditionExpression* null_condition = nullptr;
    this->left_expr = null_condition;

    ConditionExpression* null_condition2 = nullptr;
    this->right_expr = null_condition2;

    this->is_and = true;
}

ConditionExpression::ConditionExpression(variant<pred_vector,func_pred_vector,ConditionExpression*> le, variant<pred_vector,func_pred_vector,ConditionExpression*> re, bool is_and) {
    this->left_expr = le;
    this->right_expr = re;
    this->is_and = is_and;
}

bool ConditionExpression::evaluate_expression(std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions) {
    bool left_res = true, right_res = true;
    
    if(holds_alternative<ConditionExpression*>(left_expr)) {
        if(std::get<ConditionExpression*>(left_expr) != NULL) {
            left_res = std::get<ConditionExpression*>(left_expr)->evaluate_expression(world_state, world_state_functions);
        } else {
            left_res = true;
        }
    } else {
        if(holds_alternative<pred_vector>(left_expr)) {
            for(ground_literal forAll_pred : std::get<pred_vector>(left_expr)) {
                for(ground_literal state : world_state) {
                    bool same_predicate = is_same_predicate(state, forAll_pred);
                            
                    if(same_predicate) {
                        if(state.positive != forAll_pred.positive) {
                            left_res = false;
                            break;
                        }
                    }
                }

                if(!left_res) {
                    break;
                }
            }
        } else {
            for(pair<ground_literal,variant<pair<bool,variant<int,float>>,pair<string,variant<int,float>>>> forAll_pred : std::get<func_pred_vector>(left_expr)) {
                for(pair<ground_literal,variant<int,float>> state : world_state_functions) {
                    bool same_predicate = is_same_predicate(state.first, forAll_pred.first);
                                    
                    if(!same_predicate) {
                        continue;
                    }

                    if(holds_alternative<pair<bool,variant<int,float>>>(forAll_pred.second)) {
                        pair<bool,variant<int,float>> val_and_flag = std::get<pair<bool,variant<int,float>>>(forAll_pred.second);
                        if(val_and_flag.first) {
                            if(holds_alternative<int>(state.second)) {
                                int state_val = std::get<int>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    left_res = (state_val != std::get<int>(val_and_flag.second));
                                } else {
                                    left_res = !compare_int_and_float(state_val, std::get<float>(val_and_flag.second));
                                }
                            } else {
                                float state_val = std::get<float>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    left_res = !compare_int_and_float(std::get<int>(val_and_flag.second), state_val);
                                } else {
                                    left_res = !compare_floats(state_val, std::get<float>(val_and_flag.second));
                                }
                            }

                            if(!left_res) {
                                break;
                            }
                        } else {
                            if(holds_alternative<int>(state.second)) {
                                int state_val = std::get<int>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    left_res = (state_val == std::get<int>(val_and_flag.second));
                                } else {
                                    left_res = compare_int_and_float(state_val, std::get<float>(val_and_flag.second));
                                }
                            } else {
                                float state_val = std::get<float>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    left_res = compare_int_and_float(std::get<int>(val_and_flag.second), state_val);
                                } else {
                                    left_res = compare_floats(state_val, std::get<float>(val_and_flag.second));
                                }
                            }
                        }
                    } else {
                        pair<string,variant<int,float>> op_and_val = std::get<pair<string,variant<int,float>>>(forAll_pred.second);

                        if(holds_alternative<int>(state.second)) {
                            int state_val = std::get<int>(state.second);

                            if(holds_alternative<int>(op_and_val.second)) {
                                if(op_and_val.first == ocl_gt) {
                                    left_res = state_val > std::get<int>(op_and_val.second);
                                } else if(op_and_val.first == ocl_lt) {
                                    left_res = state_val < std::get<int>(op_and_val.second);
                                } else if(op_and_val.first == ocl_geq) {
                                    left_res = state_val >= std::get<int>(op_and_val.second);
                                } else if(op_and_val.first == ocl_leq) {
                                    left_res = state_val <= std::get<int>(op_and_val.second);
                                }
                            } else {
                                if(op_and_val.first == ocl_gt) {
                                    left_res = greater_than_int_and_float(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_lt) {
                                    left_res = greater_than_float_and_int(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_geq) {
                                    left_res = !greater_than_float_and_int(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_leq) {
                                    left_res = !greater_than_int_and_float(state_val, std::get<float>(op_and_val.second));
                                }
                            }
                        } else {
                            float state_val = std::get<float>(state.second);

                            if(holds_alternative<int>(op_and_val.second)) {
                                if(op_and_val.first == ocl_gt) {
                                    left_res = greater_than_float_and_int(std::get<int>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_lt) {
                                    left_res = greater_than_int_and_float(std::get<int>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_geq) {
                                    left_res = !greater_than_int_and_float(std::get<int>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_leq) {
                                    left_res = !greater_than_float_and_int(std::get<int>(op_and_val.second), state_val);
                                }
                            } else {
                                if(op_and_val.first == ocl_gt) {
                                    left_res = greater_than_floats(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_lt) {
                                    left_res = greater_than_floats(std::get<float>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_geq) {
                                    left_res = !greater_than_floats(std::get<float>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_leq) {
                                    left_res = !greater_than_floats(state_val, std::get<float>(op_and_val.second));
                                }
                            }
                        }

                        if(!left_res) break;
                    }
                }

                if(!left_res) {
                    break;
                }
            }
        }
    }

    if(holds_alternative<ConditionExpression*>(right_expr)) {
        if(std::get<ConditionExpression*>(right_expr) != NULL) {
            right_res = std::get<ConditionExpression*>(right_expr)->evaluate_expression(world_state, world_state_functions);
        } else {
            right_res = true;
        }
    } else {
        if(holds_alternative<pred_vector>(right_expr)) {
            for(ground_literal forAll_pred : std::get<pred_vector>(right_expr)) {
                for(ground_literal state : world_state) {
                    bool same_predicate = is_same_predicate(state, forAll_pred);
                            
                    if(same_predicate) {
                        if(state.positive != forAll_pred.positive) {
                            right_res = false;
                            break;
                        } else {
                            break;
                        }
                    }
                }

                if(!right_res) {
                    break;
                }
            }
        } else {
            for(pair<ground_literal,variant<pair<bool,variant<int,float>>,pair<string,variant<int,float>>>> forAll_pred : std::get<func_pred_vector>(right_expr)) {
                for(pair<ground_literal,variant<int,float>> state : world_state_functions) {
                    bool same_predicate = is_same_predicate(state.first, forAll_pred.first);
                                    
                    if(!same_predicate) {
                        continue;
                    }

                    if(holds_alternative<pair<bool,variant<int,float>>>(forAll_pred.second)) {
                        pair<bool,variant<int,float>> val_and_flag = std::get<pair<bool,variant<int,float>>>(forAll_pred.second);
                        if(val_and_flag.first) {
                            if(holds_alternative<int>(state.second)) {
                                int state_val = std::get<int>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    right_res = (state_val != std::get<int>(val_and_flag.second));
                                } else {
                                    right_res = !compare_int_and_float(state_val, std::get<float>(val_and_flag.second));
                                }
                            } else {
                                float state_val = std::get<float>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    right_res = !compare_int_and_float(std::get<int>(val_and_flag.second), state_val);
                                } else {
                                    right_res = !compare_floats(state_val, std::get<float>(val_and_flag.second));
                                }
                            }

                            if(!right_res) {
                                break;
                            }
                        } else {
                            if(holds_alternative<int>(state.second)) {
                                int state_val = std::get<int>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    right_res = (state_val == std::get<int>(val_and_flag.second));
                                } else {
                                    right_res = compare_int_and_float(state_val, std::get<float>(val_and_flag.second));
                                }
                            } else {
                                float state_val = std::get<float>(state.second);

                                if(holds_alternative<int>(val_and_flag.second)) {
                                    right_res = compare_int_and_float(std::get<int>(val_and_flag.second), state_val);
                                } else {
                                    right_res = compare_floats(state_val, std::get<float>(val_and_flag.second));
                                }
                            }
                        }
                    } else {
                        pair<string,variant<int,float>> op_and_val = std::get<pair<string,variant<int,float>>>(forAll_pred.second);

                        if(holds_alternative<int>(state.second)) {
                            int state_val = std::get<int>(state.second);

                            if(holds_alternative<int>(op_and_val.second)) {
                                if(op_and_val.first == ocl_gt) {
                                    right_res = state_val > std::get<int>(op_and_val.second);
                                } else if(op_and_val.first == ocl_lt) {
                                    right_res = state_val < std::get<int>(op_and_val.second);
                                } else if(op_and_val.first == ocl_geq) {
                                    right_res = state_val >= std::get<int>(op_and_val.second);
                                } else if(op_and_val.first == ocl_leq) {
                                    right_res = state_val <= std::get<int>(op_and_val.second);
                                }
                            } else {
                                if(op_and_val.first == ocl_gt) {
                                    right_res = greater_than_int_and_float(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_lt) {
                                    right_res = greater_than_float_and_int(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_geq) {
                                    right_res = !greater_than_float_and_int(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_leq) {
                                    right_res = !greater_than_int_and_float(state_val, std::get<float>(op_and_val.second));
                                }
                            }
                        } else {
                            float state_val = std::get<float>(state.second);

                            if(holds_alternative<int>(op_and_val.second)) {
                                if(op_and_val.first == ocl_gt) {
                                    right_res = greater_than_float_and_int(std::get<int>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_lt) {
                                    right_res = greater_than_int_and_float(std::get<int>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_geq) {
                                    right_res = !greater_than_int_and_float(std::get<int>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_leq) {
                                    right_res = !greater_than_float_and_int(std::get<int>(op_and_val.second), state_val);
                                }
                            } else {
                                if(op_and_val.first == ocl_gt) {
                                    right_res = greater_than_floats(state_val, std::get<float>(op_and_val.second));
                                } else if(op_and_val.first == ocl_lt) {
                                    right_res = greater_than_floats(std::get<float>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_geq) {
                                    right_res = !greater_than_floats(std::get<float>(op_and_val.second), state_val);
                                } else if(op_and_val.first == ocl_leq) {
                                    right_res = !greater_than_floats(state_val, std::get<float>(op_and_val.second));
                                }
                            }
                        }

                        if(!right_res) break;
                    }
                }

                if(!right_res) {
                    break;
                }
            }
        }
    }

    if(is_and) {
        return (left_res && right_res);
    } else {
        return (left_res || right_res);
    }
}

ConditionExpression* ConditionEvaluation::get_evaluation_predicates() {
    if(holds_alternative<pair<ConditionEvaluation*,ConditionEvaluation*>>(evaluation)) {
        pair<ConditionEvaluation*,ConditionEvaluation*> eval = std::get<pair<ConditionEvaluation*,ConditionEvaluation*>>(evaluation);

        ConditionExpression* eval_preds1 = eval.first->get_evaluation_predicates();
        ConditionExpression* eval_preds2 = eval.second->get_evaluation_predicates();

        ConditionExpression* result_expr = new ConditionExpression(eval_preds1, eval_preds2, is_and);

        return result_expr;
    } else if(holds_alternative<pair<pair<predicate_definition,vector<string>>,variant<pair<variant<int,float>,variant<bool,string>>,bool>>>(evaluation)) {
        auto eval = std::get<pair<pair<predicate_definition,vector<string>>,variant<pair<variant<int,float>,variant<bool,string>>,bool>>>(evaluation);
        
        if(holds_alternative<bool>(eval.second)) {
            pair<pair<predicate_definition,vector<string>>,bool> e = make_pair(eval.first,std::get<bool>(eval.second));

            pred_vector predicates;

            for(string value : e.first.second) {
                ground_literal aux;

                aux.predicate = e.first.first.name;
                aux.positive = !e.second;
                aux.args.push_back(value);

                predicates.push_back(aux);
            } 

            ConditionExpression* empty_expr = new ConditionExpression();
            ConditionExpression* result_expr = new ConditionExpression(predicates, empty_expr, true);

            return result_expr;
        } else {
            pair<pair<predicate_definition,vector<string>>,pair<variant<int,float>,variant<bool,string>>> e = make_pair(eval.first, std::get<pair<variant<int,float>,variant<bool,string>>>(eval.second));
        
            func_pred_vector func_predicates;

            for(string value : e.first.second) {
                ground_literal aux;

                aux.predicate = e.first.first.name;
                aux.args.push_back(value);

                if(holds_alternative<bool>(e.second.second)) {
                    pair<bool,variant<int,float>> val_and_flag = make_pair(std::get<bool>(e.second.second), e.second.first);

                    func_predicates.push_back(make_pair(aux,val_and_flag));
                } else {
                    pair<string,variant<int,float>> op_and_val = make_pair(std::get<string>(e.second.second), e.second.first);

                    func_predicates.push_back(make_pair(aux,op_and_val));
                }
            }

            ConditionExpression* empty_expr = new ConditionExpression();
            ConditionExpression* result_expr = new ConditionExpression(func_predicates, empty_expr, true);

            return result_expr;
        }
    }

    ConditionExpression* empty_expr = new ConditionExpression();
    return empty_expr;
}

void ConditionEvaluation::set_evaluation(variant<pair<pair<predicate_definition,vector<string>>,variant<pair<variant<int,float>,variant<bool,string>>,bool>>,bool,pair<ConditionEvaluation*,ConditionEvaluation*>> eval) {
    evaluation = eval;
}

variant<pair<pair<predicate_definition,vector<string>>,variant<pair<variant<int,float>,variant<bool,string>>,bool>>,bool,pair<ConditionEvaluation*,ConditionEvaluation*>> ConditionEvaluation::get_evaluation() {
    return evaluation;
}

void ConditionEvaluation::set_is_and(bool a) {
    is_and = a;
}

bool ConditionEvaluation::get_is_and() {
    return is_and;
}

void Condition::set_condition(variant<string,pair<Condition*,Condition*>> cond) {
    condition = cond;
}

variant<string,pair<Condition*,Condition*>> Condition::get_condition() {
    return condition;
}

void Condition::set_is_and(bool a) {
    is_and = a;
}

bool Condition::get_is_and() {
    return is_and;
}

ConditionEvaluation* Condition::evaluate_condition(variant<pair<string,string>,pair<vector<string>,string>> var_value_and_type, vector<SemanticMapping> semantic_mapping) {
    //regex r1("(((\\bnot\\b)[ ]+)?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}"); // (not) [VAR].[ATTR]
    regex r1("([!]?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}"); // ![VAR].[ATTR]
    regex r2("[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((=)|(<>)){1}[ ]+[\"][A-Za-z0-9]*[\"]){1}"); // [VAR].[ATTR] = "[VALUE]" || [VAR].[ATTR] <> "[VALUE]"
    regex r3("[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((=)|(<>)){1}[ ]+([0-9]*[.])?[0-9]+){1}"); // [VAR].[ATTR] = [INTVALUE | FLOATVALUE] || [VAR].[ATTR] <> [INTVALUE | FLOATVALUE]
    regex r4("[A-Za-z]+[A-Za-z0-9_]*[.][A-za-z]+[A-za-z_]*([ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([0-9]*[.])?[0-9]+){1}"); // [VAR].[ATTR] > [INTVALUE | FLOATVALUE] || [VAR].[ATTR] < [INTVALUE | FLOATVALUE] || [VAR].[ATTR] >= [INTVALUE | FLOATVALUE] || [VAR].[ATTR] <= [INTVALUE | FLOATVALUE]
    //TODO: insert expressions using the in operator
    
    bool invalid_condition = false; 

    if(holds_alternative<pair<Condition*,Condition*>>(condition)) {
        pair<Condition*,Condition*> cond = std::get<pair<Condition*,Condition*>>(condition);

        ConditionEvaluation* eval1 = cond.first->evaluate_condition(var_value_and_type, semantic_mapping);
        ConditionEvaluation* eval2 = cond.second->evaluate_condition(var_value_and_type, semantic_mapping);

        ConditionEvaluation* final_eval = new ConditionEvaluation();
        final_eval->set_evaluation(make_pair(eval1,eval2));

        final_eval->set_is_and(is_and);

        return final_eval;
    } else {
        string cond = std::get<string>(condition);

        if(regex_match(cond, r1)) {
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
            if(split_cond.at(0).find("!") == 0) {
                negative_condition = true;
                variable = split_cond.at(0).substr(1);
                attribute = split_cond.at(1);
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
                            string relation = std::get<string>(sm.get_prop(relatesto_key));
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
                            string relation = std::get<string>(sm.get_prop(relatesto_key));
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

            if(map_pred.name == "") {
                invalid_condition = true;
            }

            if(!invalid_condition) {
                ConditionEvaluation* eval = new ConditionEvaluation();
                eval->set_evaluation(make_pair(make_pair(map_pred,pred_args), negative_condition));

                return eval;
            }
        } else if(regex_match(cond, r2)) {
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
            if(split_cond.at(2) == "<>") {
                negative_condition = true;
            } 
            variable = split_cond.at(0);
            attribute = split_cond.at(1);

            pair<string,string> value_and_type = std::get<pair<string,string>>(var_value_and_type);

            return true;*/
            invalid_condition = true;
        } else if(regex_match(cond, r3)) {
            std::replace(cond.begin(), cond.end(), '.', ' ');

            vector<string> split_cond;

            stringstream ss(cond);
            string temp;
            while(ss >> temp) {
                split_cond.push_back(temp);
            }

            bool negative_condition = false;
            string variable, attribute;
            if(split_cond.at(2) == "<>") {
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
                            string relation = std::get<string>(sm.get_prop(relatesto_key));
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
                            string relation = std::get<string>(sm.get_prop(relatesto_key));
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

            if(map_pred.name == "") {
                invalid_condition = true;
            }

            if(!invalid_condition) {
                variant<int,float> pred_value;

                if(split_cond.at(3).find(".") == string::npos) {
                    pred_value = stoi(split_cond.at(3));
                } else {
                    pred_value = static_cast<float>(::atof(split_cond.at(3).c_str()));
                }

                ConditionEvaluation* eval = new ConditionEvaluation();
                eval->set_evaluation(make_pair(make_pair(map_pred,pred_args),make_pair(pred_value,negative_condition)));

                return eval;
            }
        } else if(regex_match(cond, r4)) {
            std::replace(cond.begin(), cond.end(), '.', ' ');

            vector<string> split_cond;

            stringstream ss(cond);
            string temp;
            while(ss >> temp) {
                split_cond.push_back(temp);
            }

            string variable, attribute;
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
                            string relation = std::get<string>(sm.get_prop(relatesto_key));
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
                            string relation = std::get<string>(sm.get_prop(relatesto_key));
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

            if(map_pred.name == "") {
                invalid_condition = true;
            }

            if(!invalid_condition) {
                string op = split_cond.at(2);
                variant<int,float> pred_value;

                if(split_cond.at(3).find(".") == string::npos) {
                    pred_value = stoi(split_cond.at(3));
                } else {
                    pred_value = static_cast<float>(::atof(split_cond.at(3).c_str()));
                }

                ConditionEvaluation* eval = new ConditionEvaluation();
                eval->set_evaluation(make_pair(make_pair(map_pred,pred_args),make_pair(pred_value,op)));

                return eval;
            }
        } else if(cond == "") {
            ConditionEvaluation* eval = new ConditionEvaluation();
            eval->set_evaluation(true);
            
            return eval;
        } else {
            invalid_condition = true;
        }

        if(invalid_condition) {
            string invalid_condition_pattern_error = "Invalid condition: " + cond;

            throw std::runtime_error(invalid_condition_pattern_error);
        }

        ConditionEvaluation* eval = new ConditionEvaluation();
        eval->set_evaluation(true);
            
        return eval;
    }
}

variant<pair<string,string>,pair<vector<string>,string>> Condition::get_var_value_and_type(map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, string variable) {
    return gm_var_map[variable];
}