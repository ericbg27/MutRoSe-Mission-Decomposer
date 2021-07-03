#include "predicate_utils.hpp"

#include "math_utils.hpp"

using namespace std;

bool is_same_predicate(variant<literal,ground_literal> pred1, variant<literal,ground_literal> pred2) {
    if(holds_alternative<literal>(pred1)) {
        if(holds_alternative<literal>(pred2)) {
            literal p1 = std::get<literal>(pred1);
            literal p2 = std::get<literal>(pred2);
            
            if(p1.predicate == p2.predicate) {
                bool equal_args = true;
                
                int arg_index = 0;
                for(string arg : p1.arguments) {
                    if(arg != p2.arguments.at(arg_index)) {
                        equal_args = false;
                        break;
                    }

                    arg_index++;
                }

                return equal_args;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        if(holds_alternative<ground_literal>(pred2)) {
            ground_literal p1 = std::get<ground_literal>(pred1);
            ground_literal p2 = std::get<ground_literal>(pred2);

            if(p1.predicate == p2.predicate) {
                bool equal_args = true;
                
                int arg_index = 0;
                for(string arg : p1.args) {
                    if(arg != p2.args.at(arg_index)) {
                        equal_args = false;
                        break;
                    }

                    arg_index++;
                }

                return equal_args;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    return false;
}

/*
    Function: check_decomposition_preconditions
    Objective: Check the decomposition predicates given the current world state

    @ Input 1: The world state
    @ Input 2: The world state functions
    @ Input 3: The decomposition being considered
    @ Output: A boolean flag indicating if preconditions held or not.
*/
bool check_decomposition_preconditions(vector<ground_literal> world_state, vector<pair<ground_literal,variant<int,float>>> world_state_functions, Decomposition d) {
    bool preconditions_hold = true;
    for(auto prec : d.prec) { 
        if(holds_alternative<ground_literal>(prec)) {              
            ground_literal p = get<ground_literal>(prec);
            
            if(!p.isComparison) {
                for(ground_literal state : world_state) {
                    bool same_predicate = is_same_predicate(state, p);

                    if(same_predicate) {
                        if(state.positive != p.positive) {
                            preconditions_hold = false;
                            break;
                        }
                    }
                }
            } else {
                for(pair<ground_literal,variant<int,float>> func_state : world_state_functions) {
                    bool same_predicate = is_same_predicate(func_state.first, p);
                    
                    if(same_predicate) {
                        string comparison_op = p.comparison_op_and_value.first;
                        variant<int,float> comparison_value = p.comparison_op_and_value.second;
                            
                        if(comparison_op == equal_comparison_op) {
                            if(holds_alternative<int>(func_state.second)) {
                                int state_val = std::get<int>(func_state.second);

                                if(holds_alternative<int>(comparison_value)) {
                                    preconditions_hold = (state_val == std::get<int>(comparison_value));
                                } else {
                                    preconditions_hold = compare_int_and_float(state_val, std::get<float>(comparison_value));
                                }
                            } else {
                                float state_val = std::get<float>(func_state.second);

                                if(holds_alternative<int>(comparison_value)) {
                                    preconditions_hold = compare_int_and_float(std::get<int>(comparison_value), state_val);
                                } else {
                                    preconditions_hold = compare_floats(state_val, std::get<float>(comparison_value));
                                }
                            }
                            
                            if(!preconditions_hold) break;
                        } else if(comparison_op == greater_comparison_op) {
                            if(holds_alternative<int>(func_state.second)) {
                                int state_val = std::get<int>(func_state.second);

                                if(holds_alternative<int>(comparison_value)) {
                                    preconditions_hold = (state_val > std::get<int>(comparison_value));
                                } else {
                                    preconditions_hold = greater_than_int_and_float(state_val, std::get<float>(comparison_value));
                                }
                            } else {
                                float state_val = std::get<float>(func_state.second);

                                if(holds_alternative<int>(comparison_value)) {
                                    preconditions_hold = greater_than_float_and_int(std::get<int>(comparison_value), state_val);
                                } else {
                                    preconditions_hold = greater_than_floats(state_val, std::get<float>(comparison_value));
                                }
                            }
                            
                            if(!preconditions_hold) break;
                        }
                    }
                }
            }
        } 

        if(!preconditions_hold) {
            break;
        }
    }

    return preconditions_hold;
}

/*
    Function: apply_effects_in_valid_decomposition
    Objective: Apply effects that need to applied to the world state considering which tasks are present in a valid decomposition

    @ Input 1: A reference to the world state
    @ Input 2: A reference to the world state functions
    @ Input 3: The valid decomposition being considered
    @ Input 4: The effects that are to be applied
    @ Output: Void. The world state is updated
*/
void apply_effects_in_valid_decomposition(vector<ground_literal>& world_state, vector<pair<ground_literal,variant<int,float>>>& world_state_functions, pair<vector<pair<int,ATNode>>,set<int>> valid_mission_decomposition, 
                                              map<int,vector<variant<ground_literal,pair<ground_literal,variant<int,float>>>>> effects_to_apply) {
    map<int,vector<variant<ground_literal,pair<ground_literal,variant<int,float>>>>>::iterator eff_it;
    for(eff_it = effects_to_apply.begin(); eff_it != effects_to_apply.end(); ++eff_it) {
        if(valid_mission_decomposition.second.find(eff_it->first) != valid_mission_decomposition.second.end()) {
            for(auto eff : eff_it->second) {
                bool found_predicate = false;
                if(holds_alternative<ground_literal>(eff)) {
                    ground_literal e = std::get<ground_literal>(eff);
                    for(ground_literal& state : world_state) {
                        bool same_predicate = is_same_predicate(state, e);

                        if(same_predicate) {
                            if(e.positive != state.positive) {
                                state.positive = e.positive;
                            }

                            found_predicate = true;
                        }
                    }

                    if(!found_predicate) {
                        world_state.push_back(e);
                    }
                } else {
                    pair<ground_literal,variant<int,float>> e = std::get<pair<ground_literal,variant<int,float>>>(eff);

                    for(pair<ground_literal,variant<int,float>>& f_state : world_state_functions) {
                        bool same_predicate = is_same_predicate(f_state.first, e.first);

                        if(same_predicate) {
                            if(e.first.isAssignCostChange) {
                                f_state.second = e.second;
                            } else {
                                if(holds_alternative<int>(e.second)) {
                                    int eff_value = std::get<int>(e.second);

                                    if(holds_alternative<int>(f_state.second)) {
                                        f_state.second = eff_value + std::get<int>(f_state.second);
                                    } else {
                                        f_state.second = eff_value + std::get<float>(f_state.second);
                                    }
                                } else {
                                    float eff_value = std::get<float>(f_state.second);

                                    if(holds_alternative<int>(f_state.second)) {
                                        f_state.second = eff_value + std::get<int>(f_state.second);
                                    } else {
                                        f_state.second = eff_value + std::get<float>(f_state.second);
                                    }
                                }
                            }

                            found_predicate = true;
                        }
                    }

                    if(!found_predicate) {
                        world_state_functions.push_back(e);
                    }
                }
            }
        }
    }
}