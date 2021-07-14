#include "predicate_utils.hpp"

#include <iostream>

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

bool is_same_non_ground_predicate(pair<literal,vector<string>> pred1, pair<literal,vector<string>> pred2) {
    literal p1 = pred1.first;
    literal p2 = pred2.first;

    vector<string> p1_args = pred1.second;
    vector<string> p2_args = pred2.second;

    if(p1.predicate == p2.predicate) {
        bool same_non_ground_predicate = true;

        if(p1_args.size() == p2_args.size()) {
            int arg_index = 0;
            for(string arg1_type : p1_args) {
                if(arg1_type != p2_args.at(arg_index)) {
                    same_non_ground_predicate = false;

                    break;
                }      

                arg_index++;
            }
        } else {
            return false;
        }

        return same_non_ground_predicate;
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
bool check_decomposition_preconditions(vector<ground_literal> world_state, vector<pair<ground_literal,variant<int,float>>> world_state_functions, map<int,vector<pair<literal,vector<string>>>> non_ground_world_state, pair<int,Decomposition> d_info, vector<Constraint> mission_constraints, set<int> current_decomposition, set<string> robot_related_sorts) {
    bool preconditions_hold = true;

    int d_id = d_info.first;
    Decomposition d = d_info.second;

    /*
        Here we must check for group tasks and, if any, check if they are sequential or not
            - If sequential, preconditions for the second task will involve effects on the first one
            - If parallel or fallback, how do we deal with this?
    */
    set<int> robot_related_precs;
    
    vector<pair<string,string>> d_vars;
    for(int var_index = 0; var_index < d.at.at.number_of_original_vars; var_index++) {
        d_vars.push_back(d.at.at.vars.at(var_index));
    }

    int prec_index = 0;
    for(auto prec : d.prec) { //NEED TO ADD THE SORTS VECTOR TO THE VALID MISSION GENERATOR AND AS INPUT OF THE FUNCTION!
        if(holds_alternative<literal>(prec)) {
            literal p = std::get<literal>(prec);

            bool robot_related = true;
            for(string arg : p.arguments) {
                for(pair<string,string> var : d_vars) {
                    if(var.first == arg) {
                        if(var.second != hddl_robot_type && var.second != hddl_robotteam_type && robot_related_sorts.find(var.second) == robot_related_sorts.end()) {
                            robot_related = false;

                            break;
                        }
                    }
                }
            }

            if(robot_related) {
                robot_related_precs.insert(prec_index);
            }
        }

        prec_index++;
    }
    // Here we need to check in the predicate definition (which we currently don't have) if all of the arguments are of robot or robotteam type

    vector<pair<int,pair<literal,vector<string>>>> ws_ng_aux;
    vector<literal> ws_ng;
    
    if(robot_related_precs.size() > 0) {
        vector<int> task_ordering;
        map<int,set<int>> execution_constrained_tasks;
        map<int,map<int,map<string,string>>> var_maps;

        for(Constraint c : mission_constraints) {
            if(c.type == SEQ) {
                vector<int>::iterator t_it = std::find(task_ordering.begin(),task_ordering.end(),c.nodes_involved.second.first);

                if(t_it != task_ordering.end()) {
                    task_ordering.insert(t_it, c.nodes_involved.first.first);
                } else {
                    task_ordering.push_back(c.nodes_involved.first.first);
                    task_ordering.push_back(c.nodes_involved.second.first);
                }
            } else if(c.type == NC) {
                //map variables of execution constrained tasks
                task t1 = std::get<Decomposition>(c.nodes_involved.first.second.content).at.at;
                task t2 = std::get<Decomposition>(c.nodes_involved.second.second.content).at.at;

                map<string,string> var_map1, var_map2;
                
                int arg_index2 = 0;
                for(int arg_index1 = 0; arg_index1 < t1.number_of_original_vars; arg_index1++) {
                    string t1_var_type = t1.vars.at(arg_index1).second;
                    if(t1_var_type == hddl_robot_type || t1_var_type == hddl_robotteam_type || robot_related_sorts.find(t1_var_type) != robot_related_sorts.end()) {
                        bool found_arg = false;
                        while(!found_arg || arg_index2 < t2.number_of_original_vars) {
                            if(t2.vars.at(arg_index2).second == t1_var_type) {
                                var_map1[t1.vars.at(arg_index1).first] = t2.vars.at(arg_index2).first;
                                var_map2[t2.vars.at(arg_index2).first] = t1.vars.at(arg_index1).first;
                                
                                found_arg = true;
                            }

                            arg_index2++;
                        }
                    }
                }

                var_maps[c.nodes_involved.first.first][c.nodes_involved.second.first] = var_map1;
                var_maps[c.nodes_involved.second.first][c.nodes_involved.first.first] = var_map2;

                execution_constrained_tasks[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);
                execution_constrained_tasks[c.nodes_involved.second.first].insert(c.nodes_involved.first.first);
            }
        }

        for(int t : task_ordering) {
            if(t == d_id) {
                break;
            }

            if(current_decomposition.find(t) != current_decomposition.end()) {
                if(execution_constrained_tasks[d_id].find(t) != execution_constrained_tasks[d_id].end()) {
                    for(pair<literal,vector<string>> eff : non_ground_world_state[t]) {
                        bool found_predicate = false;

                        for(pair<int,pair<literal,vector<string>>>& state : ws_ng_aux) {
                            //bool same_predicate = is_same_non_ground_predicate(eff, state); // THIS MUST CHANGE!
                            bool same_predicate = true;
                            if(eff.first.predicate == state.second.first.predicate) {
                                int arg_index = 0;
                                for(string arg : eff.first.arguments) {
                                    string map_var = var_maps[t][state.first][arg];

                                    if(state.second.first.arguments.at(arg_index) != map_var) {
                                        same_predicate = false;
                                    }
                                }
                            } else {
                                same_predicate = false;
                            }

                            if(same_predicate) {
                                if(eff.first.positive != state.second.first.positive) {
                                    state.second.first.positive = eff.first.positive;
                                }

                                found_predicate = true;
                                break;
                            }
                        }

                        if(!found_predicate) {
                            ws_ng_aux.push_back(make_pair(t,eff));
                        }
                    }
                }
            }
        }

        for(pair<int,pair<literal,vector<string>>> state : ws_ng_aux) {
            literal s = state.second.first;

            for(string& arg : s.arguments) {
                arg = var_maps[state.first][d_id][arg];
            }

            ws_ng.push_back(s);
        }
    }

    prec_index = 0;
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
        } else {
            literal p = std::get<literal>(prec);

            if(robot_related_precs.find(prec_index) != robot_related_precs.end()) {
                for(literal s : ws_ng) {
                    if(is_same_predicate(p,s)) {
                        if(s.positive != p.positive) {
                            preconditions_hold = false;
                        }

                        break;
                    }
                }
            }
        }

        if(!preconditions_hold) {
            break;
        }

        prec_index++;
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
void apply_effects_in_valid_decomposition(vector<ground_literal>& world_state, vector<pair<ground_literal,variant<int,float>>>& world_state_functions, map<int,vector<pair<literal,vector<string>>>>& non_ground_world_state, pair<vector<pair<int,ATNode>>,set<int>> valid_mission_decomposition,
                                            map<int,pair<vector<variant<ground_literal,pair<ground_literal,variant<int,float>>>>,vector<pair<literal,vector<string>>>>> effects_to_apply) {
    map<int,pair<vector<variant<ground_literal,pair<ground_literal,variant<int,float>>>>,vector<pair<literal,vector<string>>>>>::iterator eff_it;
    for(eff_it = effects_to_apply.begin(); eff_it != effects_to_apply.end(); ++eff_it) {
        if(valid_mission_decomposition.second.find(eff_it->first) != valid_mission_decomposition.second.end()) {
            for(auto eff : eff_it->second.first) {
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

            for(pair<literal,vector<string>> eff : eff_it->second.second) {
                bool found_predicate = false;
                for(pair<literal,vector<string>>& state : non_ground_world_state[eff_it->first]) {
                    bool same_predicate = is_same_non_ground_predicate(eff, state);

                    if(same_predicate) {
                        if(eff.first.positive != state.first.positive) {
                            state.first.positive = eff.first.positive;
                        }

                        found_predicate = true;
                        break;
                    }
                }

                if(!found_predicate) {
                    non_ground_world_state[eff_it->first].push_back(eff);
                }
            }
        }
    }
}

vector<string> get_predicate_argument_types(task t, literal pred) {
    vector<string> arg_types;

    for(string pred_arg : pred.arguments) {
        int arg_index = 0;
        while(arg_index < t.number_of_original_vars) {
            pair<string,string> var_def = t.vars.at(arg_index);

            if(var_def.first == pred_arg) {
                arg_types.push_back(var_def.second);

                break;
            }
            
            arg_index++;
        }
    }

    return arg_types;
}