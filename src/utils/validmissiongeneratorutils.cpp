#include "validmissiongeneratorutils.hpp"

#include <iostream>

using namespace std;

void expand_decomposition(Decomposition& d, vector<pair<ground_literal,variant<int,float>>> world_state_func, bool verbose) {
    if(d.path.needs_expansion) {
        vector<pair<vector<task>,int>> expansions_to_insert;

        for(auto fragment : d.path.fragments_to_expand) {
            // Check value of function predicate
            ground_literal pred;

            pred.predicate = fragment.second.predicate;

            vector<string> arguments;
            for(string arg : fragment.second.arguments) {
                for(auto var_map : d.at.variable_mapping) {
                    if(var_map.second == arg) {
                        if(holds_alternative<string>(var_map.first.first)) {
                            arguments.push_back(std::get<string>(var_map.first.first));
                        } else {
                            string collection_variable_in_function_predicate_error = "Function predicate in task " + d.at.id + " cannot contain collection-related variables";

                            throw std::runtime_error(collection_variable_in_function_predicate_error);
                        }
                    }
                }
            }

            pred.args = arguments;
            
            variant<int,float> pred_value;
            bool found_pred = false;
            for(pair<ground_literal,variant<int,float>> state : world_state_func) {
                bool same_predicate = is_same_predicate(state.first, pred);
                
                if(same_predicate) {
                    pred_value = state.second;
                    found_pred = true;

                    break;
                }
            }

            if(!found_pred) {
                string expansion_failed_error = "Failed to expand decomposition [" + d.id + "] of task [" + d.at.name + "]";

                throw std::runtime_error(expansion_failed_error);
            }
            
            int expansion_number;

            if(holds_alternative<int>(pred_value)) {
                int p_val = std::get<int>(pred_value);

                if(holds_alternative<int>(fragment.second.comparison_op_and_value.second)) {
                    expansion_number = p_val - std::get<int>(fragment.second.comparison_op_and_value.second) - 1;
                } else {
                    expansion_number = p_val - (static_cast<int>(std::get<float>(fragment.second.comparison_op_and_value.second)) + 1) - 1;
                }
            } else {
                float p_val = std::get<float>(pred_value);

                if(holds_alternative<int>(fragment.second.comparison_op_and_value.second)) {
                    expansion_number = (static_cast<int>(p_val)+1) - std::get<int>(fragment.second.comparison_op_and_value.second) - 1;
                } else {
                    expansion_number = (static_cast<int>(p_val - std::get<float>(fragment.second.comparison_op_and_value.second)) + 1) - 1;
                }
            }

            pair<int,int> task_indexes = fragment.first;

            vector<task>::const_iterator begin = d.path.decomposition.begin() + task_indexes.first;
            vector<task>::const_iterator end = d.path.decomposition.begin() + task_indexes.second + 1;
            vector<task> sub_path(begin,end);

            vector<task> expansion_result;
            for(int expansion_index = 0; expansion_index < expansion_number; expansion_index++) { //Expand tasks with indexes present in fragments
                expansion_result.insert(expansion_result.end(), sub_path.begin(), sub_path.end());
            }

            int index_to_insert = task_indexes.second+1;
            for(pair<vector<task>,int> expansion : expansions_to_insert) {
                index_to_insert += expansion.first.size();
            }
            expansions_to_insert.push_back(make_pair(expansion_result,index_to_insert));
        }

        for(pair<vector<task>,int> expansion : expansions_to_insert) {
            d.path.decomposition.insert(d.path.decomposition.begin()+expansion.second, expansion.first.begin(), expansion.first.end());
        }

        instantiate_decomposition_predicates(d.at, d, verbose);
    }
}