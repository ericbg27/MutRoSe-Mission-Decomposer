#include "typechecker.hpp"

#include <set>
#include <variant>
#include <iostream>

using namespace std;

void check_types_in_var_mapping(vector<VariableMapping> variable_mapping, map<string,string> type_mapping, GMGraph gm, vector<task> abstract_tasks) {
    map<string,vector<VariableMapping>> gm_vars;
    map<string,int> task_id_map;

    for(VariableMapping vm : variable_mapping) {
        gm_vars[vm.get_gm_var()].push_back(vm);
        if(task_id_map.find(vm.get_task_id()) == task_id_map.end()) {
            task_id_map[vm.get_task_id()] = find_gm_node_by_id(vm.get_task_id(), gm);
        }
    }

    vector<int> gm_dfs_nodes = get_dfs_gm_nodes(gm);

    for(int v : gm_dfs_nodes) {
        if(gm[v].custom_props.find("Controls") != gm[v].custom_props.end()) {
            vector<pair<string,string>> controlled_vars = std::get<vector<pair<string,string>>>(gm[v].custom_props["Controls"]);
            for(pair<string,string> cvar : controlled_vars) {
                if(gm_vars.find(cvar.first) != gm_vars.end()) {
                    for(VariableMapping vm : gm_vars[cvar.first]) {
                        int task_gm_id = task_id_map[vm.get_task_id()];
                        
                        string at_name = parse_at_text(gm[task_gm_id].text).second;
                        
                        task at;
                        for(task t : abstract_tasks) {
                            if(t.name == at_name) {
                                at = t;
                                break;
                            }
                        }
                        if(at.name == "") {
                            string at_not_found_error = "Could not find AT " + at_name + " from variable mappings in HDDL AT's";

                            throw std::runtime_error(at_not_found_error);
                        }

                        bool found_hddl_var = false;
                        for(pair<string,string> at_var : at.vars) {
                            if(at_var.first == vm.get_hddl_var()) {
                                if(at_var.second != type_mapping[cvar.second]) {
                                    string type_error = "Type error in AT: " + at.name;

                                    throw std::runtime_error(type_error);
                                }

                                found_hddl_var = true;
                                break;
                            }
                        }
                        if(!found_hddl_var) {
                            string hddl_var_not_found_err = "Could not find variable " + vm.get_hddl_var() + " in AT " + at.name + " definition";

                            throw std::runtime_error(hddl_var_not_found_err);
                        }
                    }
                }
            }
        }
    }
}