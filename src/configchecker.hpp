#ifndef __CONFIGCHECKER
#define __CONFIGCHECKER

#include <vector>
#include <map>
#include <string>

#include "config.hpp"
#include "gm.hpp"
#include "domain.hpp"

void check_config(std::vector<VariableMapping> variable_mapping, std::map<std::string,std::string> type_mapping, GMGraph gm, std::vector<task> abstract_tasks,
                    std::vector<SemanticMapping> semantic_mapping, std::vector<std::string> high_level_loc_types, std::vector<predicate_definition> predicate_definitions);

void check_types_in_var_mapping(std::map<std::string,std::vector<VariableMapping>> gm_vars, std::map<std::string,int> task_id_map, std::map<std::string,std::string> type_mapping, 
                                    GMGraph gm, std::vector<task> abstract_tasks, std::set<std::string>& ocl_types);

void check_high_level_loc_types(std::vector<std::string> high_level_loc_types, std::set<std::string> ocl_types);

void check_semantic_mapping(std::vector<SemanticMapping> semantic_mapping, std::vector<predicate_definition> predicate_definitions, std::map<std::string,std::string> type_mapping, std::set<std::string> ocl_types);

#endif