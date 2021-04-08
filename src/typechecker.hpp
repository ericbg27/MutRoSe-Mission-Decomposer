#ifndef __TYPECHECKER
#define __TYPECHECKER

#include <vector>
#include <map>
#include <string>

#include "config.hpp"
#include "gm.hpp"
#include "domain.hpp"

void check_types_in_var_mapping(std::vector<VariableMapping> variable_mapping, std::map<std::string,std::string> type_mapping, GMGraph gm, std::vector<task> abstract_tasks);

#endif