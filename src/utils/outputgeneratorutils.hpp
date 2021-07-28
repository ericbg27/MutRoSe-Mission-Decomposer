#ifndef __OUTPUT_GENERATOR_UTILS
#define __OUTPUT_GENERATOR_UTILS

#include <string>
#include <vector>
#include <variant>
#include <map>
#include <set>

#include "../missiondecomposer/missiondecomposer.hpp"
#include "../constraintmanager/constraintmanager.hpp"

std::pair<SemanticMapping,bool> find_predicate_mapping(std::variant<ground_literal,literal> predicate, std::vector<SemanticMapping> semantic_mappings, std::map<std::string,std::set<std::string>> sorts,
                                                std::map<std::string,std::string> vars, std::vector<sort_definition> sort_definitions);
#endif