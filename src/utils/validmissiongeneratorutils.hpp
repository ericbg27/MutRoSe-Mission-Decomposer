#ifndef __VALID_MISSION_GENERATOR_UTILS
#define __VALID_MISSION_GENERATOR_UTILS

#include <string>
#include <map>
#include <vector>
#include <variant>

#include "mission_decomposer_utils.hpp"
#include "parsetree.hpp"
#include "domain.hpp"
#include "predicate_utils.hpp"

void expand_decomposition(Decomposition& d, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_func, bool verbose);

#endif