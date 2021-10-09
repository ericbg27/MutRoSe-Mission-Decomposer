#ifndef __IHTN_GENERATOR_UTILS
#define __IHTN_GENERATOR_UTILS

#include <map>
#include <string>
#include <vector>

#include "atgraph.hpp"
#include "tdg_utils.hpp"

std::map<std::string,CompleteDecompositionPath> map_complete_decompositions(ATGraph mission_decomposition, std::map<std::string,std::vector<CompleteDecompositionPath>> at_complete_decomposition_paths);

#endif