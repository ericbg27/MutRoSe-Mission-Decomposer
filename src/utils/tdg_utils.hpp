#ifndef __TDG_UTILS
#define __TDG_UTILS

#include <vector>

#include "domain.hpp"

/*
    -> Fragments to expand consists in:
        - pair<int,int>: The index of the first and last tasks of the fragment
        - literal: The function precondition (which will be a comparison expression and has a comparison value and op = ">")
*/
struct DecompositionPath {
    std::vector<task> decomposition;
    bool needs_expansion = false;
    std::vector<std::pair<std::pair<int,int>,literal>> fragments_to_expand;
};

#endif