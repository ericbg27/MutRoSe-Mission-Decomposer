#ifndef __TDG_UTILS
#define __TDG_UTILS

#include <vector>
#include <variant>

#include "domain.hpp"

/*
    -> Fragments to expand consists in:
        - pair<int,int>: The index of the first and last tasks of the fragment
        - literal: The function precondition (which will be a comparison expression and has a comparison value and op = ">")
*/
struct DecompositionPath {
    std::vector<task> decomposition;
    bool needs_expansion = false;
    variant<int,float> expansion_decrease = 0;
    std::vector<std::pair<std::pair<int,int>,literal>> fragments_to_expand;
};

struct DecompositionNode {
    std::variant<task,method> content;
    int id;
    int parent;
    bool is_primitive_task_node;
};

struct CompleteDecompositionPath {
    std::vector<DecompositionNode> decomposition;
};

#endif