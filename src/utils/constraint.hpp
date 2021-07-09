#ifndef __CONSTRAINT
#define __CONSTRAINT

#include "../missiondecomposer/missiondecomposer.hpp"

enum constraint_type {SEQ,PAR,NC,NEX,FB};

// Constraint involving two different abstract tasks/decompositions
struct Constraint {
    constraint_type type;
    std::pair<std::pair<int,ATNode>,std::pair<int,ATNode>> nodes_involved;
    bool group = true;
    bool divisible = true;
};

#endif