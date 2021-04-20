#ifndef __CONSTRAINT_MANAGER
#define __CONSTRAINT_MANAGER

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <stack>
#include <queue>
#include <set>
#include <iostream>

#include "../missiondecomposer/missiondecomposer.hpp"
#include "../gm/gm.hpp"

enum constraint_type {SEQ,PAR,NC,NEX};

// Constraint involving two different abstract tasks/decompositions
struct Constraint {
    constraint_type type;
    std::pair<std::pair<int,ATNode>,std::pair<int,ATNode>> nodes_involved;
    bool group = true;
    bool divisible = true;
};

std::vector<Constraint> generate_at_constraints(ATGraph trimmed_mission_decomposition);
std::vector<Constraint> transform_at_constraints(ATGraph mission_decomposition, std::vector<Constraint> mission_constraints, GMGraph gm);

void generate_constraints_from_stacks(std::stack<std::pair<int,ATNode>>& operators_stack, std::stack<std::variant<std::pair<int,ATNode>,Constraint>>& nodes_stack, std::map<int,std::set<int>>& existing_constraints);
void generate_noncoop_constraints(std::vector<Constraint>& mission_constraints, ATGraph mission_decomposition);

Constraint generate_constraint(std::pair<int,ATNode> n1, std::pair<int,ATNode> n2, constraint_type type);

#endif