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

#include "missiondecomposer.hpp"
#include "gm.hpp"

enum constraint_type {SEQ,PAR,NC,NEX};

// Constraint involving two different abstract tasks/decompositions
struct Constraint {
    constraint_type type;
    std::pair<std::pair<int,ATNode>,std::pair<int,ATNode>> nodes_involved;
    bool group = true;
    bool divisible = true;
};

//vector<Constraint> generate_at_constraints(ATGraph mission_decomposition, queue<pair<int,ATNode>> mission_queue);
vector<Constraint> generate_at_constraints(ATGraph trimmed_mission_decomposition);
vector<Constraint> transform_at_constraints(ATGraph mission_decomposition, vector<Constraint> mission_constraints, GMGraph gm);

void generate_constraints_from_stacks(stack<pair<int,ATNode>>& operators_stack, stack<variant<pair<int,ATNode>,Constraint>>& nodes_stack, map<int,set<int>>& existing_constraints);
void generate_noncoop_constraints(vector<Constraint>& mission_constraints, ATGraph mission_decomposition);

#endif