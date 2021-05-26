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
#include "../utils/constraint.hpp"

std::vector<Constraint> generate_at_constraints(ATGraph trimmed_mission_decomposition);
std::vector<Constraint> transform_at_constraints(ATGraph mission_decomposition, std::vector<Constraint> mission_constraints, GMGraph gm, bool verbose);

void generate_constraints_from_stacks(std::stack<std::pair<int,ATNode>>& operators_stack, std::stack<std::variant<std::pair<int,ATNode>,Constraint>>& nodes_stack, std::map<int,std::set<int>>& existing_constraints);
void generate_execution_constraints(std::vector<Constraint>& mission_constraints, ATGraph mission_decomposition, bool verbose);

Constraint generate_constraint(std::pair<int,ATNode> n1, std::pair<int,ATNode> n2, constraint_type type);

#endif