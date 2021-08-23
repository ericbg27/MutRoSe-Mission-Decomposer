#ifndef __MISSION_DECOMPOSER_UTILS
#define __MISSION_DECOMPOSER_UTILS

#include <string>
#include <vector>
#include <variant>

#include "predicate_utils.hpp"
#include "atgraph.hpp"
#include "../contextmanager/contextmanager.hpp"
#include "../annotmanager/annotmanager.hpp"

std::pair<ATGraph,std::map<int,int>> generate_trimmed_at_graph(ATGraph mission_decomposition);
ATGraph generate_tree_like_at_graph(ATGraph mission_decomposition);

void instantiate_decomposition_predicates(AbstractTask at, Decomposition& d, bool verbose);

std::vector<std::pair<int,ATNode>> find_decompositions(ATGraph mission_decomposition, int node_id);

void find_non_coop_task_ids(ATGraph mission_decomposition, int node_id, set<int>& task_ids);

bool can_unite_decompositions(Decomposition d1, Decomposition d2, bool non_coop_nodes);

void print_mission_decomposition(ATGraph mission_decomposition);

bool is_unique_branch(ATGraph mission_decomposition);

#endif