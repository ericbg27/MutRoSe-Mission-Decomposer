#ifndef __INSTANCESOUTPUT
#define __INSTANCESOUTPUT

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <stack>
#include <queue>
#include <set>

#include <boost/graph/adjacency_list.hpp>

#include "missiondecomposer.hpp"
#include "gm.hpp"

using namespace std;

enum constraint_type {SEQ,PAR,NC};

struct Constraint {
    /*
        Constraint involving two different abstract tasks/decompositions
    */
    constraint_type type;
    pair<pair<int,ATNode>,pair<int,ATNode>> nodes_involved;
    bool group = true;
    bool divisible = true;
};

void generate_instances_output(ATGraph mission_decomposition, GMGraph gm, pair<string,string> output, vector<ground_literal> world_state, vector<SemanticMapping> semantic_mapping,
                                map<string,set<string>> sorts, vector<sort_definition> sort_definitions, vector<predicate_definition> predicate_definitions);

vector<vector<pair<int,ATNode>>> generate_valid_mission_decompositions(ATGraph mission_decomposition, vector<Constraint> mission_constraints, vector<ground_literal> world_state);

void recursive_valid_mission_decomposition(ATGraph mission_decomposition, vector<ground_literal> initial_world_state, vector<Constraint> mission_constraints, string last_op,
                                            queue<pair<int,ATNode>>& mission_queue, vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>>& valid_mission_decompositions,
                                                vector<pair<int,ATNode>>& possible_conflicts);

queue<pair<int,ATNode>> generate_mission_queue(ATGraph mission_decomposition);

vector<Constraint> generate_at_constraints(ATGraph mission_decomposition);

vector<Constraint> transform_at_constraints(ATGraph mission_decomposition, vector<Constraint> mission_constraints, GMGraph gm);

void generate_noncoop_constraints(vector<Constraint>& mission_constraints, ATGraph mission_decomposition);

pair<SemanticMapping,bool> find_predicate_mapping(variant<ground_literal,literal> predicate, vector<SemanticMapping> semantic_mappings, map<string,set<string>> sorts,
                                                    map<string,string> vars, vector<sort_definition> sort_definitions);

#endif