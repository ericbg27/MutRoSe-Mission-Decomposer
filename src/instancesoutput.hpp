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
#include "constraintmanager.hpp"
#include "gm.hpp"

using namespace std;

void generate_instances_output(ATGraph mission_decomposition, GMGraph gm, pair<string,string> output, vector<ground_literal> world_state, vector<SemanticMapping> semantic_mapping,
                                map<string,set<string>> sorts, vector<sort_definition> sort_definitions, vector<predicate_definition> predicate_definitions);

void output_actions(pt::ptree& output_file, map<string,task> actions);
std::map<std::string,std::string> output_tasks(pt::ptree& output_file, vector<Decomposition> task_instances, vector<SemanticMapping> semantic_mapping);
void output_constraints(pt::ptree& output_file, std::vector<Constraint> final_mission_constraints, std::map<std::string,std::string> task_id_map);
void output_mission_decompositions(pt::ptree& output_file, std::vector<std::vector<std::pair<int,ATNode>>> valid_mission_decompositions, std::map<std::string,std::string> task_id_map);

vector<vector<pair<int,ATNode>>> generate_valid_mission_decompositions(ATGraph mission_decomposition, vector<Constraint> mission_constraints, vector<ground_literal> world_state);

void recursive_valid_mission_decomposition(ATGraph mission_decomposition, vector<ground_literal> initial_world_state, vector<Constraint> mission_constraints, string last_op,
                                            queue<pair<int,ATNode>>& mission_queue, vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>>& valid_mission_decompositions,
                                                vector<pair<int,ATNode>>& possible_conflicts);

queue<pair<int,ATNode>> generate_mission_queue(ATGraph mission_decomposition);

pair<SemanticMapping,bool> find_predicate_mapping(variant<ground_literal,literal> predicate, vector<SemanticMapping> semantic_mappings, map<string,set<string>> sorts,
                                                map<string,string> vars, vector<sort_definition> sort_definitions);

void resolve_conflicts(vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>>& valid_mission_decompositions, vector<pair<int,ATNode>> possible_conflicts, ATGraph mission_decomposition,
                        vector<Constraint> mission_constraints);

#endif