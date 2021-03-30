#ifndef __AT_MANAGER
#define __AT_MANAGER

#include <vector>
#include <string>
#include <map>
#include <variant>

#include <boost/property_tree/ptree.hpp>

#include "parsetree.hpp"
#include "domain.hpp"
#include "knowledgebase.hpp"
#include "config.hpp"
#include "gm.hpp"
#include "at.hpp"

using namespace std;

namespace pt = boost::property_tree;

map<string,vector<AbstractTask>> generate_at_instances(vector<task> abstract_tasks , GMGraph gm, string location_type, 
                                                        KnowledgeBase world_db, map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map,
                                                            vector<VariableMapping> var_mapping);

void print_at_instances_info(map<string,vector<AbstractTask>> at_instances);
void print_at_paths_info(map<string,vector<vector<task>>> at_decomposition_paths);

bool check_path_validity(vector<task> path, vector<ground_literal> world_state, AbstractTask at, vector<SemanticMapping> semantic_mappings);

#endif