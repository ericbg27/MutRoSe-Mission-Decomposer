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

map<string,vector<AbstractTask>> generate_at_instances(vector<task> abstract_tasks , GMGraph gm, pt::ptree worlddb, string location_type, 
                                                        KnowledgeBase world_db, map<string, variant<pair<string,string>,pair<vector<string>,string>>>& gm_var_map,
                                                            vector<VariableMapping> var_mapping);

#endif