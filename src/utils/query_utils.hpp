#ifndef __QUERY_UTILS
#define __QUERY_UTILS

#include <string>
#include <vector>
#include <map>
#include <variant>

#include <boost/property_tree/ptree.hpp>

#include "../gm/gm.hpp"

pt::ptree get_query_ptree(GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>> valid_variables, map<int,AchieveCondition> valid_forAll_conditions, pt::ptree world_tree, std::string knowledge_unique_id);

std::pair<std::vector<pt::ptree>,std::set<std::string>> solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, std::map<std::string,std::pair<std::string,std::vector<pt::ptree>>>& valid_variables, 
																				std::string knowledge_unique_id);

#endif