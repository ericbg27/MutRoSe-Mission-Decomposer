#ifndef __KNOWLEDGEMANAGER
#define __KNOWLEDGEMANAGER

#include <map>
#include <set>
#include <string>
#include <vector>
#include <variant>

#include <boost/property_tree/ptree.hpp>

#include "at.hpp"
#include "config.hpp"
#include "knowledgebase.hpp"

namespace pt = boost::property_tree;

KnowledgeBase construct_knowledge_base(std::string name, std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg);

void initialize_objects(KnowledgeBase worlddb, KnowledgeBase robotsdb, std::map<std::string,std::set<std::string>>& sorts, std::vector<std::string> high_level_loc_types,
                            std::map<std::string,std::vector<AbstractTask>>& at_instances, std::map<std::string,std::string> type_mapping);

struct Robot {
    std::string name;
    std::string type;
    std::string pos;
};

void initialize_world_state(KnowledgeBase robotsdb, KnowledgeBase worlddb, std::vector<ground_literal>& init, std::vector<pair<ground_literal,int>>& init_functions, 
                                std::vector<SemanticMapping> semantic_mapping, std::map<std::string,std::string> type_mapping, std::map<std::string,std::set<std::string>> sorts);

void print_world_state(std::vector<ground_literal> world_state);

inline bool operator==(const Robot& r1, const Robot& r2) {
    return (r1.name == r2.name);
}

extern std::vector<Robot> assigned_robots;

#endif