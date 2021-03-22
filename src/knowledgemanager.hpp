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

using namespace std;

namespace pt = boost::property_tree;

KnowledgeBase construct_knowledge_base(string name, map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> cfg);

void initialize_objects(KnowledgeBase worlddb, KnowledgeBase robotsdb, map<string,set<string>>& sorts, string location_type,
                            map<string,vector<AbstractTask>>& at_instances, map<string,string> type_mapping);

struct Robot {
    string name;
    string type;
    string pos;
};

void initialize_world_state(KnowledgeBase robotsdb, KnowledgeBase worlddb, vector<ground_literal>& init, vector<pair<ground_literal,int>>& init_functions, 
                                vector<SemanticMapping> semantic_mapping, map<string,string> type_mapping, map<string,set<string>> sorts);

void print_world_state(vector<ground_literal> world_state);

inline bool operator==(const Robot& r1, const Robot& r2) {
    return (r1.name == r2.name);
}

extern vector<Robot> assigned_robots;

#endif