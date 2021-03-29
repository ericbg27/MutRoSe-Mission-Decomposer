#ifndef __CONTEXT_MANAGER
#define __CONTEXT_MANAGER

#include <vector>
#include <map>
#include <string>
#include <variant>

#include "config.hpp"

struct Context {
    string type;
    string condition;
};

bool check_context(Context context, std::vector<ground_literal> world_state, std::vector<SemanticMapping> semantic_mapping, 
                        std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars);

pair<bool,pair<string,predicate_definition>> get_pred_from_context(Context context, vector<SemanticMapping> semantic_mapping);
#endif