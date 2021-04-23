#ifndef __CONTEXT_MANAGER
#define __CONTEXT_MANAGER

#include <vector>
#include <map>
#include <string>
#include <variant>

#include "../config/config.hpp"
#include "../utils/condition.hpp"

class Context : public Condition {
    public:
        std::string get_context_type();

        void set_context_type(std::string tp);

    private:
        std::string type;
};

bool check_context(Context context, std::vector<ground_literal> world_state, std::vector<SemanticMapping> semantic_mapping, 
                        std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars);

std::pair<bool,std::pair<std::string,predicate_definition>> get_pred_from_context(Context context, std::vector<SemanticMapping> semantic_mapping);
#endif