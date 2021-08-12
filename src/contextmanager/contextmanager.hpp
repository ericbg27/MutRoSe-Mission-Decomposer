#ifndef __CONTEXT_MANAGER
#define __CONTEXT_MANAGER

#include <vector>
#include <map>
#include <string>
#include <variant>

#include "../config/config.hpp"
#include "../utils/condition.hpp"

const std::string trigger_context_type = "trigger";
const std::string condition_context_type = "condition";

class Context : public Condition {
    public:
        std::string get_context_type();

        void set_context_type(std::string tp);

        bool check_context(std::vector<ground_literal> world_state, std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> var_maps);

        ConditionExpression* get_inactive_predicates(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> var_maps, pred_vector world_state, std::vector<SemanticMapping> semantic_mapping);

    private:
        std::string type;
};

std::pair<bool,std::pair<std::string,predicate_definition>> get_pred_from_context(Context context, std::vector<SemanticMapping> semantic_mapping);

#endif