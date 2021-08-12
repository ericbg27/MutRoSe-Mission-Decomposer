#ifndef __CONDITION
#define __CONDITION

#include <string>
#include <map>
#include <vector>

#include "parsetree.hpp"
#include "../config/config.hpp"
#include "atgraph.hpp"

const std::string ocl_equal = "=";
const std::string ocl_different = "<>";
const std::string ocl_in = "in";
const std::string spaced_ocl_in = " in ";
const std::string ocl_gt = ">";
const std::string ocl_lt = "<";
const std::string ocl_geq = ">=";
const std::string ocl_leq = "<=";

using func_pred_vector = vector<std::pair<ground_literal,std::variant<std::pair<bool,std::variant<int,float>>,std::pair<std::string,std::variant<int,float>>>>>;
using pred_vector = vector<ground_literal>;

struct ConditionExpression {
    ConditionExpression();
    ConditionExpression(std::variant<pred_vector,func_pred_vector,ConditionExpression*> le, std::variant<pred_vector,func_pred_vector,ConditionExpression*> re, bool is_and);

    bool evaluate_expression(std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions);
    bool is_empty_expression();

    ConditionExpression* check_non_active_predicates(std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions, std::vector<SemanticMapping> semantic_mapping);

    std::variant<pred_vector,func_pred_vector,ConditionExpression*> left_expr;
    std::variant<pred_vector,func_pred_vector,ConditionExpression*> right_expr;

    bool is_and;
};

class ConditionEvaluation {
    public:
        ConditionExpression* get_evaluation_predicates();

        void set_evaluation(std::variant<std::pair<std::pair<predicate_definition,std::vector<std::string>>,std::variant<std::pair<std::variant<int,float>,std::variant<bool,std::string>>,bool>>,bool,pair<ConditionEvaluation*,ConditionEvaluation*>> eval);
        void set_is_and(bool a);

        std::variant<std::pair<std::pair<predicate_definition,std::vector<std::string>>,std::variant<std::pair<std::variant<int,float>,std::variant<bool,std::string>>,bool>>,bool,pair<ConditionEvaluation*,ConditionEvaluation*>> get_evaluation();
        bool get_is_and();

    private:
        bool is_and;
        std::variant<std::pair<std::pair<predicate_definition,std::vector<std::string>>,std::variant<std::pair<std::variant<int,float>,std::variant<bool,std::string>>,bool>>,bool,pair<ConditionEvaluation*,ConditionEvaluation*>> evaluation;
};

class Condition {
    public:
        std::variant<std::string,std::pair<Condition*,Condition*>> get_condition();
        bool get_is_and();

        void set_condition(std::variant<std::string,std::pair<Condition*,Condition*>> cond);
        void set_is_and(bool a);

        ConditionEvaluation* evaluate_condition(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> var_maps, std::vector<SemanticMapping> semantic_mapping, std::set<std::string> accepted_regex_patterns);

        std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>> get_var_value_and_type(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map, std::string variable);
    
    protected:
        std::variant<std::string,std::pair<Condition*,Condition*>> condition;
        bool is_and;
};

#endif