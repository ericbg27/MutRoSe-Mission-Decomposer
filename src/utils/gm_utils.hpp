#ifndef __GM_UTILS
#define __GM_UTILS

#include <string>
#include <vector>
#include <set>
#include <variant>
#include <map>

#include "condition.hpp"

//************************************************* CONSTANTS DECLARATION **********************************************************
const std::string goal_type_prop = "GoalType";
const std::string controls_prop = "Controls";
const std::string monitors_prop = "Monitors";
const std::string context_prop = "CreationCondition";
const std::string location_prop = "Location";
const std::string robot_number_prop = "RobotNumber";
const std::string params_prop = "Params";
const std::string achieve_condition_prop = "AchieveCondition";
const std::string queried_property_prop = "QueriedProperty";
const std::string description_prop = "Description";
const std::string failure_condition_prop = "FailureCondition";
const std::string achieve_goal_type = "Achieve";
const std::string perform_goal_type = "Perform";
const std::string query_goal_type = "Query";
const std::string istar_goal = "istar.Goal";
const std::string istar_task = "istar.Task";
const std::set<std::string> default_props{description_prop, queried_property_prop, failure_condition_prop, achieve_condition_prop};
//**********************************************************************************************************************************

class AchieveCondition : public Condition {
    public:
        bool has_forAll_expr;
        
        std::string get_iterated_var();
        std::string get_iteration_var();
        std::string get_forAll_condition();
        std::string get_condition();

        void set_iterated_var(std::string ivar);
        void set_iteration_var(std::string itvar);
        void set_forAll_condition(std::string f_cond);

        std::variant<std::pair<std::pair<predicate_definition,std::vector<std::string>>,bool>,bool> evaluate_condition(std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map);

    private:
        std::string iterated_var;
        std::string iteration_var;
        std::string forAll_condition;
};

class FailureCondition : public Condition {};

struct IterationRule {
    std::string iterated_var;
    std::string iteration_var;
    std::pair<std::string,std::string> result_var;
    std::string result_init;
    std::string end_loop;
};

struct QueriedProperty {
    std::string queried_var;
    std::pair<std::string,std::string> query_var;
    std::vector<std::string> query;
};

IterationRule parse_iterate_expr(std::string expr);

AchieveCondition parse_achieve_condition(std::string cond);

QueriedProperty parse_select_expr(std::string expr);

std::string get_node_name(std::string node_text);
std::string parse_gm_var_type(std::string var_type);

std::vector<std::pair<std::string,std::string>> parse_vars(std::string var_decl);
std::vector<std::pair<std::string,std::string>> parse_var_mapping(std::string text);

std::vector<std::string> parse_forAll_expr(std::string expr);

std::pair<std::string,std::string> parse_at_text(std::string text);
std::pair<std::string,std::string> parse_goal_text(std::string text);

std::pair<int,int> parse_robot_number(std::string text);

#endif