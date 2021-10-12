#ifndef __GM_UTILS
#define __GM_UTILS

#include <string>
#include <vector>
#include <set>
#include <variant>
#include <map>

#include "condition.hpp"
#include "query.hpp"
#include "../contextmanager/contextmanager.hpp"

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
const std::string group_prop = "Group";
const std::string divisible_prop = "Divisible";
//const std::string failure_condition_prop = "FailureCondition";
const std::string achieve_goal_type = "Achieve";
const std::string perform_goal_type = "Perform";
const std::string query_goal_type = "Query";
const std::string loop_goal_type = "Loop";
const std::string istar_goal = "istar.Goal";
const std::string istar_task = "istar.Task";
const std::string istar_and = "istar.AndRefinementLink";
const std::string istar_or = "istar.OrRefinementLink";
const std::set<std::string> default_props{description_prop, queried_property_prop, achieve_condition_prop};

const string world_db_query_var = "world_db";

const std::string hddl_robot_type = "robot";
const std::string hddl_robotteam_type = "robotteam";

const std::string select_regex_exp = "[a-zA-Z]{1}[\\w.]*(->select)[(][a-zA-Z]{1}[\\w.]*[:][a-zA-Z]+[\\w.]+[ ]*[|][ ]*(.*)[)]";
const std::string end_select_regex_exp = "[)]";
const std::string forall_regex_exp = "[a-zA-Z]+[\\w.]*(->forAll)[(][a-zA-Z]+[\\w.]*[ ]*[|][ ]*";
const std::string end_forall_regex_exp = "[)]";

const std::string var_attr_condition = "[!]?[a-zA-Z]+[\\w.]*";
const std::string equal_diff_condition = "[a-zA-Z]+[\\w.]*[ ]+((=)|(<>)){1}[ ]+([a-zA-Z]+[a-zA-Z0-9]+|\"[a-zA-Z]+[a-zA-Z0-9]+\"|([\\d]*[.])?[\\d]+)";
const std::string equal_diff_number_condition = "[A-Za-z]+[\\w]*[.][A-za-z]+[A-za-z_]*([ ]+((=)|(<>)){1}[ ]+([0-9]*[.])?[0-9]+)";
const std::string comparison_condition = "[a-zA-Z]+[\\w.]*[ ]+((>)|(<)|(>=)|(<=)){1}[ ]+([\\d]*[.])?[\\d]+";
const std::string in_condition = "[a-zA-Z]+[\\w.]*[ ]+(in)[ ]+[a-zA-Z]+[\\w.]*";
const std::string blank_condition = "";
//**********************************************************************************************************************************

class AchieveCondition : public Condition {
    public:
        bool has_forAll_expr;
        
        std::string get_iterated_var();
        std::string get_iteration_var();

        void set_iterated_var(std::string ivar);
        void set_iteration_var(std::string itvar);

        ConditionEvaluation* evaluate_condition(std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map);

    private:
        std::string iterated_var;
        std::string iteration_var;
};

class FailureCondition : public Condition {}; // TODO

struct QueriedProperty {
    std::string queried_var;
    std::pair<std::string,std::string> query_var;
    Query* query;
};

AchieveCondition parse_achieve_condition(std::string cond);

QueriedProperty parse_select_expr(std::string expr);

Context parse_context_condition(std::string condition);

std::string get_node_name(std::string node_text);
std::string parse_gm_var_type(std::string var_type);

std::vector<std::pair<std::string,std::string>> parse_vars(std::string var_decl);
std::vector<std::pair<std::string,std::string>> parse_var_mapping(std::string text);

std::vector<std::string> parse_forAll_expr(std::string expr);

std::pair<std::string,std::string> parse_at_text(std::string text);
std::pair<std::string,std::string> parse_goal_text(std::string text);

std::variant<int,std::pair<int,int>> parse_robot_number(std::string text);

#endif