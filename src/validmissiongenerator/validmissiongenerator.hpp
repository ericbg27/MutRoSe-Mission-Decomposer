#ifndef __VALID_MISSION_GENERATOR
#define __VALID_MISSION_GENERATOR

#include <map>
#include <vector>
#include <string>
#include <variant>
#include <queue>
#include <set>

#include "../missiondecomposer/missiondecomposer.hpp"
#include "../utils/constraint.hpp"
#include "../utils/validmissiongeneratorutils.hpp"

class ValidMissionGenerator {
    public:
        ValidMissionGenerator(ATGraph md, GMGraph g, std::vector<Constraint> mc, std::vector<ground_literal> ws, std::vector<std::pair<ground_literal,std::variant<int,float>>> wsf, std::vector<SemanticMapping> sm, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gmvmap, bool verb, std::set<std::string> r_srts, bool pretty);

        std::pair<std::vector<std::vector<std::pair<int,ATNode>>>,std::set<Decomposition>> generate_valid_mission_decompositions();

        std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>> recursive_valid_mission_decomposition(std::string last_op, std::queue<std::pair<int,ATNode>>& mission_queue, int depth, std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>> effects_to_apply);

        void valid_mission_decompositions_from_unique_branch(std::queue<std::pair<int,ATNode>> mission_queue);

        void check_parallel_op_children(std::queue<std::pair<int,ATNode>>& mission_queue, std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>>& children_effects, int depth, std::pair<int,ATNode> current_node);
        void check_sequential_op_children(std::queue<std::pair<int,ATNode>>& mission_queue, std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>>& children_effects, int depth, std::pair<int,ATNode> current_node);
        void check_fallback_op_children(std::queue<std::pair<int,ATNode>>& mission_queue, std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>>& children_effects, int depth, std::pair<int,ATNode> current_node);

        void check_conditions(std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>> effects_to_apply, std::pair<int,ATNode> current_node);
        void solve_conflicts(std::map<int,std::pair<std::vector<std::variant<ground_literal,std::pair<ground_literal,std::variant<int,float>>>>,std::vector<std::pair<literal,std::vector<std::string>>>>> children_effects);
        
        bool check_preconditions_for_decomposition(std::pair<int,ATNode> task_decomposition, std::pair<std::vector<std::pair<int,ATNode>>,std::set<int>> valid_mission_decomposition, Decomposition& d, std::vector<ground_literal> ws, 
                                                    std::vector<std::pair<ground_literal,std::variant<int,float>>> wsf, std::map<int,std::vector<std::pair<literal,std::vector<std::string>>>> ws_ng);

        std::vector<ground_literal> apply_pred_effects(std::map<int,std::vector<ground_literal>> pred_eff, set<int> tasks_to_consider);
        std::vector<std::pair<ground_literal,std::variant<int,float>>> apply_func_effects(std::map<int,std::vector<std::pair<ground_literal,std::variant<int,float>>>> func_eff, set<int> tasks_to_consider);

        std::queue<std::pair<int,ATNode>> generate_mission_queue();

    private:
        bool verbose;
        bool pretty_print;
        ATGraph mission_decomposition;
        GMGraph gm;
        std::set<Decomposition> expanded_decompositions;
        std::vector<Constraint> mission_constraints;
        std::vector<ground_literal> world_state;
        std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions;
        std::map<int,std::vector<std::pair<literal,std::vector<std::string>>>> non_ground_world_state;
        std::vector<std::pair<std::vector<std::pair<int,ATNode>>,std::set<int>>> valid_mission_decompositions; 
        std::vector<SemanticMapping> semantic_mapping;
        std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map;
        std::set<std::string> robot_related_sorts;
};

#endif