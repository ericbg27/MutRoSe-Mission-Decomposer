#ifndef __VALID_MISSION_GENERATOR
#define __VALID_MISSION_GENERATOR

#include <map>
#include <vector>
#include <string>
#include <variant>
#include <queue>

#include "../missiondecomposer/missiondecomposer.hpp"
#include "../utils/constraint.hpp"

class ValidMissionGenerator {
    public:
        ValidMissionGenerator(ATGraph md, GMGraph g, std::vector<Constraint> mc, std::vector<ground_literal> ws);

        std::vector<std::vector<std::pair<int,ATNode>>> generate_valid_mission_decompositions(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map, vector<SemanticMapping> semantic_mapping);

        void recursive_valid_mission_decomposition(std::string last_op, std::queue<std::pair<int,ATNode>>& mission_queue, std::vector<std::pair<int,ATNode>>& possible_conflicts, 
                                                    std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map, 
                                                            std::vector<SemanticMapping> semantic_mapping, std::map<int,std::vector<ground_literal>>& effects_to_apply, int depth);
        void resolve_conflicts(std::vector<std::pair<int,ATNode>> possible_conflicts);

        std::queue<std::pair<int,ATNode>> generate_mission_queue();

    private:
        ATGraph mission_decomposition;
        GMGraph gm;
        std::vector<Constraint> mission_constraints;
        std::vector<ground_literal> world_state;
        std::vector<std::pair<std::vector<std::pair<int,ATNode>>,std::vector<ground_literal>>> valid_mission_decompositions;      
};

#endif