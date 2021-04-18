#ifndef __INSTANCESOUTPUT
#define __INSTANCESOUTPUT

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <stack>
#include <queue>
#include <set>
#include <memory>

#include <boost/graph/adjacency_list.hpp>

#include "missiondecomposer.hpp"
#include "constraintmanager.hpp"
#include "gm.hpp"

enum output_generator_type {FILEOUTGEN};

class OutputGenerator {
    public:
        virtual void generate_instances_output(ATGraph mission_decomposition, GMGraph gm, std::pair<std::string,std::string> output, std::vector<ground_literal> world_state, std::vector<SemanticMapping> semantic_mapping,
                                std::map<std::string,set<std::string>> sorts, std::vector<sort_definition> sort_definitions, std::vector<predicate_definition> predicate_definitions) = 0;
};

enum file_output_generator_type {XMLFILEOUTGEN};

class FileOutputGenerator : public OutputGenerator {
    public:
        void output_actions(pt::ptree& output_file, std::map<std::string,task> actions);
        std::map<std::string,std::string> output_tasks(pt::ptree& output_file, std::vector<Decomposition> task_instances, std::vector<SemanticMapping> semantic_mapping);
        void output_constraints(pt::ptree& output_file, std::vector<Constraint> final_mission_constraints, std::map<std::string,std::string> task_id_map);
        void output_mission_decompositions(pt::ptree& output_file, std::vector<std::vector<std::pair<int,ATNode>>> valid_mission_decompositions, std::map<std::string,std::string> task_id_map);

        void set_file_output_generator_type(file_output_generator_type fogt);

        file_output_generator_type get_file_output_generator_type();

    private:
        file_output_generator_type fog_type;
};

class XMLOutputGenerator : public FileOutputGenerator {
    public:
        void generate_instances_output(ATGraph mission_decomposition, GMGraph gm, std::pair<std::string,std::string> output, std::vector<ground_literal> world_state, std::vector<SemanticMapping> semantic_mapping,
                                std::map<std::string,set<std::string>> sorts, std::vector<sort_definition> sort_definitions, std::vector<predicate_definition> predicate_definitions);
};

class FileOutputGeneratorFactory {
    public:
        std::shared_ptr<FileOutputGenerator> create_file_output_generator(std::string file_type);
};

std::vector<std::vector<std::pair<int,ATNode>>> generate_valid_mission_decompositions(ATGraph mission_decomposition, std::vector<Constraint> mission_constraints, std::vector<ground_literal> world_state);

void recursive_valid_mission_decomposition(ATGraph mission_decomposition, std::vector<ground_literal> initial_world_state, std::vector<Constraint> mission_constraints, std::string last_op,
                                            std::queue<std::pair<int,ATNode>>& mission_queue, std::vector<std::pair<std::vector<std::pair<int,ATNode>>,std::vector<ground_literal>>>& valid_mission_decompositions,
                                                std::vector<std::pair<int,ATNode>>& possible_conflicts);

std::queue<std::pair<int,ATNode>> generate_mission_queue(ATGraph mission_decomposition);

std::pair<SemanticMapping,bool> find_predicate_mapping(variant<ground_literal,literal> predicate, std::vector<SemanticMapping> semantic_mappings, std::map<std::string,set<std::string>> sorts,
                                                std::map<std::string,std::string> vars, std::vector<sort_definition> sort_definitions);

void resolve_conflicts(std::vector<std::pair<std::vector<std::pair<int,ATNode>>,std::vector<ground_literal>>>& valid_mission_decompositions, std::vector<std::pair<int,ATNode>> possible_conflicts, ATGraph mission_decomposition,
                        std::vector<Constraint> mission_constraints);

#endif