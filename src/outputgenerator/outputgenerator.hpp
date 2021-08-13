#ifndef __OUTPUTGENERATOR
#define __OUTPUTGENERATOR

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <stack>
#include <queue>
#include <set>
#include <memory>

#include <boost/graph/adjacency_list.hpp>

#include "../utils/outputgeneratorutils.hpp"
#include "../missiondecomposer/missiondecomposer.hpp"
#include "../constraintmanager/constraintmanager.hpp"
#include "../gm/gm.hpp"

enum output_generator_type {FILEOUTGEN};

class OutputGenerator {
    public:
        virtual void generate_instances_output(std::vector<SemanticMapping> semantic_mapping, std::map<std::string,set<std::string>> sorts, std::vector<sort_definition> sort_definitions, std::vector<predicate_definition> predicate_definitions,
                                                    std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map, std::set<std::string> robot_related_sorts) = 0;
        
        void set_mission_decomposition(ATGraph md);
        void set_gm(GMGraph g);
        void set_world_state(std::vector<ground_literal> ws);
        void set_world_state_functions(std::vector<std::pair<ground_literal,std::variant<int,float>>> wsf);
        void set_verbose(bool verb);
        void set_pretty_print(bool pretty);

    protected:
        bool verbose;
        bool pretty_print;
        ATGraph mission_decomposition;
        GMGraph gm;
        std::vector<ground_literal> world_state;
        std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions;
};

enum file_output_generator_type {XMLFILEOUTGEN, JSONFILEOUTGEN};

class FileOutputGenerator : public OutputGenerator {
    public:
        virtual void output_actions(pt::ptree& output_file, std::map<std::string,task> actions) = 0;
        virtual std::map<std::string,std::string> output_tasks(pt::ptree& output_file, std::vector<std::pair<Decomposition,std::pair<bool,bool>>> task_instances, std::vector<SemanticMapping> semantic_mapping) = 0;
        virtual void output_constraints(pt::ptree& output_file, std::vector<Constraint> final_mission_constraints, std::map<std::string,std::string> task_id_map) = 0;
        virtual void output_mission_decompositions(pt::ptree& output_file, std::vector<std::vector<std::pair<int,ATNode>>> valid_mission_decompositions, std::map<std::string,std::string> task_id_map) = 0;

        void set_file_output_generator_type(file_output_generator_type fogt);

        file_output_generator_type get_file_output_generator_type();

        void set_output(std::pair<std::string,std::string> out);

    protected:
        std::pair<std::string,std::string> output;

    private:
        file_output_generator_type fog_type;
};

#endif