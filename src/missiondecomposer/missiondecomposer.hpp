#ifndef __MISSIONDECOMPOSER
#define __MISSIONDECOMPOSER

#include <map>
#include <vector>
#include <variant>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "../gm/gm.hpp"
#include "../atmanager/at_manager.hpp"
#include "../annotmanager/annotmanager.hpp"
#include "../config/config.hpp"
#include "../knowledgemanager/knowledgemanager.hpp"
#include "../contextmanager/contextmanager.hpp"
#include "../utils/mission_decomposer_utils.hpp"

namespace pt = boost::property_tree;

enum mission_decomposer_type {FILEMISSIONDECOMPOSER};

class MissionDecomposer {
    public:
        virtual ATGraph build_at_graph(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, 
                                            std::vector<SemanticMapping> semantic_mapping) = 0;
        
        virtual void recursive_at_graph_build(int parent, general_annot* rannot, bool non_coop, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, 
                                                pt::ptree world_db, std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars) = 0;
        void final_context_dependency_links_generation();

        bool check_context_dependency(int parent_node, int context_node, Context context, std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars,
                                    std::vector<SemanticMapping> semantic_mapping);
        bool recursive_context_dependency_checking(int current_node, int context_node, std::pair<bool,std::pair<std::string,predicate_definition>> var_and_pred, std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars, 
                                                    std::vector<SemanticMapping> semantic_mapping, std::vector<int>& visited_nodes, bool parallel_checking);
        
        void create_execution_constraint_edges(int node_id);
        
        void set_verbose(bool verb);
        void set_mission_decomposer_type(mission_decomposer_type mdt);
        void set_world_state(std::vector<ground_literal> ws);
        void set_world_state_functions(std::vector<std::pair<ground_literal,std::variant<int,float>>> wsf);
        void set_at_decomposition_paths(std::map<std::string,std::vector<DecompositionPath>> atpaths);
        void set_at_instances(std::map<std::string,std::vector<AbstractTask>> atinst);
        void set_gm_annot(general_annot* gma);
        void set_gm(GMGraph g);

        mission_decomposer_type get_mission_decomposer_type();
    
    protected:
        bool verbose;
        ATGraph mission_decomposition;
        std::vector<ground_literal> world_state;
        std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions;
        std::map<std::string,std::vector<DecompositionPath>> at_decomposition_paths;
        std::map<std::string,std::vector<AbstractTask>> at_instances;
        general_annot* gmannot;
        GMGraph gm;

    private:
        mission_decomposer_type md_type;
};

class FileKnowledgeMissionDecomposer : public MissionDecomposer {
    public:
        ATGraph build_at_graph(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, 
                                            std::vector<SemanticMapping> semantic_mapping);
        
        void recursive_at_graph_build(int parent, general_annot* rannot, bool non_coop, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, 
                                                pt::ptree world_db, std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars);
        void set_fk_manager(FileKnowledgeManager* manager);

    private:
        FileKnowledgeManager* fk_manager;
};

class MissionDecomposerFactory {
    public:
        std::shared_ptr<MissionDecomposer> create_mission_decomposer(std::shared_ptr<KnowledgeManager> k_manager, std::vector<ground_literal> ws, std::vector<std::pair<ground_literal,std::variant<int,float>>> wsf, std::map<std::string,std::vector<DecompositionPath>> atpaths, std::map<std::string,std::vector<AbstractTask>> atinst, general_annot* gma, GMGraph g, bool verb);
};

#endif