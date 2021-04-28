#ifndef __MISSIONDECOMPOSER
#define __MISSIONDECOMPOSER

#include <map>
#include <vector>
#include <variant>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "../gm/gm.hpp"
#include "../atmanager/at.hpp"
#include "../atmanager/at_manager.hpp"
#include "../annotmanager/annotmanager.hpp"
#include "../config/config.hpp"
#include "../knowledgemanager/knowledgemanager.hpp"
#include "../contextmanager/contextmanager.hpp"

namespace pt = boost::property_tree;

struct Decomposition {
    std::string id;
    AbstractTask at;
    std::vector<task> path;
    std::vector<std::variant<ground_literal,literal>> prec;
    std::vector<std::variant<ground_literal,literal>> eff;
};

enum at_node_type {ATASK,OP,DECOMPOSITION,GOALNODE};

struct ATNode {
    at_node_type node_type;
    std::variant<AbstractTask,std::string,Decomposition> content;
    bool non_coop;
    bool group;
    bool divisible;
    bool is_achieve_type;
    int parent;
};

enum at_edge_type {NORMAL,CDEPEND,NONCOOP};

struct ATEdge {
    at_edge_type edge_type;
    int source;
    int target;
    bool group = true;
    bool divisible = true;
};

typedef boost::adjacency_list<boost::vecS,boost::vecS,
                                boost::bidirectionalS,
                                ATNode,
                                ATEdge> ATGraph;

class DFSATVisitor : public boost::default_dfs_visitor {
  public:
    DFSATVisitor() : vv(new std::vector<int>()) {}

    void discover_vertex(int v, const ATGraph &atg) const {
        vv->push_back(v);
        return;
    }

    std::vector<int> &GetVector() const { return *vv; }

  private:
    boost::shared_ptr<std::vector<int> > vv;
};

enum mission_decomposer_type {FILEMISSIONDECOMPOSER};

class MissionDecomposer {
    public:
        virtual ATGraph build_at_graph(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, 
                                            std::vector<SemanticMapping> semantic_mapping) = 0;
        
        virtual void recursive_at_graph_build(int parent, general_annot* rannot, bool non_coop, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, 
                                                pt::ptree world_db, std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars) = 0;
        void set_mission_decomposer_type(mission_decomposer_type mdt);
        void set_world_state(std::vector<ground_literal> ws);
        void set_at_decomposition_paths(std::map<std::string,std::vector<std::vector<task>>> atpaths);
        void set_at_instances(std::map<std::string,std::vector<AbstractTask>> atinst);
        void set_gm_annot(general_annot* gma);
        void set_gm(GMGraph g);

        mission_decomposer_type get_mission_decomposer_type();
    
    protected:
        ATGraph mission_decomposition;
        std::vector<ground_literal> world_state;
        std::map<std::string,std::vector<std::vector<task>>> at_decomposition_paths;
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
        std::shared_ptr<MissionDecomposer> create_mission_decomposer(std::shared_ptr<KnowledgeManager> k_manager, std::vector<ground_literal> ws, std::map<std::string,std::vector<std::vector<task>>> atpaths, std::map<std::string,std::vector<AbstractTask>> atinst, general_annot* gma, GMGraph g);
};

std::pair<ATGraph,std::map<int,int>> generate_trimmed_at_graph(ATGraph mission_decomposition);

void final_context_dependency_links_generation(ATGraph& mission_decomposition);

void instantiate_decomposition_predicates(AbstractTask at, Decomposition& d, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map);
    
bool check_context(Context context, std::vector<ground_literal> world_state, std::vector<SemanticMapping> semantic_mapping, 
                        std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars);

bool check_context_dependency(ATGraph& mission_decomposition, int parent_node, int current_node, Context context, general_annot* rannot,  std::vector<ground_literal> world_state, 
                                std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars, std::map<std::string,std::vector<std::vector<task>>> at_decomposition_paths,
                                    std::vector<SemanticMapping> semantic_mapping);

int find_at_graph_node_by_id(std::string id, ATGraph mission);

std::vector<std::pair<int,ATNode>> find_decompositions(ATGraph mission_decomposition, int node_id);

void create_non_coop_edges(ATGraph& mission_decomposition, int node_id);

void find_non_coop_task_ids(ATGraph mission_decomposition, int node_id, set<int>& task_ids);

bool can_unite_decompositions(Decomposition d1, Decomposition d2, bool non_coop_nodes);

void print_mission_decomposition(ATGraph mission_decomposition);

#endif