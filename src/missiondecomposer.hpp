#ifndef __MISSIONDECOMPOSER
#define __MISSIONDECOMPOSER

#include <map>
#include <vector>
#include <variant>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "gm.hpp"
#include "at.hpp"
#include "at_manager.hpp"
#include "annotmanager.hpp"
#include "config.hpp"
#include "knowledgebase.hpp"
#include "contextmanager.hpp"
#include "knowledgemanager.hpp"

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

ATGraph build_at_graph(std::map<std::string,std::vector<AbstractTask>> at_instances, std::map<std::string,std::vector<std::vector<task>>> at_decomposition_paths, general_annot* gmannot, GMGraph gm, 
                            std::vector<ground_literal> init, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map, KnowledgeBase world_db, 
                                std::vector<SemanticMapping> semantic_mapping);

void recursive_at_graph_build(ATGraph& mission_decomposition, std::vector<ground_literal> world_state,std::map<std::string,std::vector<AbstractTask>> at_instances, 
                                    std::map<std::string,std::vector<std::vector<task>>> at_decomposition_paths, general_annot* gmannot, int parent, GMGraph gm, bool non_coop,
                                        std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>>  gm_vars_map, KnowledgeBase world_db, 
                                            std::vector<SemanticMapping> semantic_mapping,std::map<std::string, std::variant<std::string,std::vector<std::string>>> instantiated_vars);

std::pair<ATGraph,std::map<int,int>> generate_trimmed_at_graph(ATGraph mission_decomposition);

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