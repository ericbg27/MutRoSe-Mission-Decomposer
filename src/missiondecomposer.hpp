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
#include "annotmanager.hpp"
#include "config.hpp"
#include "knowledgebase.hpp"

using namespace std;

namespace pt = boost::property_tree;

struct Decomposition {
    string id;
    AbstractTask at;
    vector<task> path;
    vector<variant<ground_literal,literal>> prec;
    vector<variant<ground_literal,literal>> eff;
};

enum at_node_type {ATASK,OP,DECOMPOSITION,GOALNODE};

struct ATNode {
    at_node_type node_type;
    variant<AbstractTask,string,Decomposition> content;
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

ATGraph build_at_graph(map<string,vector<AbstractTask>> at_instances, map<string,vector<vector<task>>> at_decomposition_paths, general_annot* gmannot, GMGraph gm, 
                            vector<ground_literal> init, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_vars_map, KnowledgeBase world_db, 
                                vector<SemanticMapping> semantic_mapping);

void recursive_at_graph_build(ATGraph& mission_decomposition, vector<ground_literal> world_state,map<string,vector<AbstractTask>> at_instances, 
                                    map<string,vector<vector<task>>> at_decomposition_paths, general_annot* gmannot, int parent, GMGraph gm, bool non_coop,
                                        map<string, variant<pair<string,string>,pair<vector<string>,string>>>  gm_vars_map, KnowledgeBase world_db, 
                                            vector<SemanticMapping> semantic_mapping,map<string, variant<string,vector<string>>> instantiated_vars);

bool check_path_validity(vector<task> path, vector<ground_literal> world_state, AbstractTask at);

void instantiate_decomposition_predicates(AbstractTask at, Decomposition& d, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_vars_map);

pair<bool,pair<string,predicate_definition>> get_pred_from_context(Context context, vector<SemanticMapping> semantic_mapping);
    
bool check_context(Context context, vector<ground_literal> world_state, vector<SemanticMapping> semantic_mapping, 
                        map<string, variant<string,vector<string>>> instantiated_vars);

bool check_context_dependency(ATGraph& mission_decomposition, int parent_node, int current_node, Context context, general_annot* rannot,  vector<ground_literal> world_state, 
                                map<string, variant<string,vector<string>>> instantiated_vars, map<string,vector<vector<task>>> at_decomposition_paths,
                                    vector<SemanticMapping> semantic_mapping);

int find_at_graph_node_by_id(string id, ATGraph mission);

vector<pair<int,ATNode>> find_decompositions(ATGraph mission_decomposition, int node_id);

void create_non_coop_edges(ATGraph& mission_decomposition, int node_id);

void find_non_coop_task_ids(ATGraph mission_decomposition, int node_id, set<int>& task_ids);

bool can_unite_decompositions(Decomposition d1, Decomposition d2, bool non_coop_nodes);

void print_mission_decomposition(ATGraph mission_decomposition);

#endif