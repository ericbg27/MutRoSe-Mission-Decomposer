#ifndef __MISSION_DECOMPOSER_UTILS
#define __MISSION_DECOMPOSER_UTILS

#include <string>
#include <vector>
#include <variant>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

#include "domain.hpp"
#include "parsetree.hpp"
#include "../utils/at.hpp"
#include "../contextmanager/contextmanager.hpp"
#include "../annotmanager/annotmanager.hpp"

struct Decomposition {
    std::string id;
    AbstractTask at;
    std::vector<task> path;
    std::vector<std::variant<ground_literal,literal>> prec;
    std::vector<std::variant<ground_literal,literal>> eff;
    std::vector<std::pair<ground_literal,int>> func_eff;
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

std::pair<ATGraph,std::map<int,int>> generate_trimmed_at_graph(ATGraph mission_decomposition);

void instantiate_decomposition_predicates(AbstractTask at, Decomposition& d, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_vars_map);

std::vector<std::pair<int,ATNode>> find_decompositions(ATGraph mission_decomposition, int node_id);

void find_non_coop_task_ids(ATGraph mission_decomposition, int node_id, set<int>& task_ids);

bool can_unite_decompositions(Decomposition d1, Decomposition d2, bool non_coop_nodes);

void print_mission_decomposition(ATGraph mission_decomposition);

#endif