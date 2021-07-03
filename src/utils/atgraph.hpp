#ifndef __ATGRAPH
#define __ATGRAPH

#include <string>
#include <vector>
#include <variant>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

#include "domain.hpp"
#include "parsetree.hpp"
#include "at.hpp"
#include "tdg_utils.hpp"

struct Decomposition {
    std::string id;
    AbstractTask at;
    DecompositionPath path;
    std::vector<std::variant<ground_literal,literal>> prec;
    std::vector<std::variant<ground_literal,literal>> eff;
    std::vector<std::variant<std::pair<ground_literal,std::variant<int,float>>,literal>> func_eff;
};

inline bool operator==(const Decomposition& lhs, const Decomposition& rhs) {
    return lhs.id == rhs.id;
}

inline bool operator<(const Decomposition& lhs, const Decomposition& rhs) {
    return lhs.id < rhs.id;
}

enum at_node_type {ATASK,OP,DECOMPOSITION,GOALNODE};

struct ATNode {
    at_node_type node_type;
    std::variant<AbstractTask,std::string,Decomposition> content;
    bool non_coop = false;
    bool group = true;
    bool divisible = true;
    bool is_achieve_type = false;
    int parent;
};

enum at_edge_type {NORMALOR,NORMALAND,CDEPEND,NONCOOP};

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

#endif