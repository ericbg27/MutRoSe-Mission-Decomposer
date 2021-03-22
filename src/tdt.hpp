#ifndef __TDT
#define __TDT

#include <vector>

#include "domain.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>

using namespace std;

enum node_type {AT, PT, M};
struct NodeData {
    int id;
    node_type type;
    method m;
    task t;
    int parent;
    vector<int> children;
};

enum edge_type {AAND, OOR};
struct EData {
    string type; //OOR, AAND
    int source;
    int target;
};

typedef boost::adjacency_list<boost::vecS,boost::vecS,
                                boost::directedS,
                                NodeData,
                                EData> TDTraph;

typedef boost::graph_traits<TDTraph>::vertex_descriptor vertex_t;

class TDT {
    public:
        TDT(task root_abstract_task, vector<task> a_tasks, vector<task> p_tasks, vector<method> ms);

        vector<vector<task>> retrieve_possible_decompositions();
        vector<vector<task>> decomposition_recursion(vector<int> dfs_nodes, int current_pos); 

        void add_method_path(NodeData m);
        void add_task_path(NodeData t);
        void add_edge(int s_id, int t_id);
        void print_edges();

        vector<int> DFS_visit();
    
    private:
        task root;
        TDTraph tdt;
        vector<task> abstract_tasks;
        vector<task> primitive_tasks;
        vector<method> methods;
};

class TDTDFSVisitor : public boost::default_dfs_visitor {
  public:
    TDTDFSVisitor() : vv(new std::vector<int>()) {}

    void discover_vertex(int v, const TDTraph &gm) const {
        vv->push_back(v);
        return;
    }

    std::vector<int> &GetVector() const { return *vv; }

  private:
    boost::shared_ptr<std::vector<int> > vv;
};

#endif