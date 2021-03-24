#ifndef __TDG
#define __TDG

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
    bool belongs_to_cycles=false;
    vector<int> cycle_links;
};

enum edge_type {AAND, OOR};
struct EData {
    edge_type type; //OOR, AAND
    int source;
    int target;
};

typedef boost::adjacency_list<boost::vecS,boost::vecS,
                                boost::directedS,
                                NodeData,
                                EData> TDGraph;

typedef boost::graph_traits<TDGraph>::vertex_descriptor vertex_t;

class TDG {
    public:
        TDG(task root_abstract_task, vector<task> a_tasks, vector<task> p_tasks, vector<method> ms);

        vector<vector<task>> retrieve_possible_decompositions();
        vector<vector<task>> decomposition_recursion(vector<int> dfs_nodes, int current_pos, vector<pair<string,string>> parent_vars, 
                                                        vector<literal>& world_state, vector<pair<string,string>> variable_mapping); 

        void add_method_path(NodeData m);
        void add_task_path(NodeData t);
        void add_edge(int s_id, int t_id);
        void change_world_state(task t,vector<literal>& world_state, vector<pair<string,string>> variable_mapping);
        void variable_renaming(task& t, vector<pair<string,string>> var_mapping);
        void print_edges();
        void print_method_possible_orderings(vector<vector<int>> possible_orderings, NodeData n);

        vector<int> DFS_visit();

        pair<bool,int> check_cycle(int m_id, NodeData t);

        vector<vector<int>> find_method_possible_orderings(method m, vector<int> children);
        vector<vector<int>> recursive_method_possible_ordering(map<int,set<int>> precedence_map, vector<vector<int>> current_orderings, set<int> values_to_insert);

        bool check_predicates(task t, vector<pair<string,string>> var_mapping, int t_id, vector<literal>& world_state);
    
    private:
        int root;
        TDGraph tdg;
        vector<task> abstract_tasks;
        vector<task> primitive_tasks;
        vector<method> methods;
};

class TDGDFSVisitor : public boost::default_dfs_visitor {
  public:
    TDGDFSVisitor() : vv(new std::vector<int>()) {}

    void discover_vertex(int v, const TDGraph &gm) const {
        vv->push_back(v);
        return;
    }

    std::vector<int> &GetVector() const { return *vv; }

  private:
    boost::shared_ptr<std::vector<int> > vv;
};

#endif