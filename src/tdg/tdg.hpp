#ifndef __TDG
#define __TDG

#include <vector>

#include "../utils/domain.hpp"
#include "../utils/parsetree.hpp"
#include "../utils/tdg_utils.hpp"
#include "../utils/predicate_utils.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>

enum node_type {AT, PT, M};
struct NodeData {
    int id;
    int parent;
    node_type type;
    method m;
    task t;
    std::vector<int> children;
    std::vector<int> cycle_links;
    bool belongs_to_cycles=false;
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

class TDG {
    public:
        TDG(task root_abstract_task, std::vector<task> a_tasks, std::vector<task> p_tasks, std::vector<method> ms, bool verbose);

        std::vector<DecompositionPath> retrieve_possible_decompositions();
        std::vector<DecompositionPath> decomposition_recursion(std::vector<int> dfs_nodes, int current_pos, std::vector<std::pair<std::string,std::string>> parent_vars, 
                                                        std::vector<literal>& world_state, std::vector<std::pair<std::string,std::string>> variable_mapping);

        void add_method_path(NodeData m);
        void add_task_path(NodeData t);
        void add_edge(int s_id, int t_id);
        void change_world_state(task t,std::vector<literal>& world_state, std::vector<std::pair<std::string,std::string>> variable_mapping);
        void variable_renaming(task& t, std::vector<std::pair<std::string,std::string>> var_mapping);
        void print_edges();
        void print_method_possible_orderings(std::vector<std::vector<int>> possible_orderings, NodeData n);

        std::vector<int> DFS_visit();

        std::pair<bool,int> check_cycle(int m_id, NodeData t);

        std::vector<std::vector<int>> find_method_possible_orderings(method m, std::vector<int> children);
        std::vector<std::vector<int>> recursive_method_possible_ordering(map<int,std::set<int>> precedence_map, std::vector<std::vector<int>> current_orderings, std::set<int> values_to_insert);

        std::pair<bool,std::pair<literal,bool>> check_predicates(task t, std::vector<std::pair<std::string,std::string>> var_mapping, int t_id, std::vector<literal>& world_state);
    
    private:
        bool verbose;
        int root;
        TDGraph tdg;
        std::vector<task> abstract_tasks;
        std::vector<task> primitive_tasks;
        std::vector<method> methods;
        std::vector<ground_literal> init;
        std::vector<std::pair<ground_literal,int>> init_functions;
};

#endif