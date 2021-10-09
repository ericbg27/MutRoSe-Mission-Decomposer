#ifndef __IHTNGENERATOR
#define __IHTNGENERATOR

#include "../constraintmanager/constraintmanager.hpp"
#include "../utils/atgraph.hpp"
#include "../utils/tdg_utils.hpp"
#include "../gm/gm.hpp"

struct TaskNode {
    std::string id;
    std::set<std::string> agents;
    std::string name;
};

struct MethodNode {
    std::string task_id;
    std::string name;
    std::set<std::string> agents;
    std::vector<int> decomposition;
};

struct ActionNode {
    std::string name;
    std::set<std::string> agents;
    std::vector<std::string> locations;
};

enum ihtn_node_type {IHTNTASK,IHTNACTION,IHTNMETHOD};

struct IHTNNode {
    ihtn_node_type type;
    std::variant<TaskNode,MethodNode,ActionNode> content;
};

typedef boost::adjacency_list<boost::vecS,boost::vecS,
                                boost::directedS,
                                IHTNNode> IHTN;

class IHTNDFSVisitor : public boost::default_dfs_visitor {
  public:
    IHTNDFSVisitor() : vv(new std::vector<int>()) {}

    void discover_vertex(int v, const IHTN &gm) const {
        vv->push_back(v);
        return;
    }

    std::vector<int> &GetVector() const { return *vv; }

  private:
    boost::shared_ptr<std::vector<int> > vv;
};

class IHTNGenerator {
    public:
        IHTNGenerator(GMGraph gm, ATGraph mission_decomposition, bool verbose, bool pretty_print, std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions, std::vector<std::string> high_level_loc_types, std::map<std::string,std::string> type_mappings, std::map<std::string,CompleteDecompositionPath> decomposition_path_mapping);

        void generate_ihtn(std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map, std::set<std::string> robot_related_sorts);
        IHTN ihtn_create(std::vector<int> nodes, std::map<int,ATNode> nodes_map, std::set<std::string> agents, std::map<int,std::set<std::string>> agents_map); 
    
    private:
        GMGraph gm;
        ATGraph mission_decomposition;
        bool verbose;
        bool pretty_print;
        std::vector<ground_literal> world_state;
        std::vector<std::pair<ground_literal,std::variant<int,float>>> world_state_functions;
        std::vector<std::string> high_level_loc_types;
        std::map<std::string,std::string> type_mappings;
        std::map<std::string,CompleteDecompositionPath> decomposition_path_mapping;
};

std::vector<std::vector<int>> find_decomposition_orderings(std::vector<int> decomposition, std::map<int,std::pair<std::vector<int>,std::vector<constraint_type>>> seq_fb_constraints_map);
std::vector<std::vector<int>> recursive_decomposition_ordering_find(std::vector<std::vector<int>> current_orderings, std::vector<int> decomposition, std::map<int,std::pair<std::vector<int>,std::vector<constraint_type>>> seq_fb_constraints_map);

#endif