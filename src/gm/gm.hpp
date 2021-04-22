#ifndef __GM
#define __GM

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <variant>
#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/graph/depth_first_search.hpp>

#include "../utils/parsetree.hpp"
#include "../contextmanager/contextmanager.hpp"
#include "../utils/condition.hpp"

namespace pt = boost::property_tree;

class AchieveCondition : public Condition {
    public:
        bool has_forAll_expr;
        
        std::string get_iterated_var();
        std::string get_iteration_var();
        std::string get_forAll_condition();
        std::string get_condition();

        void set_iterated_var(std::string ivar);
        void set_iteration_var(std::string itvar);
        void set_forAll_condition(std::string f_cond);

        std::variant<std::pair<std::pair<predicate_definition,std::vector<std::string>>,bool>,bool> evaluate_condition(std::vector<SemanticMapping> semantic_mapping, std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map);

    private:
        std::string iterated_var;
        std::string iteration_var;
        std::string forAll_condition;
};

class FailureCondition {
    public:
        std::string condition;
};

struct IterationRule {
    std::string iterated_var;
    std::string iteration_var;
    std::pair<std::string,std::string> result_var;
    std::string result_init;
    std::string end_loop;
};

struct QueriedProperty {
    std::string queried_var;
    std::pair<std::string,std::string> query_var;
    std::vector<std::string> query;
};

struct VertexData {
    int parent;
    std::vector<int> children;
    std::string id;
    std::string text;
    std::string type;
    std::map<std::string,std::variant<std::string,std::vector<std::string>,Context,QueriedProperty,AchieveCondition,FailureCondition,IterationRule,std::vector<std::pair<std::string,std::string>>>> custom_props;
    int x;
    int y;
    bool periodic; 
    float period;
    float deadline;
    bool group = true;
    bool divisible = true;
    bool fixed_robot_num = true;
    std::variant<int,std::pair<int,int>> robot_num;
};

struct EdgeData {
    std::string id;
    std::string type;
    std::string source;
    std::string target;
};

typedef boost::adjacency_list<boost::vecS,boost::vecS,
                                boost::directedS,
                                VertexData,
                                EdgeData> GMGraph;

class DFSVisitor : public boost::default_dfs_visitor {
  public:
    DFSVisitor() : vv(new std::vector<int>()) {}

    void discover_vertex(int v, const GMGraph &gm) const {
        vv->push_back(v);
        return;
    }

    std::vector<int> &GetVector() const { return *vv; }

  private:
    boost::shared_ptr<std::vector<int> > vv;
};

std::vector<int> get_dfs_gm_nodes(GMGraph gm);

void check_gm_validity(GMGraph gm);

std::vector<std::pair<int,VertexData>> parse_gm_nodes(pt::ptree nodes);

std::vector<std::pair<std::pair<int,int>, EdgeData>> parse_gm_edges(pt::ptree links, GMGraph& gm, std::vector<std::pair<int,VertexData>> vertex);

GMGraph graph_from_property_tree(pt::ptree root);

void check_undefined_number_of_robots(GMGraph& gm, std::vector<task> abstract_tasks, std::vector<sort_definition> sort_definitions);

std::string get_node_name(std::string node_text);
std::string parse_gm_var_type(std::string var_type);

std::vector<std::pair<std::string,std::string>> parse_vars(std::string var_decl);
std::vector<std::pair<std::string,std::string>> parse_var_mapping(std::string text);

std::vector<std::string> parse_forAll_expr(std::string expr);

IterationRule parse_iterate_expr(std::string expr);

AchieveCondition parse_achieve_condition(std::string cond);

QueriedProperty parse_select_expr(std::string expr);

std::pair<std::string,std::string> parse_at_text(std::string text);
std::pair<std::string,std::string> parse_goal_text(std::string text);

std::pair<int,int> parse_robot_number(std::string text);

void analyze_custom_props(std::map<std::string,std::string> custom_props, VertexData& v);

int find_gm_node_by_id(std::string id, GMGraph gm);

void print_gm_nodes_info(GMGraph gm);
void print_gm_var_map_info(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map);
void print_gm(GMGraph gm);

struct sort_by_id {
    inline bool operator() (const std::pair<int,VertexData> v1, const std::pair<int,VertexData> v2) {
        std::string v1_id = get_node_name(v1.second.text);
        std::string v2_id = get_node_name(v2.second.text);

        bool v1_goal, v2_goal;
        v1_goal = (v1_id.rfind("G",0) == 0);
        v2_goal = (v2_id.rfind("G",0) == 0);

        if(v1_goal && !v2_goal) {
            return true;
        } else if(!v1_goal && v2_goal) {
            return false;
        } else {
            if(v1_goal && v2_goal) {
                v1_id = v1_id.substr(1);
                v2_id = v2_id.substr(1);

                int v1_i, v2_i;
                v1_i = std::stoi(v1_id);
                v2_i = std::stoi(v2_id);

                return v1_i < v2_i;
            } else {
                v1_id = v1_id.substr(2);
                v2_id = v2_id.substr(2);

                int v1_i, v2_i;
                v1_i = std::stoi(v1_id);
                v2_i = std::stoi(v2_id);

                return v1_i < v2_i;
            }
        }
    }
};

struct sort_edges {
    inline bool operator() (const std::pair<std::pair<int,int>, EdgeData> e1, const std::pair<std::pair<int,int>, EdgeData> e2) {
        if(e1.first.first == e2.first.first) {
            return e1.first.second < e2.first.second;
        } else {
            return e1.first.first < e2.first.first;
        }
    }
};

#endif