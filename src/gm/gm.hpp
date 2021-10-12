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
#include "../utils/gm_utils.hpp"
#include "../utils/math_utils.hpp"

namespace pt = boost::property_tree;

struct VertexData {
    int parent = -1;
    std::vector<int> children;
    std::string id;
    std::string text;
    std::string type;
    std::map<std::string,std::variant<std::string,std::vector<std::string>,Context,QueriedProperty,AchieveCondition,FailureCondition,std::vector<std::pair<std::string,std::string>>>> custom_props;
    float x;
    float y;
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

std::vector<std::pair<int,VertexData>> parse_gm_nodes(pt::ptree nodes);

std::vector<std::pair<std::pair<int,int>, EdgeData>> parse_gm_edges(pt::ptree links, GMGraph& gm, std::vector<std::pair<int,VertexData>> vertex);

GMGraph graph_from_property_tree(pt::ptree root);

bool exists_path(int source, int target, GMGraph gm);

int find_gm_node_by_id(std::string id, GMGraph gm);

void check_gm_validity(GMGraph gm);
void check_undefined_number_of_robots(GMGraph& gm, std::vector<task> abstract_tasks, std::vector<sort_definition> sort_definitions);
void analyze_custom_props(std::map<std::string,std::string> custom_props, VertexData& v);
void print_gm_nodes_info(GMGraph gm);
void print_gm_var_map_info(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map);
void print_gm(GMGraph gm);

struct sort_edges {
    inline bool operator() (const std::pair<std::pair<int,int>, EdgeData> e1, const std::pair<std::pair<int,int>, EdgeData> e2) {
        std::pair<VertexData,VertexData> e1_source_target = std::make_pair(gm[e1.first.first],gm[e1.first.second]);
        std::pair<VertexData,VertexData> e2_source_target = std::make_pair(gm[e2.first.first],gm[e2.first.second]);

        if(e1.first.first == e2.first.first) {
            return !compare_floats(e1_source_target.second.x,e2_source_target.second.x) && !greater_than_floats(e1_source_target.second.x,e2_source_target.second.x);
        } else {
            if(!exists_path(e1.first.first, e2.first.first, gm) && !exists_path(e2.first.first, e1.first.first, gm)) {
                return !compare_floats(e1_source_target.first.x,e2_source_target.first.x) && !greater_than_floats(e1_source_target.first.x,e2_source_target.first.x);
            } else if(exists_path(e1.first.first, e2.first.first, gm)) {
                return true;
            } else {
                return false;
            }
        }
    }

    sort_edges(GMGraph gm) {
        this->gm = gm;
    }

    private:
        GMGraph gm;
};

#endif