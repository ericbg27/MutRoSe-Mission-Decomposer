#ifndef __QUERY_SOLVER
#define __QUERY_SOLVER

#include "gm.hpp"
#include "gm_utils.hpp"

class QuerySolver {
    public:
        QuerySolver(GMGraph gm, QueriedProperty q, pt::ptree queried_tree, int node_id);

        void solve_query_statement(std::map<std::string,std::pair<std::string,std::vector<pt::ptree>>>& valid_variables, 
                                    std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>>& gm_var_map);

    private:
        GMGraph gm;
        QueriedProperty q;
        pt::ptree queried_tree;
        int node_id;
};

#endif