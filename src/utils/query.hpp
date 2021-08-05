#ifndef __QUERY
#define __QUERY

#include <string>
#include <variant>
#include <vector>

#include <boost/property_tree/ptree.hpp>

struct Query {
    std::variant<std::vector<std::string>,std::pair<Query*,Query*>> query;
    bool is_and;

    void solve_query();
};

#endif