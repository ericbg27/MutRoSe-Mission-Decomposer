#ifndef __KNOWLEDGEBASE
#define __KNOWLEDGEBASE

#include <map>
#include <set>
#include <string>
#include <vector>
#include <variant>

#include <boost/property_tree/ptree.hpp>

namespace pt = boost::property_tree;

class KnowledgeBase {
    public:
        KnowledgeBase(std::string db_name, pt::ptree knowledge, std::string root_key);

        pt::ptree get_knowledge();

        std::string get_db_name();
        std::string get_root_key();

    private:
        pt::ptree knowledge;
        std::string db_name;
        std::string root_key;

};

#endif