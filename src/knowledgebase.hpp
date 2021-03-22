#ifndef __KNOWLEDGEBASE
#define __KNOWLEDGEBASE

#include <map>
#include <set>
#include <string>
#include <vector>
#include <variant>

#include <boost/property_tree/ptree.hpp>

using namespace std;

namespace pt = boost::property_tree;

class KnowledgeBase {
    public:
        KnowledgeBase(string db_name, pt::ptree knowledge, string root_key);

        pt::ptree get_knowledge();

        string get_db_name();
        string get_root_key();

    private:
        pt::ptree knowledge;
        string db_name;
        string root_key;

};

#endif