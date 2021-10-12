#ifndef __KNOWLEDGEBASE
#define __KNOWLEDGEBASE

#include <map>
#include <set>
#include <string>
#include <vector>
#include <variant>

#include <boost/property_tree/ptree.hpp>

namespace pt = boost::property_tree;

enum knowledge_type {FILEKNOWLEDGE};

class KnowledgeBase {};

enum knowledge_file_type {XML};

class FileKnowledgeBase : public KnowledgeBase {
    public:
        virtual pt::ptree get_knowledge() = 0;

        virtual std::string get_db_name() = 0;
        virtual std::string get_root_key() = 0;
        virtual std::string get_unique_id() = 0;

        void set_knowledge(pt::ptree knowledge);
        void set_db_name(std::string db_name);
        void set_root_key(std::string root_key);
        void set_unique_id(std::string unique_id);
        void set_knowledge_file_type(knowledge_file_type kft);

        knowledge_file_type get_knowledge_file_type();

    protected:
        pt::ptree knowledge;
        std::string db_name;
        std::string root_key;
        std::string unique_id;
        knowledge_file_type kf_type;
};

class XMLKnowledgeBase : public FileKnowledgeBase {    
    public:
        XMLKnowledgeBase(std::string db_name, pt::ptree knowledge, std::string root_key, std::string unique_id);

        pt::ptree get_knowledge();

        std::string get_db_name();
        std::string get_root_key();
        std::string get_unique_id();
};

#endif