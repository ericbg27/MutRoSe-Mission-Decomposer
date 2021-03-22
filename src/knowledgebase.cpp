#include "knowledgebase.hpp"

KnowledgeBase::KnowledgeBase(string db_name, pt::ptree knowledge, string root_key) {
    this->db_name = db_name;
    this->knowledge = knowledge;
    this->root_key = root_key;
}

pt::ptree KnowledgeBase::get_knowledge() {
    return this->knowledge;
}

string KnowledgeBase::get_db_name() {
    return this->db_name;
}

string KnowledgeBase::get_root_key() {
    return this->root_key;
}