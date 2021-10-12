#include "knowledgebase.hpp"

using namespace std;

void FileKnowledgeBase::set_db_name(std::string db_name) {
    this->db_name = db_name;
}

void FileKnowledgeBase::set_root_key(std::string root_key) {
    this->root_key = root_key;
}

void FileKnowledgeBase::set_unique_id(std::string unique_id) {
    this->unique_id = unique_id;
}

void FileKnowledgeBase::set_knowledge(pt::ptree knowledge) {
    this->knowledge = knowledge;
}

void FileKnowledgeBase::set_knowledge_file_type(knowledge_file_type kft) {
    this->kf_type = kft; 
}

knowledge_file_type FileKnowledgeBase::get_knowledge_file_type() {
    return kf_type;
}

XMLKnowledgeBase::XMLKnowledgeBase(string db_name, pt::ptree knowledge, string root_key, string unique_id) {
    set_db_name(db_name);
    set_knowledge(knowledge);
    set_root_key(root_key);
    set_unique_id(unique_id);
}

pt::ptree XMLKnowledgeBase::get_knowledge() {
    return knowledge;
}

string XMLKnowledgeBase::get_db_name() {
    return db_name;
}

string XMLKnowledgeBase::get_root_key() {
    return root_key;
}

string XMLKnowledgeBase::get_unique_id() {
    return unique_id;
}