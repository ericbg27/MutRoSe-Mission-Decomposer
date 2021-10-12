#include "knowledgemanagerfactory.hpp"

using namespace std;

shared_ptr<KnowledgeManager> KnowledgeManagerFactory::create_knowledge_manager(map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> cfg, map<string,string> type_mapping, string db_name) {
    if(cfg.find(db_name) == cfg.end()) {
        string unknown_knowledge_name = "Knowledge [" + db_name + "] was not defined in the configuration file";

        throw std::runtime_error(unknown_knowledge_name);
    }
    
    string db_type = std::get<map<string,string>>(cfg[db_name])["type"];

    string knowledge_unique_id = "name";
    if(db_name == "world_db") {
        knowledge_unique_id = std::get<map<string,string>>(cfg[db_name])["unique_id"];
    }

    if(db_type == "FILE") {
        shared_ptr<KnowledgeManager> file_manager = std::make_shared<FileKnowledgeManager>();

        file_manager->set_knowledge_type(FILEKNOWLEDGE);
        file_manager->set_unique_id(knowledge_unique_id);
        file_manager->set_type_mapping(type_mapping);
        file_manager->set_db_name(db_name);

        return file_manager;
    } else {
        string unsupported_db_type_error = "Type [" + db_type + "] is not supported as database type";

        throw std::runtime_error(unsupported_db_type_error);
    }
}