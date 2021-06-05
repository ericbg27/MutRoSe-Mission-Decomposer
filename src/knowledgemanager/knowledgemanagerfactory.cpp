#include "knowledgemanagerfactory.hpp"

using namespace std;

std::shared_ptr<KnowledgeManager> KnowledgeManagerFactory::create_knowledge_manager(std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg) {
    string dbs_type = "";

    vector<string> dbs = {"world_db"};

    for(unsigned int i = 0; i < dbs.size(); i++) {
        string db_name = dbs.at(i);
        string db_type = std::get<map<string,string>>(cfg[db_name])["type"];

        if(dbs_type == "") {
            dbs_type = db_type;
        } else {
            if(db_type != dbs_type) {
                string invalid_databases_types_error = "Databases should be of the same type";

                throw std::runtime_error(invalid_databases_types_error);
            }
        }
    }

    if(dbs_type == "FILE") {
        shared_ptr<KnowledgeManager> file_manager = std::make_shared<FileKnowledgeManager>();

        file_manager->set_knowledge_type(FILEKNOWLEDGE);

        return file_manager;
    } else {
        string unsupported_db_type_error = "Type [" + dbs_type + "] is not supported as database type";

        throw std::runtime_error(unsupported_db_type_error);
    }
}