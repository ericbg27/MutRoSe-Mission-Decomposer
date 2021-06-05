#ifndef __KNOWLEDGE_MANAGER_FACTORY
#define __KNOWLEDGE_MANAGER_FACTORY

#include <string>
#include <map>
#include <vector>
#include <memory>

#include "knowledgemanager.hpp"
#include "fileknowledgemanager.hpp"

class KnowledgeManagerFactory {
    public:
        static std::shared_ptr<KnowledgeManager> create_knowledge_manager(std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg, std::map<std::string,std::string> type_mapping);
};

#endif