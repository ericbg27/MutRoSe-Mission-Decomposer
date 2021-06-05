#ifndef __FILE_KNOWLEDGE_MANAGER
#define __FILE_KNOWLEDGE_MANAGER

#include <string>
#include <vector>
#include <map>

#include "knowledgemanager.hpp"

class FileKnowledgeManager : public KnowledgeManager {
    public:
        void construct_knowledge_base(std::string name, std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg);                                    
        
        void initialize_objects(std::map<std::string,std::set<std::string>>& sorts, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>>& at_instances);
        
        void initialize_world_state(std::vector<ground_literal>& init, std::vector<pair<ground_literal,int>>& init_functions, std::vector<SemanticMapping> semantic_mapping, std::map<std::string,std::set<std::string>> sorts);
        void initialize_attribute_mapping(SemanticMapping sm, pt::ptree worlddb_root, std::vector<ground_literal>& init, std::vector<std::pair<ground_literal,int>>& init_functions);
        void initialize_ownership_mapping(SemanticMapping sm, pt::ptree worlddb_root, std::vector<ground_literal>& init);
        void initialize_relationship_mapping(SemanticMapping sm, pt::ptree worlddb_root, std::vector<ground_literal>& init);

        shared_ptr<FileKnowledgeBase> get_world_knowledge();

    private:
        shared_ptr<FileKnowledgeBase> world_knowledge;
};

#endif