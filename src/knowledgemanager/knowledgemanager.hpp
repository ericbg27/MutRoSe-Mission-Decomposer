#ifndef __KNOWLEDGEMANAGER
#define __KNOWLEDGEMANAGER

#include <map>
#include <set>
#include <string>
#include <vector>
#include <variant>
#include <memory>

#include <boost/property_tree/ptree.hpp>

#include "../utils/at.hpp"
#include "../config/config.hpp"
#include "../knowledgebase/knowledgebase.hpp"

namespace pt = boost::property_tree;

class KnowledgeManager {
    public:
        virtual void construct_knowledge_base(std::string name, std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg) = 0;                                    
        
        virtual void initialize_objects(std::map<std::string,std::set<std::string>>& sorts, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>>& at_instances, std::map<std::string,std::string> type_mapping) = 0;
        virtual void initialize_world_state(std::vector<ground_literal>& init, std::vector<pair<ground_literal,int>>& init_functions, std::vector<SemanticMapping> semantic_mapping, std::map<std::string,std::string> type_mapping, std::map<std::string,std::set<std::string>> sorts) = 0;

        void set_knowledge_type(knowledge_type kt);

        knowledge_type get_knowledge_type();

    protected:
        knowledge_type k_type;
};                        

class FileKnowledgeManager : public KnowledgeManager {
    public:
        void construct_knowledge_base(std::string name, std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg);                                    
        
        void initialize_objects(std::map<std::string,std::set<std::string>>& sorts, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>>& at_instances, std::map<std::string,std::string> type_mapping);
        void initialize_world_state(std::vector<ground_literal>& init, std::vector<pair<ground_literal,int>>& init_functions, std::vector<SemanticMapping> semantic_mapping, std::map<std::string,std::string> type_mapping, std::map<std::string,std::set<std::string>> sorts);

        shared_ptr<FileKnowledgeBase> get_world_knowledge();
        shared_ptr<FileKnowledgeBase> get_robots_knowledge();

    private:
        shared_ptr<FileKnowledgeBase> world_knowledge;
        shared_ptr<FileKnowledgeBase> robots_knowledge;
};

class KnowledgeManagerFactory {
    public:
        static std::shared_ptr<KnowledgeManager> create_knowledge_manager(std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg);
};

struct Robot {
    std::string name;
    std::string type;
    std::string pos;
};

void print_world_state(std::vector<ground_literal> world_state);

inline bool operator==(const Robot& r1, const Robot& r2) {
    return (r1.name == r2.name);
}

extern std::vector<Robot> assigned_robots;

#endif