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
        virtual void construct_knowledge_base(std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, pair<std::string,std::string>>> cfg) = 0;                                    
        
        virtual void initialize_objects(std::map<std::string,std::set<std::string>>& sorts, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>>& at_instances) = 0;
        
        virtual void initialize_world_state(std::vector<ground_literal>& init, std::vector<pair<ground_literal,std::variant<int,float>>>& init_functions, std::vector<SemanticMapping> semantic_mapping, std::map<std::string,std::set<std::string>> sorts) = 0;

        void set_knowledge_type(knowledge_type kt);
        void set_unique_id(std::string id);
        void set_type_mapping(std::map<std::string,std::string> tm);
        void set_db_name(std::string db_name);

        std::string get_unique_id();

        knowledge_type get_knowledge_type();

    protected:
        knowledge_type k_type;
        std::string unique_id;
        std::map<std::string,std::string> type_mapping;
        std::string db_name;
};

void print_world_state(std::vector<ground_literal> world_state, std::vector<std::pair<ground_literal,std::variant<int,float>>> world_functions);

#endif