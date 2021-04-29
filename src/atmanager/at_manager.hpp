#ifndef __AT_MANAGER
#define __AT_MANAGER

#include <vector>
#include <string>
#include <map>
#include <variant>
#include <memory>

#include <boost/property_tree/ptree.hpp>

#include "../utils/atmanagerutils.hpp"

namespace pt = boost::property_tree;

enum at_manager_type {ATFILE};
class ATManager {
    public:
        virtual std::map<std::string,std::vector<AbstractTask>> generate_at_instances(std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map, 
                                                                                        std::vector<VariableMapping> var_mapping) = 0;
        
        void set_at_manager_type(at_manager_type atm);
        void set_abstract_tasks(std::vector<task> ats);
        void set_gm(GMGraph g);
        void set_high_level_loc_types(std::vector<std::string> hllt);

        at_manager_type get_at_manager_type();
    
    protected:
        std::vector<task> abstract_tasks;
        GMGraph gm;
        std::vector<std::string> high_level_loc_types;
        
    private:
        at_manager_type atm_type;
};

class FileKnowledgeATManager : public ATManager {
    public:
        std::map<std::string,std::vector<AbstractTask>> generate_at_instances(std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map,
                                                                                std::vector<VariableMapping> var_mapping);
    
        void set_fk_manager(FileKnowledgeManager* manager);

    private:
        FileKnowledgeManager* fk_manager;
};

class ATManagerFactory {
    public:
        std::shared_ptr<ATManager> create_at_manager(std::shared_ptr<KnowledgeManager> k_manager, std::vector<task> abstract_tasks, GMGraph gm, std::vector<std::string> high_level_loc_types);
};

#endif