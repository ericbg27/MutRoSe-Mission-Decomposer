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
        virtual void recursive_at_instances_generation(int current, int depth, std::map<int,int>& node_depths, pt::ptree world_tree, std::vector<VariableMapping> var_mapping,
                                                        std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map, bool insert_events) = 0;
        
        void set_at_manager_type(at_manager_type atm);
        void set_abstract_tasks(std::vector<task> ats);
        void set_gm(GMGraph g);
        void set_high_level_loc_types(std::vector<std::string> hllt);

        at_manager_type get_at_manager_type();
    
    protected:
        std::vector<task> abstract_tasks;
        GMGraph gm;
        std::vector<std::string> high_level_loc_types;
        map<string,vector<AbstractTask>> at_instances;
        std::map<int,AchieveCondition> valid_forAll_conditions;
        std::map<std::string,std::pair<std::string,std::vector<pt::ptree>>> valid_variables;
        std::map<int,int> forAll_inst_id;
        std::map<int,std::vector<std::string>> valid_events;
        
    private:
        at_manager_type atm_type;
};

class FileKnowledgeATManager : public ATManager {
    public:
        std::map<std::string,std::vector<AbstractTask>> generate_at_instances(std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map,
                                                                                std::vector<VariableMapping> var_mapping);
        void recursive_at_instances_generation(int current, int depth, std::map<int,int>& node_depths, pt::ptree world_tree, std::vector<VariableMapping> var_mapping,
                                                std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map, bool insert_events);

        void set_fk_manager(FileKnowledgeManager* manager);

    private:
        FileKnowledgeManager* fk_manager;
};

class ATManagerFactory {
    public:
        std::shared_ptr<ATManager> create_at_manager(std::shared_ptr<KnowledgeManager> k_manager, std::vector<task> abstract_tasks, GMGraph gm, std::vector<std::string> high_level_loc_types);
};

#endif