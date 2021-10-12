#ifndef __AT_MANAGER
#define __AT_MANAGER

#include <vector>
#include <string>
#include <map>
#include <variant>
#include <memory>

#include <boost/property_tree/ptree.hpp>

#include "../utils/atmanagerutils.hpp"
#include "../knowledgemanager/knowledgemanager.hpp"
#include "../knowledgemanager/fileknowledgemanager.hpp"

namespace pt = boost::property_tree;

enum at_manager_type {ATFILE};
class ATManager {
    public:
        virtual std::map<std::string,std::vector<AbstractTask>> generate_at_instances(std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map, 
                                                                                        std::vector<VariableMapping> var_mapping) = 0;
        void recursive_at_instances_generation(int current, int depth, std::map<int,int>& node_depths, pt::ptree world_tree, std::vector<VariableMapping> var_mapping,
                                                        std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map, bool insert_events);
        
        virtual void query_goal_resolution(int current_node, pt::ptree world_tree, std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map) = 0;
        virtual void achieve_goal_resolution(int current_node, int depth, pt::ptree world_tree, bool insert_events, std::map<int,int>& node_depths, std::vector<VariableMapping> var_mapping,
										std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>>& gm_var_map) = 0;

        void set_at_manager_type(at_manager_type atm);
        void set_abstract_tasks(std::vector<task> ats);
        void set_gm(GMGraph g);
        void set_high_level_loc_types(std::vector<std::string> hllt);
        void erase_invalid_structures(int depth);
        void at_id_instantiation(AbstractTask& at, std::pair<std::string,std::string> at_def);
        void robotnum_prop_instantiation(AbstractTask& at, int current_node);
        void location_prop_instantiation(AbstractTask& at, std::pair<std::string,std::string> at_def, int current_node, std::vector<VariableMapping> var_mapping,
                                            std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>> gm_var_map);
        void params_prop_instantiation(AbstractTask& at, std::pair<std::string,std::string> at_def, int current_node, std::vector<VariableMapping> var_mapping,
                                            std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>> gm_var_map);
        void events_prop_instantiation(AbstractTask& at, bool insert_events);
        
        bool check_trigger_ctx(int current_node, int depth);

        at_manager_type get_at_manager_type();
    
    protected:
        std::vector<task> abstract_tasks;
        std::vector<std::string> high_level_loc_types;
        std::map<std::string,std::vector<AbstractTask>> at_instances;
        std::map<std::string,std::pair<std::string,std::vector<pt::ptree>>> valid_variables;
        std::map<int,AchieveCondition> valid_forAll_conditions;
        std::map<int,int> forAll_inst_id;
        std::map<int,std::vector<std::string>> valid_events;
        GMGraph gm;
        
    private:
        at_manager_type atm_type;
};

class FileKnowledgeATManager : public ATManager {
    public:
        std::map<std::string,std::vector<AbstractTask>> generate_at_instances(std::map<std::string, std::variant<pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map,
                                                                                std::vector<VariableMapping> var_mapping);

        void query_goal_resolution(int current_node, pt::ptree world_tree, std::map<std::string, std::variant<std::pair<std::string,std::string>,pair<std::vector<std::string>,std::string>>>& gm_var_map);
        void achieve_goal_resolution(int current_node, int depth, pt::ptree world_tree, bool insert_events, std::map<int,int>& node_depths, std::vector<VariableMapping> var_mapping,
										std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>>& gm_var_map);

        void set_fk_manager(FileKnowledgeManager* manager);

    private:
        FileKnowledgeManager* fk_manager;
};

class ATManagerFactory {
    public:
        std::shared_ptr<ATManager> create_at_manager(std::shared_ptr<KnowledgeManager> k_manager, std::vector<task> abstract_tasks, GMGraph gm, std::vector<std::string> high_level_loc_types);
};

#endif