#ifndef __ANNOTMANAGER
#define __ANNOTMANAGER

#include <map>
#include <string>
#include <vector>

#include "../at.hpp"
#include "../gm.hpp"
#include "../knowledgemanager/knowledgemanager.hpp"

namespace pt = boost::property_tree;

enum rannot_type {OPERATOR, GOAL, TASK, MEANSEND}; //Goal type is useless (needs checking!)

//Create parser using bison for runtime annotations
struct general_annot {
    rannot_type type;
    std::string content;
    std::vector<general_annot*> children;
    general_annot* parent;
    std::string related_goal;
    bool non_coop = false;
    bool group = true;
    bool divisible = true;
};

enum annot_manager_type {FILEANNOTMANAGER};

class AnnotManager {
    public:
        virtual general_annot* retrieve_gm_annot(GMGraph gm, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>> at_instances) = 0;
        
        virtual void recursive_gm_annot_generation(general_annot* node_annot, std::vector<int> &vctr,  pt::ptree worlddb, GMGraph gm, std::vector<std::string> high_level_loc_types, int current_node,
                                        std::map<std::string,pair<std::string,std::vector<pt::ptree>>>& valid_variables, std::map<int,AchieveCondition> valid_forAll_conditions, 
                                        std::map<int,int>& node_depths) = 0;
        void set_annot_manager_type(annot_manager_type amt);

        annot_manager_type get_annot_manager_type();

    private:
        annot_manager_type am_type;
};

class FileKnowledgeAnnotManager : public AnnotManager {
    public:
        general_annot* retrieve_gm_annot(GMGraph gm, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>> at_instances);

        void recursive_gm_annot_generation(general_annot* node_annot, std::vector<int> &vctr, pt::ptree worlddb, GMGraph gm, std::vector<std::string> high_level_loc_types, int current_node,
                                        std::map<std::string,pair<std::string,std::vector<pt::ptree>>>& valid_variables, std::map<int,AchieveCondition> valid_forAll_conditions, 
                                        std::map<int,int>& node_depths);
        void set_fk_manager(FileKnowledgeManager* manager);

    private:
        FileKnowledgeManager* fk_manager;
};

class AnnotManagerFactory {
    public:
        std::shared_ptr<AnnotManager> create_annot_manager(std::shared_ptr<KnowledgeManager> k_manager);
};

extern std::map<std::string,general_annot*> goals_and_rannots; //Map from goals to runtime annotation

general_annot* retrieve_runtime_annot(std::string id);

general_annot* retrieve_gm_annot(GMGraph gm, pt::ptree worlddb, std::vector<std::string> high_level_loc_types, std::map<std::string,std::vector<AbstractTask>> at_instances);

void recursive_fill_up_runtime_annot(general_annot* rannot, VertexData gm_node);

void recursive_child_replacement(general_annot* copy, general_annot* original);

void rename_at_instances_in_runtime_annot(general_annot* gmannot, std::map<std::string,std::vector<AbstractTask>> at_instances, GMGraph gm);

void recursive_at_instances_renaming(general_annot* rannot, std::map<std::string,int>& at_instances_counter, bool in_forAll, map<string,vector<AbstractTask>> at_instances, GMGraph gm);

void print_runtime_annot_from_general_annot(general_annot* rt);

std::string recursive_rt_annot_build(general_annot* rt);

void solve_query_statement(pt::ptree queried_tree, QueriedProperty q, GMGraph gm, int node_id, std::map<std::string,std::pair<std::string,std::vector<pt::ptree>>>& valid_variables);

pt::ptree get_query_ptree(GMGraph gm, int node_id, map<string,pair<string,vector<pt::ptree>>> valid_variables, map<int,AchieveCondition> valid_forAll_conditions, pt::ptree world_tree);

#endif