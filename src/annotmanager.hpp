#ifndef __ANNOTMANAGER
#define __ANNOTMANAGER

#include "at.hpp"
#include "gm.hpp"

using namespace std;

namespace pt = boost::property_tree;

enum rannot_type {OPERATOR, GOAL, TASK, MEANSEND}; //Goal type is useless (needs checking!)

//Create parser using bison for runtime annotations
struct general_annot {
    rannot_type type;
    string content;
    vector<general_annot*> children;
    general_annot* parent;
    string related_goal;
    bool non_coop = false;
    bool group = true;
    bool divisible = true;
};

extern map<string,general_annot*> goals_and_rannots; //Map from goals to runtime annotation

general_annot* retrieve_runtime_annot(string id);

general_annot* retrieve_gm_annot(GMGraph gm, pt::ptree worlddb, string location_type, map<string,vector<AbstractTask>> at_instances);

void recursive_gm_annot_generation(general_annot* node_annot, vector<int> &vctr, GMGraph gm, pt::ptree worlddb, string location_type, int current_node,
                                        map<string,pair<string,vector<pt::ptree>>>& valid_variables, map<int,AchieveCondition> valid_forAll_conditions, 
                                        map<int,int>& node_depths);

void recursive_child_replacement(general_annot* copy, general_annot* original);

void rename_at_instances_in_runtime_annot(general_annot* gmannot, map<string,vector<AbstractTask>> at_instances);

void recursive_at_instances_renaming(general_annot* rannot, map<string,int>& at_instances_counter, bool in_forAll);

void print_runtime_annot_from_general_annot(general_annot* rt);

string recursive_rt_annot_build(general_annot* rt);

#endif