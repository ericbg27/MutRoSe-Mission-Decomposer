#ifndef __ANNOT_MANAGER_UTILS
#define __ANNOT_MANAGER_UTILS

#include <string>
#include <map>
#include <vector>
#include <variant>

#include "at.hpp"
#include "query_utils.hpp"
#include "../gm/gm.hpp"

namespace pt = boost::property_tree;

//###################################################### CONSTANTS DECLARATION #################################################################
const std::string sequential_op = ";";
const std::string parallel_op = "#";
const std::string fallback_op = "FALLBACK";
//##############################################################################################################################################

enum rannot_type {OPERATOR, GOAL, TASK, MEANSEND, EMPTYANNOT}; //Goal type is useless (needs checking!)

struct general_annot {
    rannot_type type = EMPTYANNOT;
    std::string content = "";
    std::vector<general_annot*> children;
    general_annot* parent;
    std::string related_goal = "";
    bool non_coop = false;
    bool group = true;
    bool divisible = true;
    bool or_decomposition = false;
    std::map<std::string,std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> var_maps;
};

extern std::map<std::string,general_annot*> goals_and_rannots; //Map from goals to runtime annotation

general_annot* retrieve_runtime_annot(std::string id);

void recursive_fill_up_runtime_annot(general_annot* rannot, VertexData gm_node);
void recursive_child_replacement(general_annot* copy, general_annot* original);
void rename_at_instances_in_runtime_annot(general_annot* gmannot, std::map<std::string,std::vector<AbstractTask>> at_instances, GMGraph gm);
void recursive_at_instances_renaming(general_annot* rannot, std::map<std::string,int>& at_instances_counter, bool in_forAll, map<string,vector<AbstractTask>> at_instances, GMGraph gm);
void print_runtime_annot_from_general_annot(general_annot* rt);

std::string recursive_rt_annot_build(general_annot* rt);

#endif