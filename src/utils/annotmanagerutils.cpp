#include "annotmanagerutils.hpp"

#include <iostream>
#include <regex>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "../rannot/rannot.hpp"
#include "math_utils.hpp"

using namespace std;

int parse_string(const char* in);

map<string,general_annot*> goals_and_rannots;

/*
    Function: retrieve_runtime_annot
    Objective: Retrieve the runtime annotation of some given node. We need to have goals_and_rannots initialized

    @ Input: The text of the desired node in the goal model
    @ Output: The runtime annotation of the given node
*/ 
general_annot* retrieve_runtime_annot(string id) {
    parse_string(id.c_str());

    string node_name = get_node_name(id);

    return goals_and_rannots[node_name];
}

/*
    Function: recursive_fill_up_runtime_annot
    Objective: Based on a current node, fill up execution constraints related attributes of runtime annotation nodes

    @ Input 1: The runtime annotation to be filled
    @ Input 2: The vertex of the Goal Model
    @ Output: None. The runtime annotation nodes are filled
*/
void recursive_fill_up_runtime_annot(general_annot* rannot, VertexData gm_node) {
    if(!gm_node.group || (gm_node.group && !gm_node.divisible)) {
        rannot->non_coop = true;
    } else {
        rannot->non_coop = false;
    }

    if(!gm_node.group) {
        rannot->group = false;
    }
    if(!gm_node.divisible) {
        rannot->divisible = false;
    }

    rannot->related_goal = get_node_name(gm_node.text);

    for(general_annot* child : rannot->children) {
        if(child->type == OPERATOR) {
            recursive_fill_up_runtime_annot(child, gm_node);
        }
    }
}

/*
    Function: recursive_child_replacement
    Objective: Replacing the children nodes of a given annotation, since we are dealing with references.
    This is needed in order to deal with forAll statements runtime annotations

    @ Input 1: A reference to the copy runtime annotation to be created
    @ Input 2: A reference to the original runtime annotation
    @ Output: Void. The copy runtime annotation is initialized with the values of the original one
*/ 
void recursive_child_replacement(general_annot* copy, general_annot* original) {
    if(original->children.size() > 0) {
        for(general_annot* original_child : original->children) {
            general_annot* child_copy = new general_annot();

            child_copy->content = original_child->content;
            child_copy->type = original_child->type;
            child_copy->related_goal = original_child->related_goal;
            child_copy->non_coop = original_child->non_coop;
            child_copy->group = original_child->group;
            child_copy->divisible = original_child->divisible;
            recursive_child_replacement(child_copy, original_child);

            child_copy->parent = copy;
            copy->children.push_back(child_copy);
        }
    }
}

/*
    Function: rename_at_instances_in_runtime_annot
    Objective: Rename AT instances in goal model runtime annotation

    @ Input 1: The goal model runtime annotation
    @ Input 2: The at instances map
    @ Input 3: The goal model as a GMGraph object
    @ Output: Void. The goal model runtime annotation has the AT's instances renamed
*/ 
void rename_at_instances_in_runtime_annot(general_annot* gmannot, map<string,vector<AbstractTask>> at_instances, GMGraph gm) {
    map<string,int> at_instances_counter;

    set<string> considered_tasks;

    map<string,vector<AbstractTask>>::iterator at_inst_it;
    for(at_inst_it = at_instances.begin();at_inst_it != at_instances.end();at_inst_it++) {
        for(AbstractTask at : at_inst_it->second) {
            string task_id;

            if(at.id.find("_") != string::npos) {
                task_id = at_inst_it->second.at(0).id.substr(0,at_inst_it->second.at(0).id.find("_"));
            } else {
                task_id = at_inst_it->second.at(0).id;
            }

            if(considered_tasks.find(task_id) == considered_tasks.end()) {
                at_instances_counter[task_id] = 0;

                considered_tasks.insert(task_id);
            }
        }
    }

    if(gmannot->content == parallel_op && gmannot->related_goal == "") {
        //Dealing with a forAll in the root
        for(general_annot* child : gmannot->children) {
            recursive_at_instances_renaming(child, at_instances_counter, true, at_instances, gm);
        }
    } else {
        for(general_annot* child : gmannot->children) {
            recursive_at_instances_renaming(child, at_instances_counter, false, at_instances, gm);
        }
    }
}

/*
    Function: recursive_at_instances_renaming
    Objective: Rename AT instances in the runtime annotation object and recursive call this renaming step

    @ Input 1: The runtime annotation being considered
    @ Input 2: The counter in order to know which instance we left
    @ Input 3: Boolean flag to know if we have a forAll generated node in the root
    @ Input 4: The at instances map
    @ Input 5: The goal model as a GMGraph object
    @ Output: Void. The runtime goal model annotation is renamed
*/ 
void recursive_at_instances_renaming(general_annot* rannot, map<string,int>& at_instances_counter, bool in_forAll, map<string,vector<AbstractTask>> at_instances, GMGraph gm) {
    set<string> operators {sequential_op,parallel_op,fallback_op};

    set<string>::iterator op_it;

    op_it = operators.find(rannot->content);

    if(op_it == operators.end()) { //If we have a task
        if(rannot->type == TASK) {
            int gm_id = find_gm_node_by_id(rannot->content.substr(0,rannot->content.find("_")), gm);
            pair<string,string> at_id_name = parse_at_text(gm[gm_id].text);

            vector<AbstractTask> aux;
            for(AbstractTask at : at_instances[at_id_name.second]) {
                string at_id = at.id.substr(0,at.id.find("_"));

                if(at_id == at_id_name.first) {
                    aux.push_back(at);
                }
            }

            rannot->content = aux.at(at_instances_counter[at_id_name.first]).id;

            at_instances_counter[at_id_name.first]++;
        } else {
            if(rannot->type == MEANSEND) {
                general_annot* child = rannot->children.at(0);
                recursive_at_instances_renaming(child, at_instances_counter, in_forAll, at_instances, gm);
            }
        }
    } else {
        if(rannot->content == parallel_op && rannot->related_goal == "") {
            for(general_annot* child : rannot->children) {
                recursive_at_instances_renaming(child, at_instances_counter, true, at_instances, gm);
            }
        } else {
            for(general_annot* child : rannot->children) {
                recursive_at_instances_renaming(child, at_instances_counter, in_forAll, at_instances, gm);
            }
        }
    }
}   

/*
    Function: print_runtime_annot_from_general_annot
    Objective: Print the given runtime annotation in the terminal

    @ Input: The runtime annotation to be printed 
    @ Output: Void. The runtime annotation is printed
*/ 
void print_runtime_annot_from_general_annot(general_annot* rt) {
    string rt_annot = "";

    rt_annot = recursive_rt_annot_build(rt);

    cout << rt_annot << endl;
}

/*
    Function: recursive_rt_annot_build
    Objective: Build the runtime annotation string recursively

    @ Input: The current runtime annotation being considered
    @ Output: The current runtime annotation string
*/ 
string recursive_rt_annot_build(general_annot* rt) {
    set<string> operators {sequential_op,parallel_op,fallback_op};

    set<string>::iterator op_it;

    op_it = operators.find(rt->content);

    string annot = "";
    if(rt->non_coop) {
        annot += "NC(";
    }
    if(op_it != operators.end()) {
        if(rt->content == fallback_op) {
            vector<string> children;
            for(general_annot* child : rt->children) {
                children.push_back(recursive_rt_annot_build(child));
            }

            annot += "FALLBACK(";
            int cnt = 0;
            for(string c : children) {
                annot += c;
                cnt++;
                if(cnt == 1) {
                    annot += ",";
                } else {
                    annot += ")";
                }
            }
        } else {
            vector<string> children;
            for(general_annot* child : rt->children) {
                children.push_back(recursive_rt_annot_build(child));
            }

            if(!rt->non_coop) {
                annot += "(";
            }
            unsigned int cnt = 0;
            for(string c : children) {
                annot += c;
                if(cnt < rt->children.size()-1) {
                    annot += rt->content;
                } else {
                    annot += ")";
                }
                cnt++;
            }
        }
    } else {
        if(rt->type == MEANSEND) {
            annot += recursive_rt_annot_build(rt->children.at(0));
        } else {
            annot += rt->content;
        }
    }

    if(rt->non_coop) {
        annot += ")";
    }

    return annot;
}