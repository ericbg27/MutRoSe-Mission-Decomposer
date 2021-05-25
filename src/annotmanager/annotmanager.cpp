#include "annotmanager.hpp"

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <regex>
#include <sstream>
#include <set>

using namespace std;

void AnnotManager::set_annot_manager_type(annot_manager_type atm) {
    am_type = FILEANNOTMANAGER;
}

void AnnotManager::set_gm(GMGraph g) {
    gm = g;
}

void AnnotManager::set_high_level_loc_types(vector<string> hllt){
    high_level_loc_types = hllt;
}

void AnnotManager::set_at_instances(map<string,vector<AbstractTask>> atinst) {
    at_instances = atinst;
}

annot_manager_type AnnotManager::get_annot_manager_type() {
    return am_type;
}

void FileKnowledgeAnnotManager::set_fk_manager(FileKnowledgeManager* manager) {
    fk_manager = manager;
}

/*
    Function: retrieve_gm_annot
    Objective: Retrieve the runtime annotation of the whole goal model

    @ Output: The goal model runtime annotation
*/ 
general_annot* FileKnowledgeAnnotManager::retrieve_gm_annot() {
    vector<int> vctr = get_dfs_gm_nodes(gm);
    
    VertexData root = gm[vctr.at(0)];

    gmannot = retrieve_runtime_annot(root.text);

    recursive_fill_up_runtime_annot(gmannot, gm[vctr.at(0)]);

    map<string,pair<string,vector<pt::ptree>>> valid_variables;

    map<int,int> node_depths;
    map<int,AchieveCondition> valid_forAll_conditions;

    node_depths[vctr.at(0)] = 0;

    int current_node = vctr.at(0);

    vctr.erase(vctr.begin());

    general_annot* empty_annot = new general_annot();
    empty_annot->content = "";
    empty_annot->related_goal = "";

    gmannot->parent = empty_annot;

    shared_ptr<FileKnowledgeBase> world_knowledge_base = fk_manager->get_world_knowledge();

    pt::ptree worlddb;
    if(world_knowledge_base->get_knowledge_file_type() == XML) {
        XMLKnowledgeBase* xml_base = dynamic_cast<XMLKnowledgeBase*>(world_knowledge_base.get());

        worlddb = xml_base->get_knowledge();
    }

    recursive_gm_annot_generation(gmannot, vctr, worlddb, current_node, valid_variables, valid_forAll_conditions, node_depths);

    return gmannot;
}

/*
    Function: recursive_gm_annot_generation
    Objective: Recusive generation of the goal model runtime annotation

    @ Input 1: The current node runtime annotation, generated for the whole goal model so far
    @ Input 2: The nodes indexes visited in a depth-first search manner
    @ Input 3: The world knowledge as a ptree object
    @ Input 4: The current node index
    @ Input 5: The valid variables, given the query goals select statements
    @ Input 6: The valid forAll conditions, given the achieve goals
    @ Input 7: The map of node depths
    @ Output: Void. The runtime goal model annotation is generated
*/ 
void FileKnowledgeAnnotManager::recursive_gm_annot_generation(general_annot* node_annot, vector<int> &vctr,  pt::ptree worlddb, int current_node, map<string,pair<string,vector<pt::ptree>>>& valid_variables, 
                                                                map<int,AchieveCondition> valid_forAll_conditions, map<int,int>& node_depths) {    
    set<string> operators {sequential_op,parallel_op,"FALLBACK","OPT","|"};

    set<string>::iterator op_it;

    op_it = operators.find(node_annot->content);

    int depth;
    if(gm[current_node].parent != -1) {
        depth = node_depths[gm[current_node].parent] + 1;
        node_depths[current_node] = depth;
    } else {
        depth = node_depths[current_node];
    }

    bool is_forAll_goal = false;
    if(gm[current_node].type == istar_goal) {
		if(std::get<string>(gm[current_node].custom_props[goal_type_prop]) == query_goal_type) {
            QueriedProperty q = std::get<QueriedProperty>(gm[current_node].custom_props[queried_property_prop]);

            pt::ptree query_ptree = get_query_ptree(gm, current_node, valid_variables, valid_forAll_conditions, worlddb.get_child("world_db"));

            solve_query_statement(query_ptree,q,gm,current_node,valid_variables);
		} else if(std::get<string>(gm[current_node].custom_props[goal_type_prop]) == achieve_goal_type) {
            is_forAll_goal = true;
			AchieveCondition a = std::get<AchieveCondition>(gm[current_node].custom_props[achieve_condition_prop]);
			if(a.has_forAll_expr) {
				valid_forAll_conditions[depth] = a;
			}
		}
    }

    /*
        -> If we have an operator, simply check its children.

        -> If we have a Goal/Task we have two cases:
            - We may be dealing with a leaf node, in which case we simply finish the execution or
            - We may be dealing with a non-leaf node, in which case we expand it and substitute it for its extension in the parent's children
    */
    if(op_it != operators.end()) { //GM root goal  
        bool expanded_in_forAll = false;

        if(is_forAll_goal) {
            string iterated_var = valid_forAll_conditions[depth].get_iterated_var();
            string iteration_var = valid_forAll_conditions[depth].get_iteration_var();

            int generated_instances = valid_variables[iterated_var].second.size();

            if(generated_instances > 1) {
                int c_node = vctr.at(0);

                vector<general_annot*> new_annots;
                for(int i = 0; i < generated_instances; i++) {
                    general_annot* aux = new general_annot();

                    aux->content = node_annot->content;
                    aux->type = node_annot->type;
                    aux->related_goal = node_annot->related_goal;
                    aux->parent = node_annot;
                    recursive_child_replacement(aux, node_annot);

                    new_annots.push_back(aux);
                }

                node_annot->content = parallel_op;
                node_annot->type = OPERATOR;
                node_annot->related_goal = "";
                node_annot->children.clear();
                for(general_annot* annot : new_annots) {
                    node_annot->children.push_back(annot);
                }

                int index = 0;
                unsigned int child_index = 0;
                for(general_annot* node_ch : node_annot->children) {
                    string var_type = valid_variables[iterated_var].first;
                    vector<pt::ptree> iteration_var_value;
                    iteration_var_value.push_back(valid_variables[iterated_var].second.at(index));

                    valid_variables[iteration_var] = make_pair(var_type, iteration_var_value);

                    for(general_annot* child : node_ch->children) {
                        child->parent = node_ch;
                        vector<int> vctr_aux;
                        recursive_gm_annot_generation(child, vctr, worlddb, c_node, valid_variables, valid_forAll_conditions, node_depths);

                        if(child_index < node_annot->children.size()-1) {
                            unsigned int nodes_diff = vctr_aux.size() - vctr.size();
                                
                            vector<int> to_insert(vctr_aux.begin(),vctr_aux.begin() + nodes_diff);
                            std::reverse(to_insert.begin(),to_insert.end());

                            for(int elem : to_insert) {
                                vctr.insert(vctr.begin(),elem);
                            }   
                        }

                        child_index++;
                    }

                    index++;
                }

                expanded_in_forAll = true;
            } else {
                string var_type = valid_variables[iterated_var].first;
                vector<pt::ptree> iteration_var_value;
                iteration_var_value.push_back(valid_variables[iterated_var].second.at(0));

                valid_variables[iteration_var] = make_pair(var_type, iteration_var_value);
            }
        } 

        if(!expanded_in_forAll) {
            for(general_annot* child : node_annot->children) {
                child->parent = node_annot;
                int c_node = vctr.at(0);
                recursive_gm_annot_generation(child, vctr, worlddb, c_node, valid_variables, valid_forAll_conditions, node_depths);
            }
        }
    } else {
        recursive_fill_up_runtime_annot(node_annot, gm[vctr.at(0)]);

        if(gm[vctr.at(0)].children.size() == 0) { //Leaf Node
            vctr.erase(vctr.begin());
            return;
        } else {
            general_annot* expanded_annot = retrieve_runtime_annot(gm[vctr.at(0)].text);

            if(gm[vctr.at(0)].children.size() > 1) {
                if(expanded_annot->content == "") {
                    expanded_annot->content = sequential_op;
                    for(int child : gm[vctr.at(0)].children) {
                        general_annot* aux = new general_annot();

                        string node_name = get_node_name(gm[child].text);

                        aux->content = node_name;
                        if(node_name.front() == 'G') {
                            aux->type = GOAL;
                        } else {
                            aux->type = TASK;
                        }

                        expanded_annot->children.push_back(aux);
                    }
                }
            } else { //Means-end decomposition
                int only_child = gm[vctr.at(0)].children.at(0);
                expanded_annot->content = get_node_name(gm[vctr.at(0)].text);
                expanded_annot->type = MEANSEND;

                general_annot* aux = new general_annot();

                string node_name = get_node_name(gm[only_child].text);
                
                aux->content = node_name;
                if(node_name.front() == 'G') {
                    aux->type = GOAL;
                } else {
                    aux->type = TASK;
                }

                if(aux->type == TASK) {
                    pair<string,string> node_name_id = parse_at_text(gm[only_child].text);
                    aux->content = node_name_id.first;
                }

                expanded_annot->children.push_back(aux);
            }

            node_annot->content = expanded_annot->content;
            node_annot->type = expanded_annot->type;
            node_annot->children = expanded_annot->children;
            node_annot->related_goal = expanded_annot->related_goal;
            
            vctr.erase(vctr.begin());

            bool expanded_in_forAll = false;
            
            if(is_forAll_goal) {
                string iterated_var = valid_forAll_conditions[depth].get_iterated_var();
                string iteration_var = valid_forAll_conditions[depth].get_iteration_var();

                int generated_instances = valid_variables[iterated_var].second.size();

                if(generated_instances > 1) {
                    int c_node = vctr.at(0);

                    vector<general_annot*> new_annots;
                    for(int i = 0; i < generated_instances; i++) {
                        general_annot* aux = new general_annot();

                        aux->content = node_annot->content;
                        aux->type = node_annot->type;
                        aux->related_goal = node_annot->related_goal;
                        aux->parent = node_annot;
                        aux->non_coop = node_annot->non_coop;
                        aux->group = node_annot->group;
                        aux->divisible = node_annot->divisible;
                        recursive_child_replacement(aux, node_annot);

                        new_annots.push_back(aux);
                    }

                    node_annot->content = parallel_op;
                    node_annot->type = OPERATOR;
                    node_annot->related_goal = "";
                    node_annot->children.clear();
                    node_annot->group = true;
                    node_annot->divisible = true;
                    for(general_annot* annot : new_annots) {
                        node_annot->children.push_back(annot);
                    }

                    int index = 0;
                    unsigned int child_index = 0;
                    for(general_annot* node_ch : node_annot->children) {
                        string var_type = valid_variables[iterated_var].first;
                        vector<pt::ptree> iteration_var_value;
                        iteration_var_value.push_back(valid_variables[iterated_var].second.at(index));

                        valid_variables[iteration_var] = make_pair(var_type, iteration_var_value);

                        for(general_annot* child : node_ch->children) {
                            child->parent = node_ch;
                            vector<int> vctr_aux = vctr;
                            recursive_gm_annot_generation(child, vctr, worlddb, c_node, valid_variables, valid_forAll_conditions, node_depths);
                            
                            if(child_index < node_annot->children.size()-1) {
                                unsigned int nodes_diff = vctr_aux.size() - vctr.size();
                                
                                vector<int> to_insert(vctr_aux.begin(),vctr_aux.begin() + nodes_diff);
                                std::reverse(to_insert.begin(),to_insert.end());

                                for(int elem : to_insert) {
                                    vctr.insert(vctr.begin(),elem);
                                }   
                            }

                            child_index++;
                        }

                        index++;
                    } 

                    expanded_in_forAll = true;
                } else {
                    string var_type = valid_variables[iterated_var].first;
                    size_t begin = var_type.find("(")+1;
                    size_t end = var_type.find(")", begin);
                    var_type = var_type.substr(begin,end-begin); 

                    vector<pt::ptree> iteration_var_value;
                    iteration_var_value.push_back(valid_variables[iterated_var].second.at(0));

                    valid_variables[iteration_var] = make_pair(var_type, iteration_var_value);
                }
            }

            if(!expanded_in_forAll) {
                for(general_annot* child : node_annot->children) {            
                    int c_node = vctr.at(0);
                    child->parent = node_annot;
                    recursive_gm_annot_generation(child, vctr, worlddb, c_node, valid_variables, valid_forAll_conditions, node_depths);
                }
            }
        }
    }
}

shared_ptr<AnnotManager> AnnotManagerFactory::create_annot_manager(shared_ptr<KnowledgeManager> k_manager, GMGraph gm, vector<string> high_level_loc_types, map<string,vector<AbstractTask>> at_instances) {
    shared_ptr<AnnotManager> annot_manager;
    
    if(k_manager->get_knowledge_type() == FILEKNOWLEDGE) {
		annot_manager = std::make_shared<FileKnowledgeAnnotManager>();
		annot_manager->set_annot_manager_type(FILEANNOTMANAGER);
	} else {
		string unsupported_manager_type = "Unsupported manager type found";

		throw std::runtime_error(unsupported_manager_type);
	}

    annot_manager->set_gm(gm);
    annot_manager->set_high_level_loc_types(high_level_loc_types);
    annot_manager->set_at_instances(at_instances);

    return annot_manager;
}