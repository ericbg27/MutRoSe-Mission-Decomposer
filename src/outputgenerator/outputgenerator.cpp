#include "outputgenerator.hpp"

#include <iostream>

using namespace std;

void OutputGenerator::set_mission_decomposition(ATGraph md) {
    mission_decomposition = md;
}

void OutputGenerator::set_gm(GMGraph g) {
    gm = g;
}

void OutputGenerator::set_world_state(vector<ground_literal> ws) {
    world_state = ws;
}

void FileOutputGenerator::set_file_output_generator_type(file_output_generator_type fogt) {
    fog_type = fogt;
}

file_output_generator_type FileOutputGenerator::get_file_output_generator_type() {
    return fog_type;
}

void FileOutputGenerator::set_output(pair<string,string> out) {
    output = out;
}

/*
    Function: find_predicate_mapping
    Objective: Find a semantic mapping involving a given predicate

    @ Input 1: The predicate to be evaluated
    @ Input 2: The vector of semantic mappings
    @ Input 3: The sorts map, where objects are declared
    @ Input 4: The var mappings between HDDL and OCL goal model variables
    @ Input 5: The sort definitions
    @ Output: A pair containing the semantic mapping and a boolean flag indicating if a mapping was found
*/
pair<SemanticMapping, bool> find_predicate_mapping(variant<ground_literal,literal> predicate, vector<SemanticMapping> semantic_mappings, map<string,set<string>> sorts,
                                                    map<string,string> vars, vector<sort_definition> sort_definitions) {
    SemanticMapping prec_mapping;
    bool found_mapping = false;

    if(holds_alternative<ground_literal>(predicate)) {
        /*
            If the predicate is grounded we can search in the declared objects for the necessary attributes
        */
        ground_literal p = get<ground_literal>(predicate);

        for(SemanticMapping sm : semantic_mappings) {
            if(sm.get_mapped_type() == "predicate") {
                predicate_definition map = get<predicate_definition>(sm.get_prop("map"));

                if(map.name == p.predicate) {
                    bool found_args = true;
                    int arg_index = 0;
                    for(string sort : map.argument_sorts) {
                        bool found_arg = false;
                        for(string object : sorts[sort]) {
                            if(object == p.args.at(arg_index)) {
                                found_arg = true;
                                break;
                            }
                        }

                        if(!found_arg) {
                            found_args = false;
                            break;
                        }
                    }

                    if(found_args) { 
                        prec_mapping = sm;
                        found_mapping = true;
                        break;
                    }
                }
            }
        }
    } else {
        literal p = get<literal>(predicate);

        for(SemanticMapping sm : semantic_mappings) {
            if(sm.get_mapped_type() == "predicate") {
                predicate_definition map = get<predicate_definition>(sm.get_prop("map"));

                if(map.name == p.predicate) {
                    bool found_args = true;
                    int arg_index = 0;
                    for(string sort : map.argument_sorts) {
                        bool found_arg = false;
                        
                        /*
                            Here we need to check if the predicate literal is equal to the predicate in the mapping

                            -> In order to be equal, we need to have the same predicate and the same argument types
                        */
                        if(vars[p.arguments.at(arg_index)] == sort) {
                            found_arg = true;
                        } else {
                            bool is_parent_type = false;
                            for(sort_definition s : sort_definitions) {
                                for(string d_sort : s.declared_sorts) {
                                    if(d_sort == vars[p.arguments.at(arg_index)]) {
                                        if(s.has_parent_sort) {
                                            if(s.parent_sort == sort) {
                                                is_parent_type = true;
                                                break;
                                            }
                                        }
                                    }
                                }

                                if(is_parent_type) {
                                    found_arg = true;
                                    break;
                                }
                            }
                        }

                        if(!found_arg) {
                            found_args = false;
                            break;
                        }

                        arg_index++;
                    }

                    if(found_args) { 
                        prec_mapping = sm;
                        found_mapping = true;
                        break;
                    }
                }
            }
        }
    }

    return make_pair(prec_mapping, found_mapping);
}

/*
    Function: generate_valid_mission_decompositions
    Objective: Generate the valid mission decompositions based on constraints and on the world knowledge. This
    function iniatilizes variables based on the root node and calls a recursive function that performs the generation

    @ Input 1: The Task Graph as an ATGraph object
    @ Input 2: The vector of constraints
    @ Input 3: The world state
    @ Output: The valid mission decompositions vector. A mission decomposition is a vector of pairs of the 
    form ([task_id],[task_node])
*/
vector<vector<pair<int,ATNode>>> generate_valid_mission_decompositions(ATGraph mission_decomposition, vector<Constraint> mission_constraints, vector<ground_literal> world_state,
                                                    map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, vector<SemanticMapping> semantic_mapping, GMGraph gm) {
    vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>> valid_mission_decompositions;

    queue<pair<int,ATNode>> mission_queue = generate_mission_queue(mission_decomposition);

    /*
        The idea here is:

        -> Go through the queue and recursively build the decompositions (valid ones)
        -> The initial state of the world will be updated depending if we have a sequential or a parallel operator
        
        -> When we find an operator, we need to establish the relation between previous found nodes
        -> If the operator succeeds another operator, we know that we need to relate a task with task already involved in another constraint
        -> If the operator succeds a task, we know that this operator relates to the last two tasks
    */
    vector<pair<int,ATNode>> possible_conflicts;
    map<int,vector<ground_literal>> effects_to_apply;
    recursive_valid_mission_decomposition(mission_decomposition, world_state, mission_constraints, "", mission_queue, valid_mission_decompositions, possible_conflicts, gm_var_map, semantic_mapping, gm, effects_to_apply, -1);

    vector<vector<pair<int,ATNode>>> final_valid_mission_decompositions;
    for(auto mission_decomposition : valid_mission_decompositions) {
        final_valid_mission_decompositions.push_back(mission_decomposition.first);
    }

    std::cout << "Valid Mission Decompositions: " << std::endl;
    for(auto mission_decomposition : final_valid_mission_decompositions) {
        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;
        unsigned int index = 1;
        for(pair<int,ATNode> node : mission_decomposition) {
            if(index == mission_decomposition.size()) {
                std::cout << get<Decomposition>(node.second.content).id << std::endl;
            } else {
                std::cout << get<Decomposition>(node.second.content).id << " -> ";
            }
            index++;
        }
        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl << std::endl;
    }

    return final_valid_mission_decompositions;
}

/*
    Function: recursive_valid_mission_decomposition
    Objective: Generate the valid mission decompositions based on constraints and on the world knowledge. This is the
    recursive function that in fact generates them.

    @ Input 1: The Task Graph as an ATGraph object
    @ Input 2: The initial world state before visiting the task nodes
    @ Input 3: The vector of constraints
    @ Input 4: The last operation found in the recursive calls
    @ Input 5: A reference to the mission queue
    @ Input 6: The vector of valid mission decompositions
    @ Input 7: The possible conflicts that need to be analyzed
    @ Output: Void. The valid mission decompositions will be generated
*/
void recursive_valid_mission_decomposition(ATGraph mission_decomposition, vector<ground_literal> initial_world_state, vector<Constraint> mission_constraints, string last_op,
                                            queue<pair<int,ATNode>>& mission_queue, vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>>& valid_mission_decompositions,
                                                vector<pair<int,ATNode>>& possible_conflicts, map<string, variant<pair<string,string>,pair<vector<string>,string>>> gm_var_map, 
                                                    vector<SemanticMapping> semantic_mapping, GMGraph gm, map<int,vector<ground_literal>>& effects_to_apply, int depth) {
    /*
        Here we will get the current node and check whether it is an operator or an Abstract Task
    */
   pair<int,ATNode> current_node = mission_queue.front();
   mission_queue.pop();
   depth++;

   if(holds_alternative<string>(current_node.second.content)) {
       /*
            If we have an operator, we need to check if it is parallel or sequential

            -> The behavior is different depending on which operator we are dealing with
       */
        string op = get<string>(current_node.second.content);

        /*
            std::cout << "Next Node: " << next_node.first << std::endl;
            std::cout << "Is forAll? " << next_node.second.is_forAll << std::endl;
        */

        if(op == "#") {
            /*
                If the operator is parallel we:

                -> Go through the queue while the next node in the queue is a child of this operator
                    - This is done checking the out edges of the parallel operator node and verifying if the
                    node in the queue is present
                -> For each child we recursively perform decomposition
            */
            bool checking_children = true;

            while(checking_children) {
                if(mission_queue.size() == 0) {
                    break;
                }
                pair<int,ATNode> next_node = mission_queue.front();

                bool is_child = false;

                ATGraph::out_edge_iterator ei, ei_end;
                for(boost::tie(ei,ei_end) = out_edges(current_node.first,mission_decomposition);ei != ei_end;++ei) {
                    int source = boost::source(*ei,mission_decomposition);
                    int target = boost::target(*ei,mission_decomposition);
                    auto edge = boost::edge(source,target,mission_decomposition).first;

                    int children_num = 0;
                    ATGraph::out_edge_iterator target_ei, target_ei_end;
                    for(boost::tie(target_ei,target_ei_end) = out_edges(target,mission_decomposition);target_ei != target_ei_end;++target_ei) {
                        children_num++;
                    }

                    /*
                        If we have a goal node as child we have to search its children for the next node
                    */
                    if(mission_decomposition[target].node_type == GOALNODE || (mission_decomposition[target].node_type == OP && children_num < 2)) {
                        bool goal_node = true;
                        int child_id = target;
                        while(goal_node) {
                            ATGraph::out_edge_iterator ci, ci_end;
                            for(boost::tie(ci,ci_end) = out_edges(child_id,mission_decomposition);ci != ci_end;++ci) {
                                int s = boost::source(*ci,mission_decomposition);
                                int t = boost::target(*ci,mission_decomposition);
                                auto e = boost::edge(s,t,mission_decomposition).first;

                                /*
                                    A goal node only has a goal as child if it has a Means-end decomposition
                                */
                               if(mission_decomposition[t].node_type != GOALNODE) {
                                    goal_node = false;
                                    if(mission_decomposition[e].edge_type == NORMAL) {
                                        if(t == next_node.first) {
                                            is_child = true;
                                            break;
                                        }
                                    }
                                } else {
                                    child_id = t;
                                }
                            }
                        }

                        if(is_child) {
                            break;
                        }
                    } else {
                        if(mission_decomposition[edge].edge_type == NORMAL) {
                            if(target == next_node.first) {
                                is_child = true;
                                break;
                            }
                        }
                    }
                }

                if(is_child) {
                    recursive_valid_mission_decomposition(mission_decomposition,initial_world_state,mission_constraints,"#",mission_queue,valid_mission_decompositions,possible_conflicts, gm_var_map, semantic_mapping, gm, effects_to_apply, depth);
                } else {
                    checking_children = false;
                }
            }
        } else if(op == ";") {
            /*
                Here we have to deal with the possible conflicts and then erase them
            */
            if(possible_conflicts.size() > 1) {
                std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
                std::cout << "Resolving conflicts..." << std::endl;
                std::cout << "Possible conflicts: " << std::endl;
                for(pair<int,ATNode> t : possible_conflicts) {
                    std::cout << std::get<AbstractTask>(t.second.content).name << std::endl;
                }
                std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
                resolve_conflicts(valid_mission_decompositions, possible_conflicts, mission_decomposition, mission_constraints);
                possible_conflicts.clear();
            }

            bool checking_children = true;
            while(checking_children) {
                if(mission_queue.size() == 0) {
                    break;
                }
                pair<int,ATNode> next_node = mission_queue.front();
                bool is_child = false;

                ATGraph::out_edge_iterator ei, ei_end;
                for(boost::tie(ei,ei_end) = out_edges(current_node.first,mission_decomposition);ei != ei_end;++ei) {
                    int source = boost::source(*ei,mission_decomposition);
                    int target = boost::target(*ei,mission_decomposition);
                    auto edge = boost::edge(source,target,mission_decomposition).first;

                    int children_num = 0;
                    ATGraph::out_edge_iterator target_ei, target_ei_end;
                    for(boost::tie(target_ei,target_ei_end) = out_edges(target,mission_decomposition);target_ei != target_ei_end;++target_ei) {
                        children_num++;
                    }

                    if(mission_decomposition[target].node_type == GOALNODE || (mission_decomposition[target].node_type == OP && children_num < 2)) {
                        bool goal_node = true;
                        int child_id = target;
                        while(goal_node) {
                            ATGraph::out_edge_iterator ci, ci_end;
                            for(boost::tie(ci,ci_end) = out_edges(child_id,mission_decomposition);ci != ci_end;++ci) {
                                int s = boost::source(*ci,mission_decomposition);
                                int t = boost::target(*ci,mission_decomposition);
                                auto e = boost::edge(s,t,mission_decomposition).first;

                                /*
                                    A goal node only has a goal as child if it has a Means-end decomposition
                                */
                               if(mission_decomposition[t].node_type != GOALNODE) {
                                    goal_node = false;
                                    if(mission_decomposition[e].edge_type == NORMAL) {
                                        if(t == next_node.first) {
                                            is_child = true;
                                            break;
                                        }
                                    }
                                } else {
                                    child_id = t;
                                }
                            }
                        }

                        if(is_child) {
                            break;
                        }
                    } else {
                        if(mission_decomposition[edge].edge_type == NORMAL) {
                            if(target == next_node.first) {
                                is_child = true;
                                break;
                            }
                        }
                    }
                }

                if(is_child) {
                    recursive_valid_mission_decomposition(mission_decomposition,initial_world_state,mission_constraints,";",mission_queue,valid_mission_decompositions,possible_conflicts, gm_var_map, semantic_mapping, gm, effects_to_apply, depth);
                } else {
                    checking_children = false;
                }
            }
        }

        vector<int> keys_to_erase;

        map<int,vector<ground_literal>>::iterator eff_it;
        for(eff_it = effects_to_apply.begin(); eff_it != effects_to_apply.end(); ++eff_it) {
            if(eff_it->first >= depth) {
                for(auto& vmd : valid_mission_decompositions) {
                    for(ground_literal eff : eff_it->second) {
                        bool found_pred = false;

                        for(ground_literal& state : vmd.second) {
                            if(state.predicate == eff.predicate) {
                                bool equal_args = true;

                                int arg_index = 0;
                                for(string arg : state.args) {
                                    if(arg != eff.args.at(arg_index)) {
                                        equal_args = false;
                                        break;
                                    }

                                    arg_index++;
                                }

                                if(equal_args) {
                                    found_pred = true;

                                    if(state.positive != eff.positive) {
                                        state.positive = eff.positive;
                                    }

                                    break;
                                }
                            }
                        }

                        if(!found_pred) {
                            vmd.second.push_back(eff);
                        }
                    }
                }   

                keys_to_erase.push_back(eff_it->first);
            }
        }
        
        for(int key : keys_to_erase) {
            effects_to_apply.erase(key);
        }

        if(current_node.second.is_achieve_type) {
            string achievel_goal_id;

            if(current_node.second.node_type != GOALNODE) {
                ATGraph::out_edge_iterator ei, ei_end;

                for(boost::tie(ei,ei_end) = out_edges(current_node.first,mission_decomposition);ei != ei_end;++ei) {
                    auto source = boost::source(*ei,mission_decomposition);
                    auto target = boost::target(*ei,mission_decomposition);
                    auto edge = boost::edge(source,target,mission_decomposition);

                    if(mission_decomposition[edge.first].edge_type == NORMAL) {
                        if(mission_decomposition[target].node_type == GOALNODE) {
                            achievel_goal_id = std::get<string>(mission_decomposition[target].content);
                            break;
                        }
                    }
                }
            } else {
                achievel_goal_id = std::get<string>(current_node.second.content);
            }

            int gm_node_id = find_gm_node_by_id(achievel_goal_id, gm);

            AchieveCondition achieve_condition = get<AchieveCondition>(gm[gm_node_id].custom_props[achieve_condition_prop]);

            vector<ground_literal> achieve_condition_predicates;

            variant<pair<pair<predicate_definition,vector<string>>,bool>,bool> evaluation = achieve_condition.evaluate_condition(semantic_mapping, gm_var_map);
            
            bool need_predicate_checking = false;
            if(holds_alternative<pair<pair<predicate_definition,vector<string>>,bool>>(evaluation)) {
                pair<pair<predicate_definition,vector<string>>,bool> eval = std::get<pair<pair<predicate_definition,vector<string>>,bool>>(evaluation);

                for(string value : eval.first.second) {
                    ground_literal aux;

                    aux.predicate = eval.first.first.name;
                    aux.positive = !eval.second;
                    aux.args.push_back(value);

                    achieve_condition_predicates.push_back(aux);
                } 

                need_predicate_checking = true;
            }

            if(need_predicate_checking) {
                vector<int> decompositions_to_erase;
                int decomposition_index = 0;
                for(auto decomposition : valid_mission_decompositions) {
                    vector<ground_literal> world_state = decomposition.second;

                    bool valid_achieve_condition = true;
                    for(ground_literal forAll_pred : achieve_condition_predicates) {
                        for(ground_literal state : world_state) {
                            if(state.predicate == forAll_pred.predicate) {
                                bool equal_args = true;

                                int arg_index = 0;
                                for(string arg : state.args) {
                                    if(arg != forAll_pred.args.at(arg_index)) {
                                        equal_args = false;
                                        break;
                                    }
                                                
                                    arg_index++;
                                }
                                            
                                if(!equal_args) {
                                    break;
                                }

                                if(state.positive != forAll_pred.positive) {
                                    valid_achieve_condition = false;
                                    break;
                                }
                            }
                        }

                        if(!valid_achieve_condition) {
                            break;
                        }
                    }

                    if(!valid_achieve_condition) {
                        decompositions_to_erase.push_back(decomposition_index);
                    }

                    decomposition_index++;
                }

                std::sort(decompositions_to_erase.begin(), decompositions_to_erase.end());
                for(auto i = decompositions_to_erase.rbegin(); i != decompositions_to_erase.rend(); ++i) {
                    valid_mission_decompositions.erase(valid_mission_decompositions.begin() + *i);
                }

                if(valid_mission_decompositions.size() == 0) {
                    string forAll_condition_not_achieved = "No decomposition satisfied achieve condition of goal " + achievel_goal_id;

                    throw std::runtime_error(forAll_condition_not_achieved);
                }
            }
        }
    } else {
       /*
            If we have an AT we have to do the following for each decomposition

            -> For each valid mission decomposition, check if the world state satisfies the decomposition precondition
                - If there is no valid mission decomposition, just check against the initial world state
            -> If the world state satisfies the preconditions, add the task to the valid mission decomposition
                - If not, we will still need to check if some task in the valid mission decomposition is involved in a sequential constraint
                with the actual AT. This relates to Context Dependencies
                - If the effect of this task makes the world state satisfy the precondition, add this decomposition in the valid mission decomposition
            
            -> NOTE: IF A TASK DOES NOT HAVE ANY VALID DECOMPOSITION WE RAISE AN ERROR SINCE FOR NOW WE HAVE ONLY AND TASKS IN THE GOAL MODEL
       */
        vector<pair<int,ATNode>> task_decompositions;

        ATGraph::out_edge_iterator ei, ei_end;
        for(boost::tie(ei,ei_end) = out_edges(current_node.first,mission_decomposition);ei != ei_end;++ei) {
            int target = boost::target(*ei,mission_decomposition);

            if(mission_decomposition[target].node_type == DECOMPOSITION) {
                ATNode d = mission_decomposition[target];

                task_decompositions.push_back(make_pair(target,d));
            }
        }

        bool add_to_possible_conflicts = false;
        if(valid_mission_decompositions.size() > 0) {
            /*
                Here is the case where we have at least one valid mission decomposition

                -> We must have at least one decomposition valid for each valid mission decomposition
                    - If we don't have, we must raise an error
            */
            vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>> new_valid_mission_decompositions;
            for(auto valid_mission_decomposition : valid_mission_decompositions) {
                bool valid_task_decomposition = false; //Check if one task decomposition is valid for this mission decomposition

                for(pair<int,ATNode> task_decomposition : task_decompositions) {
                    Decomposition d = get<Decomposition>(task_decomposition.second.content);
                    
                    vector<ground_literal> world_state = valid_mission_decomposition.second;
                    vector<pair<int,ATNode>> m_decomposition = valid_mission_decomposition.first;

                    bool preconditions_hold = true;
                    for(auto prec : d.prec) {
                        if(holds_alternative<ground_literal>(prec)) {                   
                            ground_literal p = get<ground_literal>(prec);
                            
                            for(ground_literal state : world_state) {
                                if(state.predicate == p.predicate) {
                                    bool equal_args = true;

                                    int index = 0;
                                    for(string arg : state.args) {
                                        if(arg != p.args.at(index)) {
                                            equal_args = false;
                                            break;
                                        }
                                    }

                                    if(equal_args) {
                                        if(state.positive != p.positive) {
                                            preconditions_hold = false;
                                            break;
                                        }
                                    }
                                }
                            }
                        } 

                        if(!preconditions_hold) {
                            break;
                        }
                    }

                    /*
                        Check for any context dependency in current decomposition. If a valid mission decomposition does not contain any tasks
                        involved in this kind of dependency, preconditions do not hold.
                    */
                    if(preconditions_hold) {
                        ATGraph::in_edge_iterator iei, ied;

                        bool found_cdepend_node = false;
                        bool has_cdependency = false;
                        for(boost::tie(iei,ied) = boost::in_edges(task_decomposition.first,mission_decomposition); iei != ied; ++iei) {
                            int source = boost::source(*iei,mission_decomposition);
                            int target = boost::target(*iei,mission_decomposition);
                            auto edge = boost::edge(source,target,mission_decomposition).first;

                            ATEdge e = mission_decomposition[edge];
                            if(e.edge_type == CDEPEND) {
                                has_cdependency = true;
                                for(pair<int,ATNode> node : valid_mission_decomposition.first) {
                                    if(node.first == source) {
                                        found_cdepend_node = true;
                                        break;
                                    }
                                }
                            }

                            if(found_cdepend_node) {
                                break;
                            }
                        }

                        if(!found_cdepend_node && has_cdependency) {
                            preconditions_hold = false;
                        }
                    }

                    if(preconditions_hold) {
                        /*
                            If preconditions hold we create a new valid mission decomposition with this task decomposition added to the mission decomposition

                            -> If we have a parallel operator we do not update the world state and add the AT to the possible conflicts
                            -> If we have a sequential operator we update the world state and put it into the valid mission decomposition
                        */
                        m_decomposition.push_back(task_decomposition);

                        if(last_op == "#") {
                            new_valid_mission_decompositions.push_back(make_pair(m_decomposition,world_state));
                            for(auto eff : d.eff) {
                                if(holds_alternative<ground_literal>(eff)) {
                                    effects_to_apply[depth].push_back(std::get<ground_literal>(eff));
                                }
                            }
                            add_to_possible_conflicts = true;
                        } else if(last_op == ";") {
                            //Update initial world state
                            vector<ground_literal> updated_state = world_state;
                            for(auto eff : d.eff) {
                                if(holds_alternative<ground_literal>(eff)) {
                                    ground_literal e = get<ground_literal>(eff);

                                    for(ground_literal& state : updated_state) {
                                        if(state.predicate == e.predicate) {
                                            bool equal_args = true;

                                            int index = 0;
                                            for(string arg : state.args) {
                                                if(arg != e.args.at(index)) {
                                                    equal_args = false;
                                                    break;
                                                }
                                            }

                                            if(equal_args) {
                                                if(state.positive != e.positive) {
                                                    state.positive = e.positive;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            new_valid_mission_decompositions.push_back(make_pair(m_decomposition,updated_state));
                        }

                        valid_task_decomposition = true;
                    } else {
                        /*
                            Here we have to put the code for checking for context dependecies

                            -> We need to check for the possible conflicts
                            -> If there is any we check the constraints to see if we can add the task decomposition to the mission decomposition or not
                                - If we can, everything is fine and we add it
                                - If we can't move on to the next
                        */
                       std::cout << "Preconditions did not hold for task: " << d.id << std::endl;
                    }
                }

                if(!valid_task_decomposition) {
                    AbstractTask at = get<AbstractTask>(current_node.second.content);
                    string invalid_task_decomposition_error = "NO VALID DECOMPOSITIONS FOR TASK " + at.id + ": " + at.name;
                    
                    throw std::runtime_error(invalid_task_decomposition_error);
                }
            }

            valid_mission_decompositions = new_valid_mission_decompositions;
        } else {
            //Here is the case where we have no valid mission decompositions yet
            bool at_least_one_decomposition_valid = false;

            for(pair<int,ATNode> task_decomposition : task_decompositions) {
                //Check preconditions using the initial world state
                Decomposition d = get<Decomposition>(task_decomposition.second.content);
                bool preconditions_hold = true;
                for(auto prec : d.prec) {
                    if(holds_alternative<ground_literal>(prec)) {
                        ground_literal p = get<ground_literal>(prec);
                        
                        for(ground_literal state : initial_world_state) {
                            if(state.predicate == p.predicate) {
                                bool equal_args = true;

                                int index = 0;
                                for(string arg : state.args) {
                                    if(arg != p.args.at(index)) {
                                        equal_args = false;
                                        break;
                                    }
                                }

                                if(equal_args) {
                                    if(state.positive != p.positive) {
                                        preconditions_hold = false;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if(!preconditions_hold) {
                        break;
                    }
                }

                if(preconditions_hold) {
                    /*
                        If preconditions hold we create a new valid mission decomposition

                        -> If we have a parallel operator we do not update the world state and add the AT to the possible conflicts
                        -> If we have a sequential operator we update the world state and put it into the valid mission decomposition
                    */
                    AbstractTask at1 = get<AbstractTask>(current_node.second.content);
                    pair<vector<pair<int,ATNode>>,vector<ground_literal>> new_valid_mission;

                    vector<pair<int,ATNode>> new_decomposition;
                    new_decomposition.push_back(task_decomposition);

                    if(last_op == "#") {
                        new_valid_mission = make_pair(new_decomposition,initial_world_state);
                        for(auto eff : d.eff) {
                            if(holds_alternative<ground_literal>(eff)) {
                                effects_to_apply[depth].push_back(std::get<ground_literal>(eff));
                            }
                        }
                        add_to_possible_conflicts = true;
                    } else if(last_op == ";") {
                        //Update initial world state
                        vector<ground_literal> updated_state = initial_world_state;
                        for(auto eff : d.eff) {
                            if(holds_alternative<ground_literal>(eff)) {
                                ground_literal e = get<ground_literal>(eff);

                                for(ground_literal& state : updated_state) {
                                    if(state.predicate == e.predicate) {
                                        bool equal_args = true;

                                        int index = 0;
                                        for(string arg : state.args) {
                                            if(arg != e.args.at(index)) {
                                                equal_args = false;
                                                break;
                                            }
                                        }

                                        if(equal_args) {
                                            if(state.positive != e.positive) {
                                                state.positive = e.positive;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        new_valid_mission = make_pair(new_decomposition,updated_state);
                    }

                    valid_mission_decompositions.push_back(new_valid_mission);
                    at_least_one_decomposition_valid = true;
                } 
            }

            if(!at_least_one_decomposition_valid) {
                AbstractTask at = get<AbstractTask>(current_node.second.content);
                string invalid_task_decomposition_error = "NO VALID DECOMPOSITIONS FOR TASK " + at.id + ": " + at.name;
                
                throw std::runtime_error(invalid_task_decomposition_error);
            }
        }

        if(add_to_possible_conflicts) {
            possible_conflicts.push_back(current_node);
        }
   }
}

/*
    Function: generate_mission_queue
    Objective: Generate the mission queue based on the Task Graph

    @ Input: The Task Graph as an ATGraph object
    @ Output: The generated mission queue
*/
queue<pair<int,ATNode>> generate_mission_queue(ATGraph mission_decomposition) {
    auto nodes = vertices(mission_decomposition);

    int graph_size = *nodes.second - *nodes.first;

    /*
        Go through the graph in a DFS order and put nodes in a queue

        -> Goal Nodes are not considered
        -> Operator nodes with only one child are not considered
    */
    queue<pair<int,ATNode>> mission_queue;

    //Populate the mission queue
    for(int i = 0;i < graph_size;i++) {
        if(mission_decomposition[i].node_type == ATASK) {
            mission_queue.push(make_pair(i,mission_decomposition[i]));
        } else if(mission_decomposition[i].node_type == OP) {
            int out_edge_num = 0;
            ATGraph::out_edge_iterator ei, ei_end;

            //Only insert OP nodes that have more than one outer edge of normal type (more than one child)
            for(boost::tie(ei,ei_end) = out_edges(i,mission_decomposition);ei != ei_end;++ei) {
                auto source = boost::source(*ei,mission_decomposition);
                auto target = boost::target(*ei,mission_decomposition);
                auto edge = boost::edge(source,target,mission_decomposition);

                if(mission_decomposition[edge.first].edge_type == NORMAL) {
                    out_edge_num++;
                }
            }

            if(out_edge_num > 1) {
                mission_queue.push(make_pair(i,mission_decomposition[i]));
            }
        }
    }

    queue<pair<int,ATNode>> queue_copy = mission_queue;
    std::cout << "Mission Queue" << std::endl;
    while(!queue_copy.empty()) {
        pair<int,ATNode> node = queue_copy.front();
        queue_copy.pop(); 
        if(node.second.node_type == ATASK) {
            std::cout << "ATASK: " << get<AbstractTask>(node.second.content).id << " - " << get<AbstractTask>(node.second.content).name << std::endl;
        } else {
            std::cout << "OPERATOR: " << get<string>(node.second.content) << std::endl;
        }
    }

    return mission_queue;
}

/*
    Function: resolve_conflicts
    Objective: Resolve possible conflicts and add actual conflicts to the conflicts vector. If conflicts appears
    we must remove valid decompositions that contain conflicting tasks

    @ Input 1: A reference to the vector of valid mission decompositions
    @ Input 2: The vector of possible conflicts
    @ Input 3: The Task Graph as an ATGraph object
    @ Input 4: The vector of mission constraints
    @ Output: Void. The valid mission decompositions may be trimmed

    NOTES: Here we do not consider conditional effects yet!
*/

void resolve_conflicts(vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>>& valid_mission_decompositions, vector<pair<int,ATNode>> possible_conflicts,
                        ATGraph mission_decomposition, vector<Constraint> mission_constraints) {
    vector<pair<pair<int,ATNode>,pair<int,ATNode>>> actual_conflicts;

    map<int,unsigned int> task_decompositions_number;

    for(unsigned int i = 0; i < possible_conflicts.size()-1; i++) {
        for(unsigned int j = i+1; j < possible_conflicts.size(); j++) {
            pair<int,ATNode> t1 = possible_conflicts.at(i);
            pair<int,ATNode> t2 = possible_conflicts.at(j);

            vector<pair<int,ATNode>> t1_decompositions;
            ATGraph::out_edge_iterator ei, ei_end;
            for(boost::tie(ei,ei_end) = out_edges(t1.first,mission_decomposition);ei != ei_end;++ei) {
                int target = boost::target(*ei,mission_decomposition);

                if(mission_decomposition[target].node_type == DECOMPOSITION) {
                    ATNode d = mission_decomposition[target];

                    t1_decompositions.push_back(make_pair(target,d));
                }
            }
            task_decompositions_number[t1.first] = t1_decompositions.size();

            vector<pair<int,ATNode>> t2_decompositions;
            for(boost::tie(ei,ei_end) = out_edges(t2.first,mission_decomposition);ei != ei_end;++ei) {
                int target = boost::target(*ei,mission_decomposition);

                if(mission_decomposition[target].node_type == DECOMPOSITION) {
                    ATNode d = mission_decomposition[target];

                    t2_decompositions.push_back(make_pair(target,d));
                }
            }
            task_decompositions_number[t2.first] = t2_decompositions.size();

            for(unsigned int k1 = 0; k1 < t1_decompositions.size(); k1++) {
                for(unsigned int k2 = 0; k2 < t2_decompositions.size(); k2++) {
                    pair<int,Decomposition> d1 = make_pair(t1_decompositions.at(k1).first, std::get<Decomposition>(t1_decompositions.at(k1).second.content));
                    pair<int,Decomposition> d2 = make_pair(t2_decompositions.at(k2).first, std::get<Decomposition>(t2_decompositions.at(k2).second.content));

                    bool is_non_divisible_or_non_group = false;
                    for(Constraint c : mission_constraints) {
                        if(c.type == NC) {
                            if(c.nodes_involved.first.first == d1.first) {
                                if(c.nodes_involved.second.first == d2.first) {
                                    if(!c.group || !c.divisible) {
                                        is_non_divisible_or_non_group = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    /*
                        We need to check:
                            -> ground_literal effects for every decomposition pair
                            -> robot-related literal effects if is_non_divisible_or_non_group is true 

                        How do we check if effects have the same arguments?
                            - If it is constant, no problem
                            - If it is a variable and it is robottype or robot:
                                * Evaluate is_non_divisible_or_non_group and then
                                proceed to verify equality
                    */
                    bool has_conflict = false;
                    for(auto eff1 : d1.second.eff) {
                        for(auto eff2 : d2.second.eff) {
                            if(holds_alternative<ground_literal>(eff1)) {
                                if(holds_alternative<ground_literal>(eff2)) {
                                    ground_literal e1 = std::get<ground_literal>(eff1);
                                    ground_literal e2 = std::get<ground_literal>(eff2);

                                    if(e1.predicate == e2.predicate) {
                                        bool equal_args = true;
                                        for(unsigned int arg_index = 0; arg_index < e1.args.size(); arg_index++) {
                                            if(e1.args.at(arg_index) != e2.args.at(arg_index)) {
                                                equal_args = false;
                                                break;
                                            }
                                        }
                                        if(equal_args) {
                                            if(e1.positive != e2.positive) {
                                                has_conflict = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                            } else if(holds_alternative<literal>(eff1)) {
                                if(holds_alternative<literal>(eff2)) {
                                    literal e1 = std::get<literal>(eff1);
                                    literal e2 = std::get<literal>(eff2);

                                    if(e1.predicate == e2.predicate) {
                                        bool equal_args = true;
                                        for(unsigned int arg_index = 0; arg_index < e1.arguments.size(); arg_index++) {
                                            if(e1.arguments.at(arg_index).rfind("?",0) == 0) { // Argument is variable
                                                if(e2.arguments.at(arg_index).rfind("?",0) == 0) {
                                                    // Find argument in abstract task definition
                                                    task at_def = d1.second.at.at;
                                                    for(pair<string,string> var : at_def.vars) {
                                                        if(var.first == e1.arguments.at(arg_index)) {
                                                            if(var.second == "robot" || var.second == "robotteam") {
                                                                if(!is_non_divisible_or_non_group) {
                                                                    equal_args = false;
                                                                }
                                                            }
                                                            break;
                                                        }
                                                    }
                                                    if(!equal_args) {
                                                        break;
                                                    }
                                                }
                                            } else {
                                                if(e1.arguments.at(arg_index) != e2.arguments.at(arg_index)) {
                                                    equal_args = false;
                                                    break;
                                                }
                                            }
                                        }
                                        if(equal_args) {
                                            if(e1.positive != e2.positive) {
                                                has_conflict = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if(has_conflict) {
                            break;
                        }
                    }
                    if(has_conflict) {
                        actual_conflicts.push_back(make_pair(t1_decompositions.at(k1),t2_decompositions.at(k2)));
                    }
                }
            }

            /*
                With the actual conflicts found, we have to check the valid decompositions
            */
            if(actual_conflicts.size() > 0) {
                map<int,vector<int>> found_task_decompositions;
                for(pair<pair<int,ATNode>,pair<int,ATNode>> conflict : actual_conflicts) {
                    found_task_decompositions[t1.first].push_back(conflict.first.first);
                    found_task_decompositions[t2.first].push_back(conflict.second.first);

                    // If all the decompositions of a task are in conflict, throw an error
                    if(found_task_decompositions[t1.first].size() == task_decompositions_number[t1.first]) {
                        string conflict_error = "Cannot solve conflicts with task " + get<AbstractTask>(mission_decomposition[t1.first].content).id + ": " + get<AbstractTask>(mission_decomposition[t1.first].content).name; 
                        throw std::runtime_error(conflict_error);
                    } else if(found_task_decompositions[t2.first].size() == task_decompositions_number[t2.first]) {
                        string conflict_error = "Cannot solve conflicts with task " + get<AbstractTask>(mission_decomposition[t2.first].content).id + ": " + get<AbstractTask>(mission_decomposition[t2.first].content).name; 
                        throw std::runtime_error(conflict_error);
                    }

                    /*
                        Delete every mission decomposition that contain these conflicting task instances
                    */
                    vector<pair<vector<pair<int,ATNode>>,vector<ground_literal>>>::iterator mission_it;
                    for(mission_it = valid_mission_decompositions.begin(); mission_it != valid_mission_decompositions.end(); ) {
                        pair<bool,bool> found_instances = make_pair(false,false);
                        bool remove_decomposition = false;
                        for(pair<int,ATNode> t : mission_it->first) {
                            if(t.first == conflict.first.first) {
                                found_instances.first = true;
                            } else if(t.first == conflict.second.first) {
                                found_instances.second = true;
                            }

                            if(found_instances.first && found_instances.second) {
                                remove_decomposition = true;
                                break;
                            }
                        }

                        if(remove_decomposition) {
                            valid_mission_decompositions.erase(mission_it);
                        } else {
                            ++mission_it;
                        }
                    }
                }
            }
        }
    }
}