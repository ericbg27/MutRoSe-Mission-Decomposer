#include <iostream>
#include <set>
#include <variant>

#include "tdg.hpp"
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

/*
    Function: TDG
    Objective: Constructor for the TDG object. Here we generate a TDG and deal with the cycles 
    problem only when generating possible decompositions

    @ Input 1: The root abstract task of the TDG, which needs to be decomposed
    @ Input 2: The abstract tasks defined in HDDL
    @ Input 3: The primitive tasks defined in HDDL
    @ Input 4: The methods defined in HDDL
    @ Output: void. The TDG object
*/ 
TDG::TDG(task root_abstract_task, vector<task> a_tasks, vector<task> p_tasks, vector<method> ms) {
    abstract_tasks = a_tasks;
    primitive_tasks = p_tasks;
    methods = ms;

    NodeData n;

    n.type = AT;
    n.t = root_abstract_task;
    n.parent = -1;

    vertex_t id = boost::add_vertex(n,tdg);

    root = id;
    tdg[id].id = id;
    n.id = id;

    add_task_path(n);
}

/*
    Function: retrieve possible decompositions
    Objective: Find all possible decompositions of the root abstract task

    @ Output: A vector of the possible decompositions of the task. One must note that 
    these are not necessarily valid in the current world state

    NOTE: Need to deal with cycles. Checking predicates is needed.
        - For each path starting in method m1 we need to have as initial state of the world the methods preconditions (if any).
            * If the method originating some path does not contain preconditions, we assume nothing about what was not told to us. This way
            if we find some predicate in action preconditions, we assume them to be true if they were not set by previous actions effects, since
            this means no harm because we are not generating a plan (just decomposing the task)
*/ 
vector<vector<task>> TDG::retrieve_possible_decompositions() {
    vector<int> depth_first_nodes = DFS_visit();

    vector<vector<task>> paths;

    vector<pair<string,string>> initial_vars = tdg[root].t.vars;
    vector<literal> world_state; //Empty since root is an AT and doesn't introduce preconditions and effects
    vector<pair<string,string>> variable_mapping; //Empty since we only have original root vars

    paths = decomposition_recursion(depth_first_nodes,0, initial_vars, world_state, variable_mapping);

    return paths;
}

/*
    Function: decomposition_recursion
    Objective: Recursively generate the decomposition based on a previously generated DFS nodes vector. World state is
    create based on previous preconditions and effects and variables are mapped accordingly to the original root abstract
    task variables.

    @ Input 1: The vector containing the TDG nodes visited using DFS
    @ Input 2: An index to the current element in the vector being considered
    @ Input 3: The original variables from the original root task
    @ Input 4: The world state generated until the moment
    @ Input 5: The variable mappings with respect to the original root abstract task variables
    @ Output: A vector of the decompositions generated at the current recursion depth

    NOTE: Every variable involved in the recursion must come from the root task
            - In this sense, variable substition must happen 
            - Inside every method we know which method variable refers to which original variable
            - Inside every action we also know which variable refers to which original variable
*/ 
vector<vector<task>> TDG::decomposition_recursion(vector<int> dfs_nodes, int current_pos, vector<pair<string,string>> original_vars, 
                                                    vector<literal>& world_state, vector<pair<string,string>> variable_mapping) {
    int node = dfs_nodes.at(current_pos);

    NodeData n = tdg[node];

    /*if(n.type != M) {
        cout << "TASK: " << n.t.name << endl;
    } else {
        cout << "METHOD: " << n.m.name << endl;
    }

    cout << "WORLD STATE" << endl;
    for(literal state : world_state) {
        if(state.positive) {
            cout << state.predicate << " ";
        } else {
            cout << "not " << state.predicate << " ";
        }
        for(string arg : state.arguments) {
            cout << arg << " ";
        }
        cout << endl;
    }
    cout << endl;*/

    //getchar();

    vector<vector<task>> generated_paths;
    if(n.type == M) {
        /*
            If any task in the methods subtasks cannot be executed, the method can't be used.
                - This needs checking in order to skip
                - This only works if we have an and decomposition of subtasks (we are assuming we only have and)
                - Also, we are assuming totally ordered methods
        */
        vector<vector<int>> possible_orderings = find_method_possible_orderings(n.m,n.children);

        print_method_possible_orderings(possible_orderings, n);

        //bool method_exec = true;
        vector<vector<vector<task>>> child_paths;
        for(vector<int> ordering : possible_orderings) {
            bool ordering_exec = true;
            for(int c : ordering) {
                vector<literal> world_state_copy = world_state;
                vector<int>::iterator it = std::find(dfs_nodes.begin(),dfs_nodes.end(),c);
                int c_pos = std::distance(dfs_nodes.begin(),it);

                task child_task = tdg[c].t;
                vector<pair<string,string>> child_var_mapping;
                for(auto subtask : n.m.ps) {
                    if(subtask.task == child_task.name) {
                        int index = 0;
                        for(string var : subtask.args) {
                            string aux;
                            for(auto mapping : variable_mapping) {
                                if(mapping.first == var) { //Found mapping from methods
                                    aux = mapping.second;
                                    child_var_mapping.push_back(make_pair(child_task.vars.at(index).first,aux));
                                    break;
                                }
                            }

                            index++;
                        }

                        break;
                    }
                }

                /*
                    If we have an action, check for preconditions based on the current world state and only call decomposition if they are met
                */
                        
                bool executable;
                executable = check_predicates(child_task,child_var_mapping,c,world_state_copy);

                if(executable) {
                    vector<vector<task>> aux = decomposition_recursion(dfs_nodes,c_pos,original_vars,world_state_copy,child_var_mapping);
                    child_paths.push_back(aux);
                } else {
                    ordering_exec = false;
                    break;
                }
            }

            vector<vector<task>> ordering_paths;
            if(ordering_exec) {
                for(auto aux : child_paths) {
                    vector<vector<task>> g_paths_temp = ordering_paths;
                    ordering_paths.clear();
                    for(auto p : aux) {
                        if(g_paths_temp.size() > 0) {
                            for(auto g_pt : g_paths_temp) {
                                vector<task> p_temp = p;
                                p_temp.insert(p_temp.begin(),g_pt.begin(),g_pt.end());
                                ordering_paths.push_back(p_temp);
                            }
                        } else {
                            ordering_paths.push_back(p);
                        }
                    }
                }
            }

            generated_paths.insert(generated_paths.end(),ordering_paths.begin(),ordering_paths.end());

            child_paths.clear();
        }

        /*if(method_exec) {
            for(auto aux : child_paths) {
                vector<vector<task>> g_paths_temp = generated_paths;
                generated_paths.clear();
                for(auto p : aux) {
                    if(g_paths_temp.size() > 0) {
                        for(auto g_pt : g_paths_temp) {
                            vector<task> p_temp = p;
                            p_temp.insert(p_temp.begin(),g_pt.begin(),g_pt.end());
                            generated_paths.push_back(p_temp);
                        }
                    } else {
                        generated_paths.push_back(p);
                    }
                }
            }
        }*/
    } else if(n.type == AT) {
        vector<literal> initial_world_state = world_state;
        for(int c : n.children) {
            vector<int>::iterator it = std::find(dfs_nodes.begin(),dfs_nodes.end(),c);
            int c_pos = std::distance(dfs_nodes.begin(),it);

            method child_method = tdg[c].m;
            vector<pair<string,string>> child_var_mapping;
            if(n.id == root) { //If we are dealing with the root
                int index = 0;
                for(string arg : child_method.atargs) {
                    child_var_mapping.push_back(make_pair(arg,original_vars.at(index).first));
                    index++; 
                }
            } else {
                int index = 0;
                for(string arg : child_method.atargs) {
                    child_var_mapping.push_back(make_pair(arg,variable_mapping.at(index).second)); //Mapping to original vars
                }
            }

            /*
                Methods preconditions are transformed into actions, so we only check for preconditions when
                checking methods children
            */

            vector<vector<task>> aux = decomposition_recursion(dfs_nodes,c_pos,original_vars,world_state,child_var_mapping);
            generated_paths.insert(generated_paths.end(),aux.begin(),aux.end());

            world_state = initial_world_state;
        }
    } else if(n.type == PT) {
        change_world_state(n.t,world_state,variable_mapping);
        vector<task> t;
        task nt = n.t;
        variable_renaming(nt,variable_mapping);
        t.push_back(nt);
        generated_paths.push_back(t);
    }

    return generated_paths;
}

/*
    Function: add_method_path
    Objective: Add method path in TDG by iterating through method decomposition tasks
    and further adding these tasks paths to the TDG

    @ Input: The method node being considered
    @ Output: void. The path will be added to the TDG object calling the function
*/ 
void TDG::add_method_path(NodeData m) {
    for(plan_step ps : m.m.ps) {
        NodeData t_node;
        task n_task;

        bool primitive = true;

        for(task at : abstract_tasks) {
            if(at.name == ps.task) {
                primitive = false;
                n_task = at;
                break;
            }
        }

        if(primitive) {
            for(task pt : primitive_tasks) {
                if(pt.name == ps.task) {
                    n_task = pt;
                    break;
                }
            }
            t_node.type = PT;
        } else {
            t_node.type = AT;
        }

        t_node.t = n_task;

        pair<bool,int> cycle = make_pair(false,-1);
        if(!primitive) {
            cycle = check_cycle(m.id,t_node);
        }

        if(!cycle.first) {
            vertex_t id = boost::add_vertex(t_node,tdg);

            tdg[id].id = id;

            t_node.id = id;

            add_edge(m.id, tdg[id].id);

            add_task_path(t_node);
        } else {
            add_edge(m.id, cycle.second);
        }
    }
}

/*
    Function: add_task_path
    Objective: Add a task (primitive or abstract) to the TDG. Find methods that decompose the task 
    and from them generate the complete path resulting from their decomposition

    @ Input: The task node being considered
    @ Output: void. The path will be added to the TDG object calling the function
*/ 
void TDG::add_task_path(NodeData t) {
    if(t.type != PT) {
        for(method m : methods) {
            if(m.at == t.t.name) {
                NodeData m_node;
                m_node.type = M;
                m_node.m = m;
                
                vertex_t id = boost::add_vertex(m_node,tdg);

                tdg[id].id = id;

                m_node.id = id;

                add_edge(t.id, tdg[id].id);

                add_method_path(m_node);
            }
        }
    }
}

/*
    Function: add_edge
    Objective: Add an edge between TDG nodes

    @ Input 1: The soruce node ID
    @ Input 2: The target node ID
    @ Output: void. The edge will be added in the TDG
*/ 
void TDG::add_edge(int s_id, int t_id) {
    EData edge;

    edge.source = s_id;
    edge.target = t_id;

    if(tdg[s_id].type == M) {
        edge.type = OOR;
    } else {
        edge.type = AAND;
    }

    tdg[s_id].children.push_back(t_id);
    tdg[t_id].parent = s_id;

     boost::add_edge(s_id,t_id,edge,tdg);
}

/*
    Function: DFS_visit
    Objective: Perform a Depth-First visit in the TDG and return a vector of node ID's
    generated through this visit

    @ Output: The vector of node ID's
*/ 
vector<int> TDG::DFS_visit() {
    auto indexmap = boost::get(boost::vertex_index, tdg);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    TDGDFSVisitor vis;
    boost::depth_first_search(tdg, vis, colormap, 0);

    vector<int> vctr = vis.GetVector();

    return vctr;
}

/*
    Function: print_edges
    Objective: Print TDG edges

    @ Output: void. The edges will be printed in a terminal
*/ 
void TDG::print_edges() {
    boost::graph_traits<TDGraph>::edge_iterator it, end;
	
	for(tie(it,end) = boost::edges(tdg);it != end;++it) {
        NodeData s = tdg[boost::source(*it, tdg)];
        NodeData t = tdg[boost::target(*it, tdg)];

        if(s.type != M) {
		    std::cout << s.t.name << " -> ";
        } else {
            std::cout << s.m.name << " -> ";
        }

        if(t.type != M) {
		    std::cout << t.t.name << endl;
        } else {
            std::cout << t.m.name << endl;
        }
	}
}

/*
    Function: print_method_possible_orderings
    Objective: Print all the possible orderings for a method

    @ Input 1: The possible orderings for the method
    @ Input 2: The method node being considered 
    @ Output: void. The possible orderings will be printed in a terminal
*/ 
void TDG::print_method_possible_orderings(vector<vector<int>> possible_orderings, NodeData n) {
    cout << "Possible orderings for method " << n.m.name << ":" << endl;
    for(vector<int> ordering : possible_orderings) {
        for(int t : ordering) {
            cout << tdg[t].t.name << " ";
        }
        cout << endl;
    }
}

/*
    Function: add_task_path
    Objective: Here we have a method that is already in the graph and a task that is to be introduced to the graph
    if it is not already in it. In case it is, return true since we have a cycle.

    @ Input 1: The method ID in the TDG
    @ Input 2: The task node in the TDG
    @ Output: A pair containing a flag if we have a cycle or not and the ID of the task to which this cycle refers to.

    NOTE: Task t must be a parent in some degree of m in order for us to consider a cycle
*/ 
pair<bool,int> TDG::check_cycle(int m_id, NodeData t) {
    pair<bool,int> cycle = make_pair(false,-1);
    NodeData m = tdg[m_id];

    NodeData current_node = m;
    bool at_root = false;
    while(at_root == false) {
        if(current_node.parent == -1) at_root = true;

        if(current_node.type == AT) {
            if(current_node.t.name == t.t.name) {
                current_node.belongs_to_cycles = true;
                current_node.cycle_links.push_back(m.id);
                cycle.first = true;
                cycle.second = current_node.id;

                break;
            }
        }

        if(current_node.parent != -1) {
            current_node = tdg[current_node.parent];
        }
    }

    return cycle;
}

/*
    Function: check_predicates
    Objective: Here we check if all predicates for a task hold in the current world state. One must note that since
    the world state is generated based on the root abstract task of the TDG and we must rename variables.

    @ Input 1: The task to be evaluated
    @ Input 2: The variable mapping between the task and the TDG's root abstract task
    @ Input 3: The task ID in the TDG
    @ Input 4: The world state
    @ Output: A boolean flag indicating if predicates hold
*/ 
bool TDG::check_predicates(task t, vector<pair<string,string>> var_mapping, int t_id, vector<literal>& world_state) {
    vector<literal> t_precs, precs_to_add;

    t_precs = t.prec;
    /*cout << "Task " << t.name << " preconditions: " << endl;
    for(auto prec : t_precs) {
        if(!prec.positive) {
            cout << "not ";
        }
        cout << prec.predicate << " ";
        for(auto arg : prec.arguments) {
            cout << arg << " ";
        }
        cout << endl;
    }*/

    //Rename predicates variables
    for(literal& prec : t_precs) {
        for(string& arg : prec.arguments) {
            for(pair<string,string>& arg_mapping : var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    /*cout << "Task " << t.name << " (renamed) preconditions: " << endl;
    for(auto prec : t_precs) {
        if(!prec.positive) {
            cout << "not ";
        }
        cout << prec.predicate << " ";
        for(auto arg : prec.arguments) {
            cout << arg << " ";
        }
        cout << endl;
    }*/

    bool executable = true;
    if(tdg[t_id].type == PT) {
        for(literal& prec : t_precs) {
            bool found_prec = false;
            for(literal& state : world_state) {
                bool args_equal = false;
                if(state.predicate == prec.predicate) {
                    int i = 0;
                    bool diff = false;
                    for(auto arg : state.arguments) {
                        if(arg != prec.arguments.at(i)) {
                            diff = true;
                            break;
                        }

                        i++;
                    }
                    if(!diff) {
                        args_equal = true;
                    }
                }

                if(args_equal) { //Dealing with same predicate with same arguments
                    found_prec = true;
                    if(!((prec.positive && state.positive) || (!prec.positive && !state.positive))) {
                        executable = false;
                    }

                    break;
                }
            }

            if(!found_prec) {
                precs_to_add.push_back(prec);
            }
        }

        if(executable) {
            for(literal prec : precs_to_add) {
                world_state.push_back(prec);
            }
        }
    }

    return executable;
}

/*
    Function: change_world state
    Objective: Here we change the world state based on the effects of a task. We must consider the variable mapping
    with respect to the TDG's root abstract task

    @ Input 1: The task to be evaluated
    @ Input 2: A reference to the world state
    @ Input 3: The variable mapping with respect to the TDG's root abstract task
    @ Output: Void. The reference to the world state is changed
*/ 
void TDG::change_world_state(task t,vector<literal>& world_state, vector<pair<string,string>> variable_mapping) {
    vector<literal> t_effs = t.eff;

    //Rename predicates variables
    for(literal& prec : t_effs) {
        for(string& arg : prec.arguments) {
            for(pair<string,string>& arg_mapping : variable_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    for(literal& eff : t_effs) {
        bool found_pred = false;
        vector<literal>::iterator state;
        for(state = world_state.begin();state != world_state.end();++state) {
            bool args_equal = false;
            if(state->predicate == eff.predicate) {
                int i = 0;
                bool diff = false;
                for(string& arg : state->arguments) {
                    if(arg != eff.arguments.at(i)) {
                        diff = true;
                        break;
                    }

                    i++;
                }
                if(!diff) {
                    args_equal = true;
                }
            }

            if(args_equal) { //Dealing with same predicate with same arguments
                found_pred = true;
                if(((eff.positive && !state->positive) || (!eff.positive && state->positive))) {
                    state->positive = eff.positive;
                }

                break;
            }
        }

        if(!found_pred) {
            world_state.push_back(eff);
        }
    }
}

/*
    Function: variable_renaming
    Objective: Rename variables of a specific task

    @ Input 1: A reference to the task being considered
    @ Input 2: The variable mapping between the task and the TDG's root abstract task
    @ Output: Void. The reference to the task is modified
*/ 
void TDG::variable_renaming(task& t, vector<pair<string,string>> var_mapping) {
    //Rename preconditions
    for(literal& prec : t.prec) {
        for(string& arg : prec.arguments) {
            for(pair<string,string>& arg_mapping : var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    //Rename effects
    for(literal& eff : t.eff) {
        for(string& arg : eff.arguments) {
            for(pair<string,string>& arg_mapping : var_mapping) {
                if(arg_mapping.first == arg) {
                    arg = arg_mapping.second;
                    break;
                }
            }
        }
    }

    //Rename task variables
    for(pair<string,string>& var : t.vars) {
        for(pair<string,string>& arg_mapping : var_mapping) {
            if(arg_mapping.first == var.first) {
                var.first = arg_mapping.second;
                break;
            }
        }
    }

    /*cout << "Renamed Task: " << t.name << endl;
    cout << "Vars: ";
    for(auto var : t.vars) {
        cout << var.first << " - " << var.second << " ";
    }
    cout << endl;
    cout << "Preconditions: " << endl;
    for(auto prec : t.prec) {
        if(!prec.positive) {
            cout << "not ";
        }
        cout << prec.predicate << " ";
        for(auto arg : prec.arguments) {
            cout << arg << " ";
        }
        cout << endl;
    }
    cout << "Effects: " << endl;
    for(auto eff : t.eff) {
        if(!eff.positive) {
            cout << "not ";
        }
        cout << eff.predicate << " ";
        for(auto arg : eff.arguments) {
            cout << arg << " ";
        }
        cout << endl;
    }
    cout << endl;*/
}

/*
    Function: find_method_possible_orderings
    Objective: Find the possible orderings for a method's decomposition

    @ Input 1: The method being considered
    @ Input 2: The ID's of this method's children nodes in the TDG
    @ Output: The vector of the possible orderings by means of task ID's
*/ 
vector<vector<int>> TDG::find_method_possible_orderings(method m, vector<int> children) {
    map<string,int> plan_step_id_map;
    for(plan_step ps : m.ps) {
        for(int c : children) {
            if(tdg[c].t.name == ps.task) {
                plan_step_id_map[ps.id] = c;
                break;
            }
        }
    }

    map<int,set<int>> precedences_map;
    for(pair<string,string> o : m.ordering) {
        if(precedences_map.find(plan_step_id_map[o.first]) == precedences_map.end()) {
            set<int> precedence;
            precedence.insert(plan_step_id_map[o.second]);
            precedences_map[plan_step_id_map[o.first]] = precedence;
        } else {
            precedences_map[plan_step_id_map[o.first]].insert(plan_step_id_map[o.second]);
        }
    }

    vector<vector<int>> possible_orderings;

    set<int> children_set;
    for(int c : children) {
        children_set.insert(c);
    }

    possible_orderings = recursive_method_possible_ordering(precedences_map, possible_orderings, children_set);

    return possible_orderings;
}

/*
    Function: recursive_method_possible_orderings
    Objective: Recursive generations of a method's possible orderings of decomposition

    @ Input 1: The map of precendence constraints given in HDDL
    @ Input 2: The orderings generated so far
    @ Input 3: The task ID's to be inserted
    @ Output: The possible orderings generated in this depth of the recursion
*/ 
vector<vector<int>> TDG::recursive_method_possible_ordering(map<int,set<int>> precedence_map, vector<vector<int>> current_orderings, set<int> values_to_insert) {
    vector<vector<int>> new_orderings;

    if(values_to_insert.size() > 0) {
        for(int val : values_to_insert) {
            bool can_insert_value = true;
            map<int,set<int>>::iterator precedences_iterator;
            for(precedences_iterator = precedence_map.begin();precedences_iterator != precedence_map.end();++precedences_iterator) {
                if(precedences_iterator->first != val) {
                    /*
                        If we have a value that is involved in a precedence constraint with the current value being checked and it was not inserted we
                        cannot insert the current value yet
                    */
                    if(values_to_insert.find(precedences_iterator->first) != values_to_insert.end()) {
                        if(precedences_iterator->second.find(val) != precedences_iterator->second.end()) {
                            can_insert_value = false;
                            break;
                        }
                    }
                }
            }

            if(can_insert_value) {
                set<int> updated_values_to_insert = values_to_insert;
                updated_values_to_insert.erase(val);

                vector<vector<int>> updated_current_orderings = current_orderings;

                if(current_orderings.size() > 0) {
                    for(vector<int>& ordering : updated_current_orderings) {
                        ordering.push_back(val);
                    }
                } else {
                    vector<int> first_ordering;
                    first_ordering.push_back(val);
                    updated_current_orderings.push_back(first_ordering);
                }

                vector<vector<int>> aux = recursive_method_possible_ordering(precedence_map,updated_current_orderings,updated_values_to_insert);
                for(vector<int> o : aux) {
                    new_orderings.push_back(o);
                }
            }
        }
    } else {
        new_orderings = current_orderings;
    }

    return new_orderings;
}