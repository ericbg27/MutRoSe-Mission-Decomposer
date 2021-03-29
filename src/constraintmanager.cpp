#include "constraintmanager.hpp"

using namespace std;

/*
    Function: generate_at_constraints
    Objective: Generate all mission constraints, including parallel ones

    @ Input: The task graph as an ATGraph object
    @ Output: The vector with all of the mission constraints
*/
vector<Constraint> generate_at_constraints(ATGraph mission_decomposition, queue<pair<int,ATNode>> mission_queue) {
    stack<pair<int,ATNode>> operators_stack;
    stack<variant<pair<int,ATNode>,Constraint>> nodes_stack;

    vector<Constraint> mission_constraints;
    map<int,set<int>> existing_constraints;

    /*
        Nodes with id -1 are artificial just to simulate the division between branches of the tree
    */
    string last_op = "";

    while(!mission_queue.empty()) {
        pair<int,ATNode> current_node = mission_queue.front();
        mission_queue.pop();

        if(current_node.second.node_type == ATASK) {
            nodes_stack.push(current_node);
        
            if(operators_stack.size() > 1 && nodes_stack.size() >= 2) {
                bool new_branch = false;
                
                /*
                    Check if we have at least 2 nodes until we reach some artificial node
                */
                stack<variant<pair<int,ATNode>,Constraint>> nodes_stack_cpy = nodes_stack;
                for(int cnt = 0;cnt < 2;cnt++) {
                    if(holds_alternative<pair<int,ATNode>>(nodes_stack_cpy.top())) {
                        if(get<pair<int,ATNode>>(nodes_stack_cpy.top()).first == -1) {
                            new_branch = true;
                        }
                        nodes_stack_cpy.pop();
                    }
                }

                if(new_branch) {
                    last_op = "";
                } else {
                    /*
                        Here is the logic for creating a constraint

                        -> Idea: Go through the stack and
                            - If we find an AT we will get it and use it to form new constraint(s)
                            - If we find a constraint we don't erase it from the stack but:
                                - If we have a sequential operator we just take into consideration the most recent constraint
                                - If we have a parallel operator we take into consideration all of the constraints we have until reaching one that has id -1 or the
                                first created constraint
                    */
                    if(get<string>(operators_stack.top().second.content) == "#") {
                        /*
                            While we don't reach an artificial node or the end of the nodes stack we populate our temporary vector
                        */
                        queue<variant<pair<int,ATNode>,Constraint>> temp;
                        bool end_reached = false;
                        while(!end_reached) {
                            if(holds_alternative<pair<int,ATNode>>(nodes_stack.top())) {
                                if(get<pair<int,ATNode>>(nodes_stack.top()).first == -1) {
                                    end_reached = true;
                                }
                            }

                            if(!end_reached) {
                                temp.push(nodes_stack.top());
                                nodes_stack.pop();
                            }

                            if(nodes_stack.empty()) {
                                end_reached = true;
                            }
                        }

                        /*
                            Using the temporary vector, generate the constraints, delete AT's and generate the new constraints

                            -> Remember that we will have an AT in the first position of the queue and we will generate constraints using it 
                            and all of the constraints (or another AT) we already have
                        */
                        pair<int,ATNode> last_task = get<pair<int,ATNode>>(temp.front());
                        temp.pop();

                        stack<Constraint> new_constraints;
                        stack<Constraint> old_constraints;

                        while(!temp.empty()) {
                            if(holds_alternative<pair<int,ATNode>>(temp.front())) {
                                /*
                                    We are dealing with a task, so we need to build a constraint with it
                                */
                                Constraint c;
                                c.type = PAR;
                                c.nodes_involved = make_pair(get<pair<int,ATNode>>(temp.front()),last_task);

                                existing_constraints[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);

                                new_constraints.push(c);
                                temp.pop();
                            } else {
                                /*
                                    We are dealing with a constraint. Since we are dealing with a parallel operator we will create parallel constraints with both of
                                    the tasks
                                */
                                Constraint c1, c2;

                                pair<pair<int,ATNode>,pair<int,ATNode>> constraint_nodes = get<Constraint>(temp.front()).nodes_involved;
                                c1.type = PAR;
                                c1.nodes_involved = make_pair(constraint_nodes.first, last_task);

                                c2.type = PAR;
                                c2.nodes_involved = make_pair(constraint_nodes.second, last_task);

                                if(existing_constraints[constraint_nodes.second.first].find(last_task.first) == existing_constraints[constraint_nodes.second.first].end()) {
                                    existing_constraints[constraint_nodes.second.first].insert(last_task.first);
                                    new_constraints.push(c2);
                                }

                                if(existing_constraints[constraint_nodes.first.first].find(last_task.first) == existing_constraints[constraint_nodes.first.first].end()) {
                                    existing_constraints[constraint_nodes.first.first].insert(last_task.first);
                                    new_constraints.push(c1);
                                }

                                old_constraints.push(get<Constraint>(temp.front()));
                                temp.pop();
                            }
                        }

                        while(!old_constraints.empty()) {
                            nodes_stack.push(old_constraints.top());
                            old_constraints.pop();
                        }

                        while(!new_constraints.empty()) {
                            nodes_stack.push(new_constraints.top());
                            new_constraints.pop();
                        }

                        last_op = "#";
                    } else {
                        if(last_op == "#") {
                            /*
                                If the last operator was parallel, we need to create constraints with all of the constraints (and their tasks) until we reach a sequential
                                constraint, where we will create a constraint with the last involved task
                            */
                            queue<variant<pair<int,ATNode>,Constraint>> temp;
                            bool end_reached = false;
                            while(!end_reached) {
                                if(holds_alternative<pair<int,ATNode>>(nodes_stack.top())) {
                                    if(get<pair<int,ATNode>>(nodes_stack.top()).first == -1) {
                                        end_reached = true;
                                    }
                                } else {
                                    if(get<Constraint>(nodes_stack.top()).type == SEQ) {
                                        end_reached = true;
                                        temp.push(nodes_stack.top());
                                        nodes_stack.pop();
                                    }
                                }

                                if(!end_reached) {
                                    temp.push(nodes_stack.top());
                                    nodes_stack.pop();
                                }

                                if(nodes_stack.empty()) {
                                    end_reached = true;
                                }
                            }

                            pair<int,ATNode> last_task = get<pair<int,ATNode>>(temp.front());
                            temp.pop();

                            stack<Constraint> new_constraints;
                            stack<Constraint> old_constraints;

                            while(!temp.empty()) {
                                if(holds_alternative<pair<int,ATNode>>(temp.front())) {
                                    /*
                                        We are dealing with a task, so we need to build a constraint with it
                                    */
                                    Constraint c;
                                    c.type = SEQ;
                                    c.nodes_involved = make_pair(get<pair<int,ATNode>>(temp.front()),last_task);

                                    existing_constraints[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);

                                    new_constraints.push(c);
                                    temp.pop();
                                } else {
                                    if(get<Constraint>(temp.front()).type == PAR) {
                                        Constraint c1, c2;

                                        pair<pair<int,ATNode>,pair<int,ATNode>> constraint_nodes = get<Constraint>(temp.front()).nodes_involved;
                                        c1.type = SEQ;
                                        c1.nodes_involved = make_pair(constraint_nodes.first, last_task);

                                        c2.type = SEQ;
                                        c2.nodes_involved = make_pair(constraint_nodes.second, last_task);

                                        if(existing_constraints[constraint_nodes.second.first].find(last_task.first) == existing_constraints[constraint_nodes.second.first].end()) {
                                            existing_constraints[constraint_nodes.second.first].insert(last_task.first);
                                            new_constraints.push(c2);
                                        }

                                        if(existing_constraints[constraint_nodes.first.first].find(last_task.first) == existing_constraints[constraint_nodes.first.first].end()) {
                                            existing_constraints[constraint_nodes.first.first].insert(last_task.first);
                                            new_constraints.push(c1);
                                        }

                                        old_constraints.push(get<Constraint>(temp.front()));
                                        temp.pop();
                                    } else {
                                        Constraint c1, c2;

                                        pair<pair<int,ATNode>,pair<int,ATNode>> constraint_nodes = get<Constraint>(temp.front()).nodes_involved;

                                        c2.type = SEQ;
                                        c2.nodes_involved = make_pair(constraint_nodes.second, last_task);

                                        if(existing_constraints[constraint_nodes.second.first].find(last_task.first) == existing_constraints[constraint_nodes.second.first].end()) {
                                            existing_constraints[constraint_nodes.second.first].insert(last_task.first);
                                            new_constraints.push(c2);
                                        }

                                        old_constraints.push(get<Constraint>(temp.front()));
                                        temp.pop();
                                    }
                                }
                            }

                            while(!old_constraints.empty()) {
                                nodes_stack.push(old_constraints.top());
                                old_constraints.pop();
                            }

                            while(!new_constraints.empty()) {
                                nodes_stack.push(new_constraints.top());
                                new_constraints.pop();
                            }

                        } else if(last_op == ";") {
                            /*
                                If the last operator was sequential, we just need to create a constraint with the last constraint in the stack

                                -> If we create a constraint with another constraint, we just need to create with the second involved task
                            */
                            pair<int,ATNode> last_task = get<pair<int,ATNode>>(nodes_stack.top());
                            nodes_stack.pop();

                            Constraint new_constraint;
                            Constraint last_constraint = get<Constraint>(nodes_stack.top());

                            new_constraint.type = SEQ;
                            new_constraint.nodes_involved = make_pair(last_constraint.nodes_involved.second,last_task);

                            nodes_stack.push(new_constraint);
                            existing_constraints[last_constraint.nodes_involved.second.first].insert(last_task.first);
                        } else { 
                            /*
                                If no operator was considered yet, we need to create a constraint with another task
                            */
                            Constraint c;

                            pair<int,ATNode> last_task = get<pair<int,ATNode>>(nodes_stack.top());
                            nodes_stack.pop();

                            pair<int,ATNode> other_task = get<pair<int,ATNode>>(nodes_stack.top());
                            nodes_stack.pop();

                            c.type = SEQ;
                            c.nodes_involved = make_pair(other_task, last_task);

                            nodes_stack.push(c);
                            existing_constraints[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);
                        }

                        last_op = ";";
                    }

                    operators_stack.pop();
                }
            }
        } else {
            if(operators_stack.size() == 1 && nodes_stack.size() > 0) {
                pair<int,ATNode> artificial_node;
                artificial_node.first = -1;

                nodes_stack.push(artificial_node);
            }
            operators_stack.push(current_node);
        }
    }

    stack<variant<pair<int,ATNode>,Constraint>> nodes_cpy = nodes_stack;

    /*
        Here we will have only one operator and several constraints, which we must combine to have all of the constraints of the mission
    */
    pair<int,ATNode> final_operator = operators_stack.top();
    operators_stack.pop();

    queue<Constraint> final_constraints;

    stack<variant<pair<int,ATNode>,Constraint>> nodes_stack_copy = nodes_stack;
    while(!nodes_stack_copy.empty()) {
        if(holds_alternative<Constraint>(nodes_stack_copy.top())) {
            Constraint c = get<Constraint>(nodes_stack_copy.top());

            mission_constraints.insert(mission_constraints.begin(),c);
        }

        nodes_stack_copy.pop();
    }

    if(get<string>(final_operator.second.content) == "#") {
        /*
            If we have a parallel operator we need to create parallel constraints for all tasks that are not in the existing constraints map
        */
        vector<vector<Constraint>> constraint_branches;
        vector<Constraint> aux;
        while(!nodes_stack.empty()) {
            if(holds_alternative<pair<int,ATNode>>(nodes_stack.top())) {
                //Artificial node
                constraint_branches.push_back(aux);
                aux.clear();
            } else {
                aux.push_back(get<Constraint>(nodes_stack.top()));
            }

            nodes_stack.pop();

            if(nodes_stack.empty()) {
                constraint_branches.push_back(aux);
            }
        }

        unsigned int i,j;
        for(i=0;i<constraint_branches.size()-1;i++) {
            for(j=i+1;j<constraint_branches.size();j++) {
                unsigned int index1, index2;
                vector<Constraint> v1 = constraint_branches.at(i);
                vector<Constraint> v2 = constraint_branches.at(j);

                for(index1=0;index1<v1.size();index1++) {
                    for(index2=0;index2<v2.size();index2++) {
                        Constraint c1,c2;

                        c1 = v1.at(index1);
                        c2 = v2.at(index2);

                        Constraint nc1,nc2,nc3,nc4;

                        nc1.type = PAR;
                        nc1.nodes_involved = make_pair(c1.nodes_involved.first,c2.nodes_involved.first);

                        nc2.type = PAR;
                        nc2.nodes_involved = make_pair(c1.nodes_involved.first,c2.nodes_involved.second);

                        nc3.type = PAR;
                        nc3.nodes_involved = make_pair(c1.nodes_involved.second,c2.nodes_involved.first);

                        nc4.type = PAR;
                        nc4.nodes_involved = make_pair(c1.nodes_involved.second,c2.nodes_involved.second);

                        if(existing_constraints[nc1.nodes_involved.first.first].find(nc1.nodes_involved.second.first) == existing_constraints[nc1.nodes_involved.first.first].end()) {
                            final_constraints.push(nc1);
                            existing_constraints[nc1.nodes_involved.first.first].insert(nc1.nodes_involved.second.first);
                        }
                        if(existing_constraints[nc2.nodes_involved.first.first].find(nc2.nodes_involved.second.first) == existing_constraints[nc2.nodes_involved.first.first].end()) {
                            final_constraints.push(nc2);
                            existing_constraints[nc2.nodes_involved.first.first].insert(nc2.nodes_involved.second.first);
                        }
                        if(existing_constraints[nc3.nodes_involved.first.first].find(nc3.nodes_involved.second.first) == existing_constraints[nc3.nodes_involved.first.first].end()) {
                            final_constraints.push(nc3);
                            existing_constraints[nc3.nodes_involved.first.first].insert(nc3.nodes_involved.second.first);
                        }
                        if(existing_constraints[nc4.nodes_involved.first.first].find(nc4.nodes_involved.second.first) == existing_constraints[nc4.nodes_involved.first.first].end()) {
                            final_constraints.push(nc4);
                            existing_constraints[nc4.nodes_involved.first.first].insert(nc4.nodes_involved.second.first);
                        }
                    }
                }
            }
        }
    } else if(get<string>(final_operator.second.content) == ";") {
        /*
            If we have a sequential operator, we just need to create additional constraints between the constraints that are between artificial nodes
        */
        vector<vector<Constraint>> constraint_branches;
        vector<Constraint> aux;
        while(!nodes_stack.empty()) {
            if(holds_alternative<pair<int,ATNode>>(nodes_stack.top())) {
                //Artificial node
                constraint_branches.push_back(aux);
                aux.clear();
            } else {
                aux.push_back(get<Constraint>(nodes_stack.top()));
            }

            nodes_stack.pop();

            if(nodes_stack.empty()) {
                constraint_branches.push_back(aux);
            }
        }

        unsigned int i;
        for(i=0;i<constraint_branches.size()-1;i++) {
            Constraint c1, c2;

            c1 = constraint_branches.at(i).back();
            c2 = constraint_branches.at(i+1).front();

            Constraint c;

            c.type = SEQ;
            c.nodes_involved = make_pair(c1.nodes_involved.second,c2.nodes_involved.first);

            if(existing_constraints[c.nodes_involved.first.first].find(c.nodes_involved.second.first) == existing_constraints[c.nodes_involved.first.first].end()) {
                final_constraints.push(c);
            }
        }
    }

    queue<Constraint> final_constraints_copy = final_constraints;
    
    while(!final_constraints.empty()) {
        mission_constraints.push_back(final_constraints.front());

        final_constraints.pop();
    }

    return mission_constraints; 
}

/*
    Function: transform_at_constraints
    Objective: Here we will create the final constraints involving the decompositions and not the abstract tasks

    @ Input 1: The Task Graph as an ATGraph object
    @ Input 2: The vector of constraints
    @ Input 3: The goal model as a GMGraph object
    @ Output: The final constraint vector of the mission

    NOTES:  -> For now we will only return the sequential constraints since they define precedence. Parallel constraints are not considered
            -> One note is that parallel constraints where we have Context Dependencies will be transformed into sequential
            -> We have to check for Context Dependencies
                - If we find that a node has Context Dependencies we have to check with who it has
                    * If it has with a high-level node (like a goal) we have to find all of the instances it has this dependency
                    * If a parallel dependency between these tasks exist, change to sequential. If it is already sequential, nothing needs to be done
*/
vector<Constraint> transform_at_constraints(ATGraph mission_decomposition, vector<Constraint> mission_constraints, GMGraph gm) {
    vector<Constraint> transformed_constraints;
    map<int,vector<pair<int,ATNode>>> constraint_nodes_decompositions;

    for(Constraint c : mission_constraints) {
        if(c.type == SEQ) {
            /*
                Check if the nodes involved are present in the decompositions map

                -> After having the decompositions, create constraints combining all of them
            */
            pair<int,ATNode> n1 = c.nodes_involved.first;
            pair<int,ATNode> n2 = c.nodes_involved.second;

            vector<pair<int,ATNode>> n1_decompositions, n2_decompositions;
            
            if(constraint_nodes_decompositions.find(n1.first) != constraint_nodes_decompositions.end()) {
                n1_decompositions = constraint_nodes_decompositions[n1.first];
            } else {
                n1_decompositions = find_decompositions(mission_decomposition,n1.first);
            }

            if(constraint_nodes_decompositions.find(n2.first) != constraint_nodes_decompositions.end()) {
                n2_decompositions = constraint_nodes_decompositions[n2.first];
            } else {
                n2_decompositions = find_decompositions(mission_decomposition,n2.first);

                constraint_nodes_decompositions[n2.first] = n2_decompositions;
            }

            /*
                Verify which decompositions of the first task in which effects prohibit the decompositions of the second task by making its preconditions
                invalid.

                -> We have to check which tasks are involved in a Non-group or Non-divisible group goal
                -> These will involve the same robots
                    - We can assume that these will use the same number of robots
            */
            bool non_coop_nodes = boost::edge(n1.first,n2.first,mission_decomposition).second;
            unsigned int i,j;
            for(i=0;i<n1_decompositions.size();i++) {
                for(j=0;j<n2_decompositions.size();j++) {
                    bool can_unite = can_unite_decompositions(get<Decomposition>(n1_decompositions.at(i).second.content),get<Decomposition>(n2_decompositions.at(j).second.content),non_coop_nodes);

                    if(can_unite) {
                        Constraint new_c;

                        new_c.type = SEQ;
                        new_c.nodes_involved.first = n1_decompositions.at(i);
                        new_c.nodes_involved.second = n2_decompositions.at(j);

                        transformed_constraints.push_back(new_c);
                    }
                }
            }
        } else {
            //Here we check for Context Dependencies
            pair<int,ATNode> n1 = c.nodes_involved.first;
            pair<int,ATNode> n2 = c.nodes_involved.second;

            vector<pair<int,ATNode>> n1_decompositions, n2_decompositions;
            
            if(constraint_nodes_decompositions.find(n1.first) != constraint_nodes_decompositions.end()) {
                n1_decompositions = constraint_nodes_decompositions[n1.first];
            } else {
                n1_decompositions = find_decompositions(mission_decomposition,n1.first);
            }

            if(constraint_nodes_decompositions.find(n2.first) != constraint_nodes_decompositions.end()) {
                n2_decompositions = constraint_nodes_decompositions[n2.first];
            } else {
                n2_decompositions = find_decompositions(mission_decomposition,n2.first);

                constraint_nodes_decompositions[n2.first] = n2_decompositions;
            }

            /*
                The idea here is that for each decomposition of the first task we check:
                    - Does it have a Context Dependency?
                    - If it has, is it with an Abstract Task?
                        - If it is we check if this Abstract Task is the second task
                            - If it is, we create Sequential constraints with all of the decompositions of the second task
                    - If it is not with an Abstract Task, is the second task a child of the node involved in the context dependency?
                        - If it is, we create Sequential constraints with all of the decompositions of the second task
            */
            for(pair<int,ATNode> n1_decomposition : n1_decompositions) {
                ATGraph::out_edge_iterator ei, ei_end;

                int node_id = n1_decomposition.first;
                Decomposition d = get<Decomposition>(n1_decomposition.second.content);

                bool context_dependency_valid = false;
                //Insert only the nodes which are linked by outer edges of type NORMAL
                for(boost::tie(ei,ei_end) = out_edges(node_id,mission_decomposition);ei != ei_end;++ei) {
                    auto source = boost::source(*ei,mission_decomposition);
                    int target = boost::target(*ei,mission_decomposition);
                    auto edge = boost::edge(source,target,mission_decomposition).first;

                    if(mission_decomposition[edge].edge_type == CDEPEND) {
                        //For each Context Dependency, check if it involves the second task
                        if(mission_decomposition[target].node_type == ATASK) {
                            if(target == n2.first) {
                                context_dependency_valid = true;
                                break;
                            }
                        } else {
                            auto indexmap = boost::get(boost::vertex_index, mission_decomposition);
                            auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

                            DFSATVisitor vis;
                            boost::depth_first_search(mission_decomposition, vis, colormap, target);

                            std::vector<int> vctr = vis.GetVector();

                            int current_node = vctr.at(0);
                            vctr.erase(vctr.begin());
                            while(current_node != 0) {
                                if(mission_decomposition[current_node].node_type == ATASK) {
                                    if(current_node == n2.first) {
                                        context_dependency_valid = true;
                                        break;
                                    }
                                }

                                current_node = vctr.at(0);
                                vctr.erase(vctr.begin());
                            }
                        }
                    }

                    if(context_dependency_valid) {
                        for(pair<int,ATNode> n2_decomposition : n2_decompositions) {
                            Constraint c;

                            c.type = SEQ;
                            c.nodes_involved.first = n1_decomposition;
                            c.nodes_involved.second = n2_decomposition;

                            transformed_constraints.push_back(c);
                        }

                        break;
                    }
                }
            }
        }
    }

    std::cout << std::endl;
    std::cout << "Transformed constraints size: " << transformed_constraints.size() << std::endl;
    std::cout << "Transformed Constraints:" << std::endl; 
    for(Constraint c : transformed_constraints) {
        std::cout << get<Decomposition>(c.nodes_involved.first.second.content).id;
        if(c.type == PAR) {
            std::cout << " # ";
        } else {
            std::cout << " ; ";
        }
        std::cout << get<Decomposition>(c.nodes_involved.second.second.content).id;
        std::cout << std::endl;
    }

    return transformed_constraints;
}

/*
    Function: generate_noncoop_constraints
    Objective: Generate execution constraints present in the mission

    @ Input 1: A reference to the vector of constraints
    @ Input 2: The Task Graph as an ATGraph object
    @ Output: Void. The execution constraints will be added to the constraint vector
*/
void generate_noncoop_constraints(vector<Constraint>& mission_constraints, ATGraph mission_decomposition) {
    map<int,set<int>> non_coop_constraint_map;
    map<int,vector<pair<int,ATNode>>> constraint_nodes_decompositions;

    vector<Constraint> non_coop_constraints;
    
    auto nodes = vertices(mission_decomposition);

    int graph_size = *nodes.second - *nodes.first;

    for(int index = 0;index < graph_size;index++) {
        if(mission_decomposition[index].node_type == ATASK) {
            vector<pair<int,ATNode>> source_decompositions;
            if(constraint_nodes_decompositions.find(index) != constraint_nodes_decompositions.end()) {
                source_decompositions = constraint_nodes_decompositions[index];
            } else {
                source_decompositions = find_decompositions(mission_decomposition,index);
                constraint_nodes_decompositions[index] = source_decompositions;
            }

            ATGraph::out_edge_iterator ei, ei_end;
            for(boost::tie(ei,ei_end) = out_edges(index,mission_decomposition);ei != ei_end;++ei) {
                int source = boost::source(*ei,mission_decomposition);
                int target = boost::target(*ei,mission_decomposition);
                auto edge = boost::edge(source,target,mission_decomposition).first;

                ATEdge e = mission_decomposition[edge]; 

                if(e.edge_type == NONCOOP) {
                    vector<pair<int,ATNode>> target_decompositions;
                    if(constraint_nodes_decompositions.find(target) != constraint_nodes_decompositions.end()) {
                        target_decompositions = constraint_nodes_decompositions[target];
                    } else {
                        target_decompositions = find_decompositions(mission_decomposition,target);
                        constraint_nodes_decompositions[target] = target_decompositions;
                    }

                    bool already_exists = (non_coop_constraint_map[target].find(source) != non_coop_constraint_map[target].end());

                    if(!already_exists) {
                        non_coop_constraint_map[source].insert(target);

                        for(unsigned int i=0;i<source_decompositions.size();i++) {
                            for(unsigned int j=0;j<target_decompositions.size();j++) {
                                bool can_unite = can_unite_decompositions(get<Decomposition>(source_decompositions.at(i).second.content),get<Decomposition>(target_decompositions.at(j).second.content),true);

                                if(can_unite) {
                                    Constraint new_c;

                                    new_c.type = NC;
                                    new_c.nodes_involved.first = source_decompositions.at(i);
                                    new_c.nodes_involved.second = target_decompositions.at(j);
                                    new_c.group = e.group;
                                    new_c.divisible = e.divisible;

                                    mission_constraints.push_back(new_c);
                                    non_coop_constraints.push_back(new_c);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << std::endl;
    std::cout << "Non Coop constraints size: " << non_coop_constraints.size() << std::endl;
    std::cout << "Non Coop Constraints:" << std::endl; 
    for(Constraint c : non_coop_constraints) {
        std::cout << get<Decomposition>(c.nodes_involved.first.second.content).id;
        std::cout << " NC ";
        std::cout << get<Decomposition>(c.nodes_involved.second.second.content).id;
        std::cout << std::endl;
    }
    std::cout << std::endl;
}