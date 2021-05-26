#include "constraintmanager.hpp"

using namespace std;

/*
    Function: generate_at_constraints
    Objective: Generate all mission constraints, including parallel ones

    @ Input: The task graph as an ATGraph object
    @ Output: The vector with all of the mission constraints
*/
vector<Constraint> generate_at_constraints(ATGraph trimmed_mission_decomposition) {
    stack<pair<int,ATNode>> operators_stack;
    stack<variant<pair<int,ATNode>,Constraint>> nodes_stack;

    stack<pair<int,ATNode>> current_branch_operators_stack;
    stack<variant<pair<int,ATNode>,Constraint>> current_branch_nodes_stack;

    vector<Constraint> mission_constraints;
    map<int,set<int>> existing_constraints;

    /*
        Nodes with id -1 are artificial just to simulate the division between branches of the tree
    */
    string last_op = "";

    auto indexmap = boost::get(boost::vertex_index, trimmed_mission_decomposition);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    DFSATVisitor vis;
    boost::depth_first_search(trimmed_mission_decomposition, vis, colormap, 0);

    vector<int> vctr = vis.GetVector();
    operators_stack.push(make_pair(vctr.at(0),trimmed_mission_decomposition[vctr.at(0)]));

    pair<int,int> current_root_node = make_pair(vctr.at(0),1);

    unsigned int index;
    for(index = 1;index < vctr.size();index++) {      
        int v = index;

        pair<int,ATNode> current_node = make_pair(v, trimmed_mission_decomposition[v]);

        if(current_node.second.parent == vctr.at(0)) {
            pair<int,ATNode> artificial_node;
            artificial_node.first = -1;

            if(v != vctr.at(1)) {
                generate_constraints_from_stacks(current_branch_operators_stack, current_branch_nodes_stack, existing_constraints);
                
                while(!current_branch_nodes_stack.empty()) {
                    nodes_stack.push(current_branch_nodes_stack.top());
                    
                    current_branch_nodes_stack.pop();
                }
                
                nodes_stack.push(artificial_node);
            }

            current_root_node.first = current_node.first;
            current_root_node.second = index+1;
        } else if(current_node.second.parent == current_root_node.first && v != vctr.at(current_root_node.second)) {
            pair<int,ATNode> artificial_node;
            artificial_node.first = -1;

            current_branch_nodes_stack.push(artificial_node);

            current_root_node.first = current_node.first;
            current_root_node.second = index+1;
        }

        if(current_node.second.node_type == ATASK) {
            current_branch_nodes_stack.push(current_node);
        
            if(current_branch_operators_stack.size() > 0 && current_branch_nodes_stack.size() >= 2) {
                bool new_branch = false;
                
                /*
                    Check if we have at least 2 nodes until we reach some artificial node
                */
                stack<variant<pair<int,ATNode>,Constraint>> nodes_stack_cpy = current_branch_nodes_stack;
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
                    if(get<string>(current_branch_operators_stack.top().second.content) == "#") {
                        /*
                            While we don't reach an artificial node or the end of the nodes stack we populate our temporary vector
                        */
                        queue<variant<pair<int,ATNode>,Constraint>> temp;
                        bool end_reached = false;
                        while(!end_reached) {
                            if(holds_alternative<pair<int,ATNode>>(current_branch_nodes_stack.top())) {
                                if(get<pair<int,ATNode>>(current_branch_nodes_stack.top()).first == -1) {
                                    end_reached = true;
                                }
                            }

                            if(!end_reached) {
                                temp.push(current_branch_nodes_stack.top());
                                current_branch_nodes_stack.pop();
                            }

                            if(current_branch_nodes_stack.empty()) {
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
                                Constraint c = generate_constraint(std::get<pair<int,ATNode>>(temp.front()), last_task, PAR);

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

                                c1 = generate_constraint(constraint_nodes.first, last_task, PAR);
                                c2 = generate_constraint(constraint_nodes.second, last_task, PAR);

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
                            current_branch_nodes_stack.push(old_constraints.top());
                            old_constraints.pop();
                        }

                        while(!new_constraints.empty()) {
                            current_branch_nodes_stack.push(new_constraints.top());
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
                                if(holds_alternative<pair<int,ATNode>>(current_branch_nodes_stack.top())) {
                                    if(get<pair<int,ATNode>>(current_branch_nodes_stack.top()).first == -1) {
                                        end_reached = true;
                                    }
                                } else {
                                    if(get<Constraint>(current_branch_nodes_stack.top()).type == SEQ) {
                                        end_reached = true;
                                        temp.push(current_branch_nodes_stack.top());
                                        current_branch_nodes_stack.pop();
                                    }
                                }

                                if(!end_reached) {
                                    temp.push(current_branch_nodes_stack.top());
                                    current_branch_nodes_stack.pop();
                                }

                                if(current_branch_nodes_stack.empty()) {
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
                                    Constraint c = generate_constraint(std::get<pair<int,ATNode>>(temp.front()),last_task, SEQ);

                                    existing_constraints[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);

                                    new_constraints.push(c);
                                    temp.pop();
                                } else {
                                    if(get<Constraint>(temp.front()).type == PAR) {
                                        Constraint c1, c2;

                                        pair<pair<int,ATNode>,pair<int,ATNode>> constraint_nodes = get<Constraint>(temp.front()).nodes_involved;

                                        c1 = generate_constraint(constraint_nodes.first, last_task, SEQ);
                                        c2 = generate_constraint(constraint_nodes.second, last_task, SEQ);

                                        if(existing_constraints[constraint_nodes.second.first].find(last_task.first) == existing_constraints[constraint_nodes.second.first].end()) {
                                            existing_constraints[constraint_nodes.second.first].insert(last_task.first);
                                            new_constraints.push(c2);
                                        }

                                        if(existing_constraints[constraint_nodes.first.first].find(last_task.first) == existing_constraints[constraint_nodes.first.first].end()) {
                                            existing_constraints[constraint_nodes.first.first].insert(last_task.first);
                                            new_constraints.push(c1);
                                        }

                                        old_constraints.push(std::get<Constraint>(temp.front()));
                                        temp.pop();
                                    } else {
                                        Constraint c;

                                        pair<pair<int,ATNode>,pair<int,ATNode>> constraint_nodes = std::get<Constraint>(temp.front()).nodes_involved;

                                        c = generate_constraint(constraint_nodes.second, last_task, SEQ);

                                        if(existing_constraints[constraint_nodes.second.first].find(last_task.first) == existing_constraints[constraint_nodes.second.first].end()) {
                                            existing_constraints[constraint_nodes.second.first].insert(last_task.first);
                                            new_constraints.push(c);
                                        }

                                        old_constraints.push(std::get<Constraint>(temp.front()));
                                        temp.pop();
                                    }
                                }
                            }

                            while(!old_constraints.empty()) {
                                current_branch_nodes_stack.push(old_constraints.top());
                                old_constraints.pop();
                            }

                            while(!new_constraints.empty()) {
                                current_branch_nodes_stack.push(new_constraints.top());
                                new_constraints.pop();
                            }

                        } else if(last_op == ";") {
                            /*
                                If the last operator was sequential, we just need to create a constraint with the last constraint in the stack

                                -> If we create a constraint with another constraint, we just need to create with the second involved task
                            */
                            pair<int,ATNode> last_task = std::get<pair<int,ATNode>>(current_branch_nodes_stack.top());
                            current_branch_nodes_stack.pop();

                            Constraint new_constraint;
                            Constraint last_constraint = std::get<Constraint>(current_branch_nodes_stack.top());

                            new_constraint = generate_constraint(last_constraint.nodes_involved.second,last_task, SEQ);

                            current_branch_nodes_stack.push(new_constraint);
                            existing_constraints[last_constraint.nodes_involved.second.first].insert(last_task.first);
                        } else { 
                            /*
                                If no operator was considered yet, we need to create a constraint with another task
                            */
                            Constraint c;

                            pair<int,ATNode> last_task = std::get<pair<int,ATNode>>(current_branch_nodes_stack.top());
                            current_branch_nodes_stack.pop();

                            pair<int,ATNode> other_task = std::get<pair<int,ATNode>>(current_branch_nodes_stack.top());
                            current_branch_nodes_stack.pop();

                            c = generate_constraint(other_task, last_task, SEQ);

                            current_branch_nodes_stack.push(c);
                            existing_constraints[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);
                        }

                        last_op = ";";
                    }

                    current_branch_operators_stack.pop();
                }
            }
        } else {
            current_branch_operators_stack.push(current_node);
        }
    }

    generate_constraints_from_stacks(current_branch_operators_stack, current_branch_nodes_stack, existing_constraints);
    
    while(!current_branch_nodes_stack.empty()) {
        nodes_stack.push(current_branch_nodes_stack.top());
                    
        current_branch_nodes_stack.pop();
    }
    
    /*
        Here we will have the final operators and several constraints and possibly tasks, which we must combine to have all of the constraints of the mission
    */
    generate_constraints_from_stacks(operators_stack, nodes_stack, existing_constraints);

    while(!nodes_stack.empty()) {
        if(holds_alternative<Constraint>(nodes_stack.top())) {
            mission_constraints.push_back(std::get<Constraint>(nodes_stack.top()));
        } else {
            pair<int,ATNode> node = std::get<pair<int,ATNode>>(nodes_stack.top());
            string constraint_error = "Could not generate constraint with node " + std::get<AbstractTask>(node.second.content).id;

            throw std::runtime_error(constraint_error);
        }

        nodes_stack.pop();
    }

    return mission_constraints; 
}

/*
    Function: generate_constraints from stacks
    Objective: Generate constraints based on an input operators stack and an input nodes stack. Additionally, an existing
    constraints map is updated since this is an auxiliary function of the generate_at_constraints function

    @ Input 1: The operators stack
    @ Input 2: The nodes stack
    @ Input 3: The existing constraints map
    @ Output: Void. The input structures are updated
*/
void generate_constraints_from_stacks(stack<pair<int,ATNode>>& operators_stack, stack<variant<pair<int,ATNode>,Constraint>>& nodes_stack, map<int,set<int>>& existing_constraints) {
    while(!operators_stack.empty()) {
        pair<int,ATNode> current_op = operators_stack.top();
        operators_stack.pop();

        stack<variant<pair<int,ATNode>,Constraint>> considered_nodes;
        pair<bool,bool> changed_branch = make_pair(false,false);
        while(!changed_branch.second && !nodes_stack.empty()) {
            if(holds_alternative<pair<int,ATNode>>(nodes_stack.top())) {
                pair<int,ATNode> node = std::get<pair<int,ATNode>>(nodes_stack.top());
                if(node.first == -1) {
                    if(changed_branch.first) {
                        changed_branch.second = true;
                    } else {
                        changed_branch.first = true;
                        considered_nodes.push(node);
                        nodes_stack.pop();
                    }
                } else {
                    considered_nodes.push(node);
                    nodes_stack.pop();
                }
            } else {
                Constraint node = std::get<Constraint>(nodes_stack.top());
                nodes_stack.pop();
                considered_nodes.push(node);
            }
        }

        stack<variant<pair<int,ATNode>,Constraint>> aux_stack = considered_nodes;
        while(!aux_stack.empty()) {
            if(holds_alternative<Constraint>(aux_stack.top())) {
                nodes_stack.push(std::get<Constraint>(aux_stack.top()));
            }

            aux_stack.pop();
        }

        if(get<string>(current_op.second.content) == "#") {
            /*
                If we have a parallel operator we need to create parallel constraints for all tasks that are not in the existing constraints map
            */
            vector<vector<Constraint>> constraint_branches;
            vector<Constraint> aux;
            while(!considered_nodes.empty()) {
                if(holds_alternative<pair<int,ATNode>>(considered_nodes.top())) {
                    pair<int,ATNode> top_node = std::get<pair<int,ATNode>>(considered_nodes.top());
                    if(top_node.first == -1) {
                        // Artificial node
                        constraint_branches.push_back(aux);
                        aux.clear();
                    } else {
                        // Abstract Task node
                        pair<int,ATNode> artificial_node;
                        artificial_node.first = -1;

                        Constraint c = generate_constraint(top_node, artificial_node, PAR);
                        aux.push_back(c);
                    }
                } else {
                    Constraint top_node = std::get<Constraint>(considered_nodes.top());
                    aux.push_back(top_node);
                }

                considered_nodes.pop();

                if(considered_nodes.empty() && aux.size() > 0) {
                    constraint_branches.push_back(aux);
                }
            }

            if(constraint_branches.size() > 1) {
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

                                nc1 = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, PAR);

                                nc2 = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.second, PAR);
                                if(c2.nodes_involved.second.first == -1) {
                                    nc2.type = NEX;
                                }

                                nc3 = generate_constraint(c1.nodes_involved.second, c2.nodes_involved.first, PAR);
                                if(c1.nodes_involved.second.first == -1) {
                                    nc3.type = NEX;
                                }

                                nc4 = generate_constraint(c1.nodes_involved.second, c2.nodes_involved.second, PAR);
                                if(c1.nodes_involved.second.first == -1 || c2.nodes_involved.second.first == -1) {
                                    nc4.type = NEX;
                                }

                                if(existing_constraints[nc1.nodes_involved.first.first].find(nc1.nodes_involved.second.first) == existing_constraints[nc1.nodes_involved.first.first].end()) {
                                    nodes_stack.push(nc1);
                                
                                    existing_constraints[nc1.nodes_involved.first.first].insert(nc1.nodes_involved.second.first);
                                }
    
                                if(nc2.type != NEX) {
                                    if(existing_constraints[nc2.nodes_involved.first.first].find(nc2.nodes_involved.second.first) == existing_constraints[nc2.nodes_involved.first.first].end()) {
                                        nodes_stack.push(nc2);

                                        existing_constraints[nc2.nodes_involved.first.first].insert(nc2.nodes_involved.second.first);
                                    }
                                }

                                if(nc3.type != NEX) {
                                    if(existing_constraints[nc3.nodes_involved.first.first].find(nc3.nodes_involved.second.first) == existing_constraints[nc3.nodes_involved.first.first].end()) {
                                        nodes_stack.push(nc3);

                                        existing_constraints[nc3.nodes_involved.first.first].insert(nc3.nodes_involved.second.first);
                                    }
                                }

                                if(nc4.type != NEX) {
                                    if(existing_constraints[nc4.nodes_involved.first.first].find(nc4.nodes_involved.second.first) == existing_constraints[nc4.nodes_involved.first.first].end()) {
                                        nodes_stack.push(nc4);

                                        existing_constraints[nc4.nodes_involved.first.first].insert(nc4.nodes_involved.second.first);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else if(get<string>(current_op.second.content) == ";") {
            /*
                If we have a sequential operator, we just need to create additional constraints between the constraints that are between artificial nodes
            */
            vector<vector<Constraint>> constraint_branches;
            vector<Constraint> aux;
            while(!considered_nodes.empty()) {
                if(holds_alternative<pair<int,ATNode>>(considered_nodes.top())) {
                    pair<int,ATNode> top_node = std::get<pair<int,ATNode>>(considered_nodes.top());
                    if(top_node.first == -1) {
                        //Artificial node
                        constraint_branches.push_back(aux);
                        aux.clear();
                    } else {
                        /*
                            Abstract Task Node
                        */
                        pair<int,ATNode> artificial_node;
                        artificial_node.first = -1;
                        
                        Constraint c = generate_constraint(top_node, artificial_node, PAR);
                        aux.push_back(c);
                    }
                } else {
                    Constraint top_node = std::get<Constraint>(considered_nodes.top());
                    aux.push_back(top_node);
                }

                considered_nodes.pop();

                if(considered_nodes.empty() && aux.size() > 0) {
                    constraint_branches.push_back(aux);
                }
            }

            if(constraint_branches.size() > 1) {
                unsigned int i,j;
                for(i=0;i<constraint_branches.size()-1;i++) {
                    for(j=i+1;j<constraint_branches.size();j++) {
                        vector<Constraint> vc1, vc2;

                        vc1 = constraint_branches.at(i);
                        vc2 = constraint_branches.at(i+1);

                        unsigned int index1, index2;
                        for(index1=0;index1<vc1.size();index1++) {
                            for(index2=0;index2<vc2.size();index2++) {
                                Constraint c1 = vc1.at(index1);
                                Constraint c2 = vc2.at(index2);

                                vector<Constraint> c;
                                if(c1.type == SEQ) {
                                    pair<int,ATNode> node;
                                    if(c1.nodes_involved.second.first == -1) {
                                        node = c1.nodes_involved.first;
                                    } else {
                                        node = c1.nodes_involved.second;
                                    }
                                    if(c2.type == SEQ) {
                                        Constraint ct = generate_constraint(node, c2.nodes_involved.first, SEQ);

                                        c.push_back(ct);
                                    } else {
                                        unsigned int k;
                                        if(c2.nodes_involved.second.first == -1) {
                                            Constraint ct = generate_constraint(node, c2.nodes_involved.first, SEQ);

                                            c.push_back(ct);
                                        } else {
                                            for(k = 0;k < 2;k++) {
                                                Constraint ct;

                                                if(k == 0) {
                                                    ct = generate_constraint(node, c2.nodes_involved.first, SEQ);
                                                } else {
                                                    ct = generate_constraint(node, c2.nodes_involved.second, SEQ);
                                                }

                                                c.push_back(ct);
                                            }
                                        }
                                    }
                                } else {
                                    if(c2.type == SEQ) {
                                        unsigned int k;
                                        if(c1.nodes_involved.second.first == -1) {
                                            Constraint ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, SEQ);

                                            c.push_back(ct);
                                        } else {
                                            for(k = 0;k < 2;k++) {
                                                Constraint ct;

                                                if(k == 0) {
                                                    ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, SEQ);
                                                } else {
                                                    ct = generate_constraint(c1.nodes_involved.second, c2.nodes_involved.first, SEQ);
                                                }

                                                c.push_back(ct);
                                            }
                                        }
                                    } else {
                                        if(c1.nodes_involved.second.first == -1) {
                                            unsigned int k;
                                            if(c2.nodes_involved.second.first == -1) { 
                                                Constraint ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, SEQ);

                                                c.push_back(ct);
                                            } else {
                                                for(k = 0;k < 2;k++) {
                                                    Constraint ct;

                                                    if(k == 0) {
                                                        ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, SEQ);
                                                    } else {
                                                        ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.second, SEQ);
                                                    }

                                                    c.push_back(ct);
                                                }
                                            }
                                        } else {
                                            unsigned int k;
                                            if(c2.nodes_involved.second.first == -1) {
                                                for(k = 0;k < 2;k++) {
                                                    Constraint ct;

                                                    if(k == 0) {
                                                        ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, SEQ);
                                                    } else {
                                                        ct = generate_constraint(c1.nodes_involved.second, c2.nodes_involved.first, SEQ);
                                                    }

                                                    c.push_back(ct);
                                                }
                                            } else {
                                                for(k = 0;k < 4;k++) {
                                                    Constraint ct;

                                                    switch(k) {
                                                        case 0:
                                                            ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.first, SEQ);
                                                            break;
                                                        case 1:
                                                            ct = generate_constraint(c1.nodes_involved.first, c2.nodes_involved.second, SEQ);
                                                            break;
                                                        case 2:
                                                            ct = generate_constraint(c1.nodes_involved.second, c2.nodes_involved.first, SEQ);
                                                            break;
                                                        case 3:
                                                            ct = generate_constraint(c1.nodes_involved.second, c2.nodes_involved.second, SEQ);
                                                    }

                                                    c.push_back(ct);
                                                }
                                            }
                                        }
                                    }
                                }

                                for(Constraint ct : c) {
                                    if(existing_constraints[ct.nodes_involved.first.first].find(ct.nodes_involved.second.first) == existing_constraints[ct.nodes_involved.first.first].end()) {
                                        nodes_stack.push(ct);

                                        existing_constraints[ct.nodes_involved.first.first].insert(ct.nodes_involved.second.first);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
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
vector<Constraint> transform_at_constraints(ATGraph mission_decomposition, vector<Constraint> mission_constraints, GMGraph gm, bool verbose) {
    vector<Constraint> transformed_constraints;
    map<int,vector<pair<int,ATNode>>> constraint_nodes_decompositions;

    /*std::cout << std::endl;
    std::cout << "Mission constraints size: " << mission_constraints.size() << std::endl;
    std::cout << "Mission Constraints:" << std::endl; 
    for(Constraint c : mission_constraints) {
        std::cout << std::get<AbstractTask>(c.nodes_involved.first.second.content).id;
        if(c.type == PAR) {
            std::cout << " # ";
        } else {
            std::cout << " ; ";
        }
        std::cout << std::get<AbstractTask>(c.nodes_involved.second.second.content).id;
        std::cout << std::endl;
    }*/

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

                constraint_nodes_decompositions[n1.first] = n1_decompositions;
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
                    bool can_unite = can_unite_decompositions(std::get<Decomposition>(n1_decompositions.at(i).second.content),get<Decomposition>(n2_decompositions.at(j).second.content),non_coop_nodes);

                    if(can_unite) {
                        Constraint new_c = generate_constraint(n1_decompositions.at(i), n2_decompositions.at(j), SEQ);

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

                constraint_nodes_decompositions[n1.first] = n1_decompositions;
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
                            }
                        } 
                    }

                    if(context_dependency_valid) {
                        for(pair<int,ATNode> n2_decomposition : n2_decompositions) {
                            Constraint c = generate_constraint(n1_decomposition, n2_decomposition, SEQ);

                            transformed_constraints.push_back(c);
                        }

                        break;
                    }
                }
            }
        }
    }

    if(verbose) {
        std::cout << std::endl;
        std::cout << "Number of Sequential Mission Constraints: " << transformed_constraints.size() << std::endl;
        std::cout << "Sequential Constraints:" << std::endl; 
        for(Constraint c : transformed_constraints) {
            std::cout << std::get<Decomposition>(c.nodes_involved.first.second.content).id;
            if(c.type == PAR) {
                string parallel_constraint_error = "Parallel constraints should not exist in final constraints";

                throw std::runtime_error(parallel_constraint_error);
            } else {
                std::cout << " ; ";
            }
            std::cout << std::get<Decomposition>(c.nodes_involved.second.second.content).id;
            std::cout << std::endl;
        }
    } else {
        for(Constraint c : transformed_constraints) {
            if(c.type == PAR) {
                string parallel_constraint_error = "Parallel constraints should not exist in final constraints";

                throw std::runtime_error(parallel_constraint_error);
            }
        }
    }

    return transformed_constraints;
}

/*
    Function: generate_execution_constraints
    Objective: Generate execution constraints present in the mission

    @ Input 1: A reference to the vector of constraints
    @ Input 2: The Task Graph as an ATGraph object
    @ Output: Void. The execution constraints will be added to the constraint vector
*/
void generate_execution_constraints(vector<Constraint>& mission_constraints, ATGraph mission_decomposition, bool verbose) {
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
                                    Constraint new_c = generate_constraint(source_decompositions.at(i), target_decompositions.at(j), NC);
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

    if(verbose) {
        std::cout << std::endl;
        std::cout << "Number of execution constraints: " << non_coop_constraints.size() << std::endl;
        std::cout << "Execution Constraints:" << std::endl; 
        for(Constraint c : non_coop_constraints) {
            std::cout << get<Decomposition>(c.nodes_involved.first.second.content).id;
            std::cout << " EC ";
            std::cout << get<Decomposition>(c.nodes_involved.second.second.content).id;
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

/*
    Function: generate_constraints
    Objective: Generates a constraint given the two nodes involved and the constraint type

    @ Input 1: The first node
    @ Input 2: The second node
    @ Input 3: The constraint type
    @ Output: The generated constraint
*/
Constraint generate_constraint(pair<int,ATNode> n1, pair<int,ATNode> n2, constraint_type type) {
    Constraint c;
    c.type = type;
    c.nodes_involved = make_pair(n1,n2);

    return c;
}