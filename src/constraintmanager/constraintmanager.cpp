#include "constraintmanager.hpp"

#include <algorithm>

using namespace std;

ConstraintManager::ConstraintManager(GMGraph g, ATGraph md, bool verb) {
    this->mission_decomposition = md;
    this->gm = g;
    this->verbose = verb;
}

/*
    Function: generate_mission_constraints
    Objective: Generate mission constraints (sequential and execution)

    Output: The mission constraints.
*/
vector<Constraint> ConstraintManager::generate_mission_constraints() {
     bool is_unique = is_unique_branch(mission_decomposition);
    
    if(!is_unique) {
        ATGraph trimmed_mission_decomposition = generate_tree_like_at_graph(mission_decomposition);
        generate_at_constraints(trimmed_mission_decomposition);
    }

    transform_at_constraints();
    generate_execution_constraints();
    trim_mission_constraints();

    check_execution_constraints();

    if(verbose) {
        vector<Constraint> sequential_constraints, execution_constraints, fallback_constraints;
        for(Constraint c : mission_constraints) {
            if(c.type == SEQ) {
                sequential_constraints.push_back(c);
            } else if(c.type == NC) {
                execution_constraints.push_back(c);
            } else if(c.type == FB) {
                fallback_constraints.push_back(c);
            }
        }

        std::cout << std::endl;
        std::cout << "Number of Sequential Mission Constraints: " << sequential_constraints.size() << std::endl;
        std::cout << "Sequential Constraints:" << std::endl; 
        for(Constraint c : sequential_constraints) {
            std::cout << std::get<Decomposition>(c.nodes_involved.first.second.content).id;
            
            std::cout << " ; ";

            std::cout << std::get<Decomposition>(c.nodes_involved.second.second.content).id;
            std::cout << std::endl;
        }

        std::cout << std::endl;
        std::cout << "Number of execution constraints: " << execution_constraints.size() << std::endl;
        std::cout << "Execution Constraints:" << std::endl; 
        for(Constraint c : execution_constraints) {
            std::cout << get<Decomposition>(c.nodes_involved.first.second.content).id;
            std::cout << " EC ";
            std::cout << get<Decomposition>(c.nodes_involved.second.second.content).id;
            std::cout << std::endl;
        }

        std::cout << std::endl;
        std::cout << "Number of fallback constraints: " << fallback_constraints.size() << std::endl;
        std::cout << "Fallback Constraints:" << std::endl; 
        for(Constraint c : fallback_constraints) {
            std::cout << get<Decomposition>(c.nodes_involved.first.second.content).id;
            std::cout << " FB ";
            std::cout << get<Decomposition>(c.nodes_involved.second.second.content).id;
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    return mission_constraints;
}

/*
    Function: generate_at_constraints
    Objective: Generate all mission constraints, including parallel ones

    @ Input: The trimmed mission decomposition as an ATGraph object
    @ Output: The vector with all of the mission constraints
*/
void ConstraintManager::generate_at_constraints(ATGraph trimmed_mission_decomposition) {
    auto indexmap = boost::get(boost::vertex_index, trimmed_mission_decomposition);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    DFSATVisitor vis;
    boost::depth_first_search(trimmed_mission_decomposition, vis, colormap, 0);

    vector<int> dfs_nodes = vis.GetVector();

    variant<pair<int,ATNode>,ConstraintTree*> ct = recursive_constraint_tree_build(dfs_nodes, trimmed_mission_decomposition);
    if(!holds_alternative<ConstraintTree*>(ct)) {
        string constraint_generation_error = "Could not generate constraints";

        throw std::runtime_error(constraint_generation_error);
    }

    ConstraintTree* constraints_tree = std::get<ConstraintTree*>(ct);
    constraints_tree->generate_constraints();

    mission_constraints = constraints_tree->constraints;
    mission_constraints.insert(mission_constraints.end(), constraints_tree->child_constraints.begin(), constraints_tree->child_constraints.end());
}

std::variant<std::pair<int,ATNode>,ConstraintTree*> ConstraintManager::recursive_constraint_tree_build(vector<int>& dfs_nodes, ATGraph trimmed_mission_decomposition) {
    int current_node_index = dfs_nodes.at(0);
    dfs_nodes.erase(dfs_nodes.begin());

    ATNode current_node = trimmed_mission_decomposition[current_node_index];

    if(holds_alternative<string>(current_node.content)) { // Operator
        string op = std::get<string>(current_node.content);

        ConstraintTree* op_tree;
        if(op == sequential_op) {
            op_tree = new SequentialConstraintTree();
        } else if(op == fallback_op) {
            op_tree = new FallbackConstraintTree();
        } else if(op == parallel_op) {
            op_tree = new ParallelConstraintTree();
        }

        bool is_child = true;
        while(is_child) {
            if(dfs_nodes.size() == 0) {
                break;
            }

            int next_node_index = dfs_nodes.at(0);
            auto edge = boost::edge(current_node_index, next_node_index, trimmed_mission_decomposition);

            if(!edge.second) { //If edge exists we have a child
                is_child = false;
            }

            if(is_child) {
                variant<pair<int,ATNode>,ConstraintTree*> child_tree = recursive_constraint_tree_build(dfs_nodes, trimmed_mission_decomposition);
                op_tree->children.push_back(child_tree);
            }
        }

        return op_tree;
    } else if(holds_alternative<AbstractTask>(current_node.content)) {
        bool is_child = true;

        while(is_child) {
            if(dfs_nodes.size() == 0) {
                break;
            }

            int next_node_index = dfs_nodes.at(0);
            auto edge = boost::edge(current_node_index, next_node_index, trimmed_mission_decomposition);

            if(!edge.second) { //If edge exists we have a child
                is_child = false;
            }

            if(is_child) {
                variant<pair<int,ATNode>,ConstraintTree*> decomposition = recursive_constraint_tree_build(dfs_nodes, trimmed_mission_decomposition);
            }
        }

        return make_pair(current_node_index, current_node);
    } 
    
    return make_pair(current_node_index, current_node);
}

void SequentialConstraintTree::generate_constraints() {
    for(auto child : children) {
        if(holds_alternative<ConstraintTree*>(child)) {
            ConstraintTree* ch = std::get<ConstraintTree*>(child);

            ch->generate_constraints();
            child_constraints.insert(child_constraints.end(),ch->constraints.begin(),ch->constraints.end());
            child_constraints.insert(child_constraints.end(),ch->child_constraints.begin(),ch->child_constraints.end());
        }
    }

    unsigned int right_child_index = 1;
    for(unsigned int left_child_index = 0; left_child_index < children.size()-1; left_child_index++) {
        variant<pair<int,ATNode>,vector<Constraint>> left_child_val, right_child_val;

        if(holds_alternative<ConstraintTree*>(children.at(left_child_index))) {
            ConstraintTree* left_child = std::get<ConstraintTree*>(children.at(left_child_index));
        
            left_child_val = left_child->constraints;
        } else {
            pair<int,ATNode> left_child = std::get<pair<int,ATNode>>(children.at(left_child_index));

            left_child_val = left_child;
        }

        if(holds_alternative<ConstraintTree*>(children.at(right_child_index))) {
            ConstraintTree* right_child = std::get<ConstraintTree*>(children.at(right_child_index));
        
            right_child_val = right_child->constraints;
        } else {
            pair<int,ATNode> right_child = std::get<pair<int,ATNode>>(children.at(right_child_index));

            right_child_val = right_child;
        }

        vector<Constraint> child_generated_constraints = generate_constraints_from_child_contents(left_child_val, right_child_val);
        constraints.insert(constraints.end(), child_generated_constraints.begin(), child_generated_constraints.end());

        right_child_index++;
    }
}

vector<Constraint> SequentialConstraintTree::generate_constraints_from_child_contents(variant<pair<int,ATNode>,vector<Constraint>> left_val, variant<pair<int,ATNode>,vector<Constraint>> right_val) {
    vector<Constraint> new_constraints;
    
    if(holds_alternative<vector<Constraint>>(left_val)) { 
        vector<Constraint> left_value = std::get<vector<Constraint>>(left_val);

        if(holds_alternative<vector<Constraint>>(right_val)) {
            vector<Constraint> right_value = std::get<vector<Constraint>>(right_val);

            for(Constraint lval : left_value) {
                for(Constraint rval : right_value) {
                    if(lval.type == SEQ) {
                        if(rval.type == SEQ || rval.type == FB) {
                            Constraint c = generate_constraint(lval.nodes_involved.second,rval.nodes_involved.first,SEQ);

                            bool insert_constraint = is_new_constraint(c, constraints_map);

                            if(insert_constraint) {
                                new_constraints.push_back(c);
                            }
                        } else {
                            Constraint c1, c2;

                            c1 = generate_constraint(lval.nodes_involved.second,rval.nodes_involved.first,SEQ);
                            c2 = generate_constraint(lval.nodes_involved.second,rval.nodes_involved.second,SEQ);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }
                        }
                    } else if(lval.type == PAR) {
                        if(rval.type == SEQ || rval.type == FB) {
                            Constraint c1, c2;

                            c1 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.first, SEQ);
                            c2 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, SEQ);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }
                        } else {
                            Constraint c1, c2, c3, c4;

                            c1 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.first, SEQ);
                            c2 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.second, SEQ);
                            c3 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, SEQ);
                            c4 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.second, SEQ);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }

                            insert_constraint = is_new_constraint(c3, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c3);
                            }
                            
                            insert_constraint = is_new_constraint(c4, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c4);
                            }
                        }
                    } else if(lval.type == FB) {
                        if(rval.type == SEQ || rval.type == FB) {
                            Constraint c1, c2;

                            c1 = generate_constraint(lval.nodes_involved.first,rval.nodes_involved.first,SEQ);
                            c2 = generate_constraint(lval.nodes_involved.second,rval.nodes_involved.first,SEQ);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }
                        } else {
                            Constraint c1, c2, c3, c4;

                            c1 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.first, SEQ);
                            c2 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.second, SEQ);
                            c3 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, SEQ);
                            c4 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.second, SEQ);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }

                            insert_constraint = is_new_constraint(c3, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c3);
                            }
                            
                            insert_constraint = is_new_constraint(c4, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c4);
                            }
                        }
                    }
                }
            }
        } else {
            pair<int,ATNode> right_value = std::get<pair<int,ATNode>>(right_val);

            for(Constraint lval : left_value) {
                if(lval.type == SEQ) {
                    Constraint c = generate_constraint(lval.nodes_involved.second, right_value, SEQ);

                    bool insert_constraint = is_new_constraint(c, constraints_map);

                    if(insert_constraint) {
                        new_constraints.push_back(c);
                    }
                } else if(lval.type == FB || lval.type == PAR) {
                    Constraint c1, c2;

                    c1 = generate_constraint(lval.nodes_involved.first, right_value, SEQ);
                    c2 = generate_constraint(lval.nodes_involved.second, right_value, SEQ);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c1, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c1);
                    }

                    insert_constraint = is_new_constraint(c2, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c2);
                    }
                }
            }
        }
    } else {
        pair<int,ATNode> left_value = std::get<pair<int,ATNode>>(left_val);

        if(holds_alternative<vector<Constraint>>(right_val)) {
            vector<Constraint> right_value = std::get<vector<Constraint>>(right_val);

            for(Constraint rval : right_value) {
                if(rval.type == SEQ || rval.type == FB) {
                    Constraint c = generate_constraint(left_value, rval.nodes_involved.first, SEQ);

                    bool insert_constraint = is_new_constraint(c, constraints_map);

                    if(insert_constraint) {
                        new_constraints.push_back(c);
                    }
                } else if(rval.type == PAR) {
                    Constraint c1, c2;

                    c1 = generate_constraint(left_value, rval.nodes_involved.first, SEQ);
                    c2 = generate_constraint(left_value, rval.nodes_involved.second, SEQ);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c1, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c1);
                    }

                    insert_constraint = is_new_constraint(c2, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c2);
                    }
                }
            }
        } else {
            pair<int,ATNode> right_value = std::get<pair<int,ATNode>>(right_val);

            Constraint c = generate_constraint(left_value, right_value, SEQ);

            bool insert_constraint = is_new_constraint(c, constraints_map);

            if(insert_constraint) {
                new_constraints.push_back(c);
            }
        }
    }

    return new_constraints;
}

void FallbackConstraintTree::generate_constraints() {
    for(auto child : children) {
        if(holds_alternative<ConstraintTree*>(child)) {
            ConstraintTree* ch = std::get<ConstraintTree*>(child);

            ch->generate_constraints();
            child_constraints.insert(child_constraints.end(),ch->constraints.begin(),ch->constraints.end());
            child_constraints.insert(child_constraints.end(),ch->child_constraints.begin(),ch->child_constraints.end());
        }
    }

    unsigned int right_child_index = 1;
    for(unsigned int left_child_index = 0; left_child_index < children.size()-1; left_child_index++) {
        variant<pair<int,ATNode>,vector<Constraint>> left_child_val, right_child_val;

        if(holds_alternative<ConstraintTree*>(children.at(left_child_index))) {
            ConstraintTree* left_child = std::get<ConstraintTree*>(children.at(left_child_index));
        
            left_child_val = left_child->constraints;
        } else {
            pair<int,ATNode> left_child = std::get<pair<int,ATNode>>(children.at(left_child_index));

            left_child_val = left_child;
        }

        if(holds_alternative<ConstraintTree*>(children.at(right_child_index))) {
            ConstraintTree* right_child = std::get<ConstraintTree*>(children.at(right_child_index));
        
            right_child_val = right_child->constraints;
        } else {
            pair<int,ATNode> right_child = std::get<pair<int,ATNode>>(children.at(right_child_index));

            right_child_val = right_child;
        }

        vector<Constraint> child_generated_constraints = generate_constraints_from_child_contents(left_child_val, right_child_val);
        constraints.insert(constraints.end(), child_generated_constraints.begin(), child_generated_constraints.end());

        right_child_index++;
    }
}

vector<Constraint> FallbackConstraintTree::generate_constraints_from_child_contents(variant<pair<int,ATNode>,vector<Constraint>> left_val, variant<pair<int,ATNode>,vector<Constraint>> right_val) {
    vector<Constraint> new_constraints;

    if(holds_alternative<vector<Constraint>>(left_val)) { 
        vector<Constraint> left_value = std::get<vector<Constraint>>(left_val);

        if(holds_alternative<vector<Constraint>>(right_val)) {
            vector<Constraint> right_value = std::get<vector<Constraint>>(right_val);

            for(Constraint lval : left_value) {
                for(Constraint rval : right_value) {
                    if(lval.type == SEQ || lval.type == PAR) {
                        if(rval.type == SEQ || rval.type == PAR) {
                            Constraint c1, c2, c3, c4;

                            c1 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.first, FB);
                            c2 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.second, FB);
                            c3 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, FB);
                            c4 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.second, FB);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }

                            insert_constraint = is_new_constraint(c3, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c3);
                            }
                            
                            insert_constraint = is_new_constraint(c4, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c4);
                            }
                        } else {
                            Constraint c1, c2;

                            c1 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.first, FB);
                            c2 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, FB);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }
                        }
                    } else {
                        if(rval.type == SEQ || rval.type == PAR) {
                            Constraint c1, c2;

                            c1 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, FB);
                            c2 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.second, FB);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c1, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c1);
                            }

                            insert_constraint = is_new_constraint(c2, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c2);
                            }
                        } else {
                            Constraint c;

                            c = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, FB);

                            bool insert_constraint;

                            insert_constraint = is_new_constraint(c, constraints_map);
                            if(insert_constraint) {
                                new_constraints.push_back(c);
                            }
                        }
                    }
                }
            }
        } else {
            pair<int,ATNode> right_value = std::get<pair<int,ATNode>>(right_val);

            for(Constraint lval : left_value) {
                if(lval.type == SEQ || lval.type == PAR) {
                    Constraint c1, c2;

                    c1 = generate_constraint(lval.nodes_involved.first, right_value, FB);
                    c2 = generate_constraint(lval.nodes_involved.second, right_value, FB);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c1, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c1);
                    }

                    insert_constraint = is_new_constraint(c2, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c2);
                    }
                } else {
                    Constraint c;

                    c = generate_constraint(lval.nodes_involved.second, right_value, FB);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c);
                    }
                }
            }
        }
    } else {
        pair<int,ATNode> left_value = std::get<pair<int,ATNode>>(left_val);

        if(holds_alternative<vector<Constraint>>(right_val)) {
            vector<Constraint> right_value = std::get<vector<Constraint>>(right_val);

            for(Constraint rval : right_value) {
                if(rval.type == SEQ || rval.type == PAR) {
                    Constraint c1, c2;

                    c1 = generate_constraint(left_value, rval.nodes_involved.first, FB);
                    c2 = generate_constraint(left_value, rval.nodes_involved.second, FB);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c1, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c1);
                    }

                    insert_constraint = is_new_constraint(c2, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c2);
                    }   
                } else {
                    Constraint c;

                    c = generate_constraint(left_value, rval.nodes_involved.first, FB);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c);
                    }
                }
            }
        } else {
            pair<int,ATNode> right_value = std::get<pair<int,ATNode>>(right_val);

            Constraint c = generate_constraint(left_value, right_value, FB);

            bool insert_constraint = is_new_constraint(c, constraints_map);

            if(insert_constraint) {
                new_constraints.push_back(c);
            }
        }
    }

    return new_constraints;
}

void ParallelConstraintTree::generate_constraints() {
    for(auto child : children) {
        if(holds_alternative<ConstraintTree*>(child)) {
            ConstraintTree* ch = std::get<ConstraintTree*>(child);

            ch->generate_constraints();
            child_constraints.insert(child_constraints.end(),ch->constraints.begin(),ch->constraints.end());
            child_constraints.insert(child_constraints.end(),ch->child_constraints.begin(),ch->child_constraints.end());
        }
    }

    for(unsigned int left_child_index = 0; left_child_index < children.size()-1; left_child_index++) {
        for(unsigned int right_child_index = left_child_index+1; right_child_index < children.size(); right_child_index++) {
            variant<pair<int,ATNode>,vector<Constraint>> left_child_val, right_child_val;

            if(holds_alternative<ConstraintTree*>(children.at(left_child_index))) {
                ConstraintTree* left_child = std::get<ConstraintTree*>(children.at(left_child_index));
            
                left_child_val = left_child->constraints;
            } else {
                pair<int,ATNode> left_child = std::get<pair<int,ATNode>>(children.at(left_child_index));

                left_child_val = left_child;
            }

            if(holds_alternative<ConstraintTree*>(children.at(right_child_index))) {
                ConstraintTree* right_child = std::get<ConstraintTree*>(children.at(right_child_index));
            
                right_child_val = right_child->constraints;
            } else {
                pair<int,ATNode> right_child = std::get<pair<int,ATNode>>(children.at(right_child_index));

                right_child_val = right_child;
            }

            vector<Constraint> child_generated_constraints = generate_constraints_from_child_contents(left_child_val, right_child_val);
            constraints.insert(constraints.end(), child_generated_constraints.begin(), child_generated_constraints.end());
        }
    }
}

vector<Constraint> ParallelConstraintTree::generate_constraints_from_child_contents(variant<pair<int,ATNode>,vector<Constraint>> left_val, variant<pair<int,ATNode>,vector<Constraint>> right_val) {
    vector<Constraint> new_constraints;
    
    if(holds_alternative<vector<Constraint>>(left_val)) { 
        vector<Constraint> left_value = std::get<vector<Constraint>>(left_val);

        if(holds_alternative<vector<Constraint>>(right_val)) {
            vector<Constraint> right_value = std::get<vector<Constraint>>(right_val);

            for(Constraint lval : left_value) {
                for(Constraint rval : right_value) {
                    Constraint c1, c2, c3, c4;

                    c1 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.first, PAR);
                    c2 = generate_constraint(lval.nodes_involved.first, rval.nodes_involved.second, PAR);
                    c3 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.first, PAR);
                    c4 = generate_constraint(lval.nodes_involved.second, rval.nodes_involved.second, PAR);

                    bool insert_constraint;

                    insert_constraint = is_new_constraint(c1, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c1);
                    }

                    insert_constraint = is_new_constraint(c2, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c2);
                    }

                    insert_constraint = is_new_constraint(c3, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c3);
                    }
                    
                    insert_constraint = is_new_constraint(c4, constraints_map);
                    if(insert_constraint) {
                        new_constraints.push_back(c4);
                    }
                }
            }
        } else {
            pair<int,ATNode> right_value = std::get<pair<int,ATNode>>(right_val);

            for(Constraint lval : left_value) {
                Constraint c1, c2;

                c1 = generate_constraint(lval.nodes_involved.first, right_value, PAR);
                c2 = generate_constraint(lval.nodes_involved.second, right_value, PAR);

                bool insert_constraint;

                insert_constraint = is_new_constraint(c1, constraints_map);
                if(insert_constraint) {
                    new_constraints.push_back(c1);
                }

                insert_constraint = is_new_constraint(c2, constraints_map);
                if(insert_constraint) {
                    new_constraints.push_back(c2);
                }
            }
        }
    } else {
        pair<int,ATNode> left_value = std::get<pair<int,ATNode>>(left_val);

        if(holds_alternative<vector<Constraint>>(right_val)) {
            vector<Constraint> right_value = std::get<vector<Constraint>>(right_val);

            for(Constraint rval : right_value) {
                Constraint c1, c2;

                c1 = generate_constraint(left_value, rval.nodes_involved.first, PAR);
                c2 = generate_constraint(left_value, rval.nodes_involved.second, PAR);

                bool insert_constraint;

                insert_constraint = is_new_constraint(c1, constraints_map);
                if(insert_constraint) {
                    new_constraints.push_back(c1);
                }

                insert_constraint = is_new_constraint(c2, constraints_map);
                if(insert_constraint) {
                    new_constraints.push_back(c2);
                }
            }
        } else {
            pair<int,ATNode> right_value = std::get<pair<int,ATNode>>(right_val);

            Constraint c = generate_constraint(left_value, right_value, PAR);

            bool insert_constraint = is_new_constraint(c, constraints_map);

            if(insert_constraint) {
                new_constraints.push_back(c);
            }
        }
    }

    return new_constraints;
}

bool is_new_constraint(Constraint c, map<int,set<int>>& constraints_map) {
    bool is_new = false;

    if(constraints_map.find(c.nodes_involved.first.first) != constraints_map.end()) {
        if(constraints_map[c.nodes_involved.first.first].find(c.nodes_involved.second.first) == constraints_map[c.nodes_involved.first.first].end()) {
            is_new = true;

            constraints_map[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);
        }
    } else {
        is_new = true;
        
        constraints_map[c.nodes_involved.first.first] = set<int>();
        constraints_map[c.nodes_involved.first.first].insert(c.nodes_involved.second.first);
    }

    return is_new;
}

/*
    Function: transform_at_constraints
    Objective: Here we will create the final constraints involving the decompositions and not the abstract tasks

    @ Input 1: The Task Graph as an ATGraph object
    @ Input 2: The vector of constraints
    @ Input 3: The goal model as a GMGraph object
    @ Input 4: The verbose flag
    @ Output: The final constraint vector of the mission

    NOTES:  -> For now we will only return the sequential constraints since they define precedence. Parallel constraints are not considered
            -> One note is that parallel constraints where we have Context Dependencies will be transformed into sequential
            -> We have to check for Context Dependencies
                - If we find that a node has Context Dependencies we have to check with who it has
                    * If it has with a high-level node (like a goal) we have to find all of the instances it has this dependency
                    * If a parallel dependency between these tasks exist, change to sequential. If it is already sequential, nothing needs to be done
*/
void ConstraintManager::transform_at_constraints() {
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

                constraint_nodes_decompositions[n1.first] = n1_decompositions;
            }

            if(constraint_nodes_decompositions.find(n2.first) != constraint_nodes_decompositions.end()) {
                n2_decompositions = constraint_nodes_decompositions[n2.first];
            } else {
                n2_decompositions = find_decompositions(mission_decomposition,n2.first);

                constraint_nodes_decompositions[n2.first] = n2_decompositions;
            }

            unsigned int i,j;
            for(i=0;i<n1_decompositions.size();i++) {
                for(j=0;j<n2_decompositions.size();j++) {
                    Constraint new_c = generate_constraint(n1_decompositions.at(i), n2_decompositions.at(j), SEQ);

                    transformed_constraints.push_back(new_c);
                }
            }
        } else if(c.type == FB) {
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

            unsigned int i,j;
            for(i=0;i<n1_decompositions.size();i++) {
                for(j=0;j<n2_decompositions.size();j++) {
                    Constraint new_c = generate_constraint(n1_decompositions.at(i), n2_decompositions.at(j), FB);

                    transformed_constraints.push_back(new_c);
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

    
    for(Constraint c : transformed_constraints) {
        if(c.type == PAR) {
            string parallel_constraint_error = "Parallel constraints should not exist in final constraints";

            throw std::runtime_error(parallel_constraint_error);
        }
    }

    mission_constraints = transformed_constraints;
}

/*
    Function: generate_execution_constraints
    Objective: Generate execution constraints present in the mission

    @ Output: Void. The execution constraints will be added to the constraint vector
*/
void ConstraintManager::generate_execution_constraints() {
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
                                //bool can_unite = can_unite_decompositions(get<Decomposition>(source_decompositions.at(i).second.content),get<Decomposition>(target_decompositions.at(j).second.content),true);

                                //if(can_unite) {
                                    Constraint new_c = generate_constraint(source_decompositions.at(i), target_decompositions.at(j), NC);
                                    new_c.group = e.group;
                                    new_c.divisible = e.divisible;

                                    mission_constraints.push_back(new_c);
                                    non_coop_constraints.push_back(new_c);
                                //}
                            }
                        }
                    }
                }
            }
        }
    }
}

void ConstraintManager::trim_mission_constraints() {
    // 1st walk-through
    map<int,set<int>> first_nodes, second_nodes;
    map<int,set<int>> fb_first_nodes, fb_second_nodes;

    for(Constraint c : mission_constraints) {
        if(c.type == SEQ || c.type == FB) {
            int first_node = c.nodes_involved.first.first;
            int second_node = c.nodes_involved.second.first;

            if(c.type == SEQ) {
                first_nodes[second_node].insert(first_node);
                second_nodes[first_node].insert(second_node);
            } /*else {
                fb_first_nodes[second_node].insert(first_node);
                fb_second_nodes[first_node].insert(second_node);
            }*/
        }
    }

    // 2nd walk-through
    vector<Constraint>::iterator constraint_it = mission_constraints.begin();
    while(constraint_it != mission_constraints.end()) {
        if(constraint_it->type == SEQ || constraint_it->type == FB) {
            int first_node = constraint_it->nodes_involved.first.first;
            int second_node = constraint_it->nodes_involved.second.first;

            vector<int> v = {-1};
            if(constraint_it->type == SEQ) {
                if(second_nodes.find(first_node) != second_nodes.end() && first_nodes.find(second_node) != first_nodes.end()) {
                    set<int> first_node_set = second_nodes[first_node];
                    set<int> second_node_set = first_nodes[second_node];

                    std::set_intersection(first_node_set.begin(), first_node_set.end(), second_node_set.begin(), second_node_set.end(), v.begin());
                }
            } else {
                if(fb_second_nodes.find(first_node) != fb_second_nodes.end() && fb_first_nodes.find(second_node) != fb_first_nodes.end()) {
                    set<int> first_node_set = fb_second_nodes[first_node];
                    set<int> second_node_set = fb_first_nodes[second_node];

                    std::set_intersection(first_node_set.begin(), first_node_set.end(), second_node_set.begin(), second_node_set.end(), v.begin());
                }
            }

            if(v.at(0) != -1) {
                mission_constraints.erase(constraint_it);
            } else {
                constraint_it++;
            }
        } else {
            constraint_it++;
        }
    }
}

void ConstraintManager::check_execution_constraints() {
    for(Constraint c : mission_constraints) {
        if(c.type == NC) {
            Decomposition t1 = std::get<Decomposition>(c.nodes_involved.first.second.content);
            Decomposition t2 = std::get<Decomposition>(c.nodes_involved.second.second.content);

            bool robot_num_error = false;

            if(t1.at.fixed_robot_num) {
                if(t2.at.fixed_robot_num) {
                    int t1_robot_num = std::get<int>(t1.at.robot_num);
                    int t2_robot_num = std::get<int>(t2.at.robot_num);

                    if(t1_robot_num != t2_robot_num) {
                        robot_num_error = true;
                    }
                } else {
                    int t1_robot_num = std::get<int>(t1.at.robot_num);
                    pair<int,int> t2_robot_num = std::get<pair<int,int>>(t2.at.robot_num);

                    if(t1_robot_num < t2_robot_num.first || t1_robot_num > t2_robot_num.second) {
                        robot_num_error = true;
                    }
                }
            } else {
                if(t2.at.fixed_robot_num) {
                    pair<int,int> t1_robot_num = std::get<pair<int,int>>(t1.at.robot_num);
                    int t2_robot_num = std::get<int>(t2.at.robot_num);

                    if(t2_robot_num < t1_robot_num.first || t2_robot_num > t1_robot_num.second) {
                        robot_num_error = true;
                    }
                } else {
                    pair<int,int> t1_robot_num = std::get<pair<int,int>>(t1.at.robot_num);
                    pair<int,int> t2_robot_num = std::get<pair<int,int>>(t2.at.robot_num);

                    if(t1_robot_num.second < t2_robot_num.first || t2_robot_num.second < t1_robot_num.first) {
                        robot_num_error = true;
                    }
                }
            }

            if(robot_num_error) {
                string different_robot_num = "Wrong robot number for execution constrained tasks [" + t1.id + "] and [" + t2.id + "]";

                throw std::runtime_error(different_robot_num);
            }
        }
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