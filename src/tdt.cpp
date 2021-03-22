#include <iostream>
#include <set>

#include "tdt.hpp"
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

TDT::TDT(task root_abstract_task, vector<task> a_tasks, vector<task> p_tasks, vector<method> ms) {
    /*
        Here we are generating a TDT. We have two options:
            - Check methods and actions preconditions alongside actions effects in order to determine when to finish building the tree
                - This needs to be done when we have ATs that form a cycle, so that we know when to stop the cycle
                - We need to have a max number of cycles allowed, possibly given by the user with a default value
                    - If the max number of cyclic iterations is reached and we did not got out of the cycle, no feasible generation of the path exists
                        - If we have OR paths, the tree can still exist
                    - This idea will use the principle of the recursive decomposition function, but this time we are building the tree
            - We can generate a TDG and deal with cyclic iterations only when finding possible decompositions
    */
    /*
        Let's generate a TDG and deal with the cycles problem only when generating possible decompositions
    */
    //Add root task node
    root = root_abstract_task;
    abstract_tasks = a_tasks;
    primitive_tasks = p_tasks;
    methods = ms;

    NodeData n;

    n.type = AT;
    n.t = root_abstract_task;
    n.parent = -1;

    vertex_t id = boost::add_vertex(n,tdt);

    tdt[id].id = id;
    n.id = id;

    add_task_path(n);
}
/*
In this simple version we don't return possible partial orderings. This means that we assume tasks to be totally unordered
*/
vector<vector<task>> TDT::retrieve_possible_decompositions() {
    vector<int> depth_first_nodes = DFS_visit();

    vector<vector<task>> paths;

    paths = decomposition_recursion(depth_first_nodes,0);

    return paths;
}

vector<vector<task>> TDT::decomposition_recursion(vector<int> dfs_nodes, int current_pos) {
    int node = dfs_nodes.at(current_pos);

    NodeData n = tdt[node];

    vector<vector<task>> generated_paths;
    if(n.type == M) {
        for(int c : n.children) {
            std::vector<int>::iterator it = std::find(dfs_nodes.begin()+current_pos,dfs_nodes.end(),c);
            int c_pos = std::distance(dfs_nodes.begin()+current_pos,it) + current_pos;
            vector<vector<task>> aux = decomposition_recursion(dfs_nodes,c_pos);
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
    } else if(n.type == AT) {
        for(int c : n.children) {
            std::vector<int>::iterator it = std::find(dfs_nodes.begin()+current_pos,dfs_nodes.end(),c);
            int c_pos = std::distance(dfs_nodes.begin()+current_pos,it) + current_pos;
            vector<vector<task>> aux = decomposition_recursion(dfs_nodes,c_pos);
            generated_paths.insert(generated_paths.end(),aux.begin(),aux.end());
        }
    } else if(n.type == PT) {
        vector<task> t;
        t.push_back(n.t);
        generated_paths.push_back(t);
    }

    return generated_paths;
}

void TDT::add_method_path(NodeData m) {
    for(auto ps : m.m.ps) {
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

        vertex_t id = boost::add_vertex(t_node,tdt);

        tdt[id].id = id;

        t_node.id = id;

        add_edge(m.id, tdt[id].id);

        add_task_path(t_node);
    }
}

void TDT::add_task_path(NodeData t) {
    //Find methods that decompose the task and from them generate the complete path resulting from their decomposition
    if(t.type != PT) {
        for(method m : methods) {
            if(m.at == t.t.name) {
                NodeData m_node;
                m_node.type = M;
                m_node.m = m;
                
                vertex_t id = boost::add_vertex(m_node,tdt);

                tdt[id].id = id;

                m_node.id = id;

                add_edge(t.id, tdt[id].id);

                add_method_path(m_node);
            }
        }
    }
}

void TDT::add_edge(int s_id, int t_id) {
    EData edge;

    edge.source = s_id;
    edge.target = t_id;

    if(tdt[s_id].type == M) {
        edge.type = OOR;
    } else {
        edge.type = AAND;
    }

    tdt[s_id].children.push_back(t_id);
    tdt[t_id].parent = s_id;

     boost::add_edge(s_id,t_id,edge,tdt);
}

vector<int> TDT::DFS_visit() {
    auto indexmap = boost::get(boost::vertex_index, tdt);
    auto colormap = boost::make_vector_property_map<boost::default_color_type>(indexmap);

    TDTDFSVisitor vis;
    boost::depth_first_search(tdt, vis, colormap, 0);

    std::vector<int> vctr = vis.GetVector();

    return vctr;
}

void TDT::print_edges() {
    boost::graph_traits<TDTraph>::edge_iterator it, end;
	
	for(tie(it,end) = boost::edges(tdt);it != end;++it) {
        NodeData s = tdt[boost::source(*it, tdt)];
        NodeData t = tdt[boost::target(*it, tdt)];

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