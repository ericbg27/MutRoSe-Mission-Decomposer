#include "ihtn_generator_utils.hpp"

using namespace std;

map<string,CompleteDecompositionPath> map_complete_decompositions(ATGraph mission_decomposition, map<string,vector<CompleteDecompositionPath>> at_complete_decomposition_paths) {
    map<string,CompleteDecompositionPath> path_map;

    ATGraph::vertex_iterator vi, vend;
    for(boost::tie(vi,vend) = vertices(mission_decomposition); vi != vend; ++vi) {
        if(mission_decomposition[*vi].node_type == DECOMPOSITION) {
            Decomposition d = std::get<Decomposition>(mission_decomposition[*vi].content);

            int path_index = stoi(d.id.substr(d.id.find("|")+1))-1;
            CompleteDecompositionPath path = at_complete_decomposition_paths[d.at.at.name].at(path_index);

            path_map[d.id] = path;
        }
    }

    return path_map;
}