#include "fileoutputgeneratorfactory.hpp"

using namespace std;

shared_ptr<FileOutputGenerator> FileOutputGeneratorFactory::create_file_output_generator(GMGraph gm, ATGraph mission_decomposition, vector<ground_literal> world_state, vector<pair<ground_literal,int>> world_state_functions, pair<string,string> output) {
    shared_ptr<FileOutputGenerator> file_output_gen;

    if(output.second == "XML") {
        file_output_gen = std::make_shared<XMLOutputGenerator>();
        file_output_gen->set_file_output_generator_type(XMLFILEOUTGEN);
    }
    
    file_output_gen->set_gm(gm);
    file_output_gen->set_mission_decomposition(mission_decomposition);
    file_output_gen->set_world_state(world_state);
    file_output_gen->set_world_state_functions(world_state_functions);  
    file_output_gen->set_output(output);

    return file_output_gen;
}