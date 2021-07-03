#include "fileoutputgeneratorfactory.hpp"

using namespace std;

shared_ptr<FileOutputGenerator> FileOutputGeneratorFactory::create_file_output_generator(GMGraph gm, ATGraph mission_decomposition, vector<ground_literal> world_state, vector<pair<ground_literal,variant<int,float>>> world_state_functions, pair<string,string> output, bool verbose) {
    shared_ptr<FileOutputGenerator> file_output_gen;

    if(output.second == "XML") {
        file_output_gen = std::make_shared<XMLOutputGenerator>();
        file_output_gen->set_file_output_generator_type(XMLFILEOUTGEN);
    } else if(output.second == "JSON") {
        file_output_gen = std::make_shared<JSONOutputGenerator>();
        file_output_gen->set_file_output_generator_type(JSONFILEOUTGEN);
    }
    
    file_output_gen->set_gm(gm);
    file_output_gen->set_mission_decomposition(mission_decomposition);
    file_output_gen->set_world_state(world_state);
    file_output_gen->set_world_state_functions(world_state_functions);  
    file_output_gen->set_output(output);
    file_output_gen->set_verbose(verbose);

    return file_output_gen;
}