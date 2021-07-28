#include "outputgenerator.hpp"

#include <iostream>

using namespace std;

void OutputGenerator::set_verbose(bool verb) {
    verbose = verb;
}

void OutputGenerator::set_mission_decomposition(ATGraph md) {
    mission_decomposition = md;
}

void OutputGenerator::set_gm(GMGraph g) {
    gm = g;
}

void OutputGenerator::set_world_state(vector<ground_literal> ws) {
    world_state = ws;
}

void OutputGenerator::set_world_state_functions(vector<pair<ground_literal,variant<int,float>>> wsf) {
    world_state_functions = wsf;
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