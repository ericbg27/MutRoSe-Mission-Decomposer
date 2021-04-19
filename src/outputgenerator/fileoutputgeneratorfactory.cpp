#include "fileoutputgeneratorfactory.hpp"

using namespace std;

std::shared_ptr<FileOutputGenerator> FileOutputGeneratorFactory::create_file_output_generator(string file_type) {
    shared_ptr<FileOutputGenerator> file_output_gen;

    if(file_type == "XML") {
        file_output_gen = std::make_shared<XMLOutputGenerator>();
        file_output_gen->set_file_output_generator_type(XMLFILEOUTGEN);
    }

    return file_output_gen;
}