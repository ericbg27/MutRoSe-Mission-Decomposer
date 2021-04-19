#ifndef __FILEOUTPUTGENERATORFACTORY
#define __FILEOUTPUTGENERATORFACTORY

#include "xmloutputgenerator.hpp"
#include "outputgenerator.hpp"

class FileOutputGeneratorFactory {
    public:
        std::shared_ptr<FileOutputGenerator> create_file_output_generator(std::string file_type);
};

#endif