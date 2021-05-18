#ifndef __CONFIG
#define __CONFIG

#include <map>
#include <variant>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "../utils/config_utils.hpp"

namespace pt = boost::property_tree;

// Afterwards we can use a variant here, depending on what we want to give to the main file
std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, std::pair<std::string,std::string>>> parse_configuration_file(std::string filename);

#endif