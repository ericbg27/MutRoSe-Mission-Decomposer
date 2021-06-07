#ifndef __CONFIG
#define __CONFIG

#include <map>
#include <variant>
#include <string>
#include <set>

#include <boost/property_tree/ptree.hpp>

#include "../utils/config_utils.hpp"

namespace pt = boost::property_tree;

const std::set<std::string> accepted_file_types = {"XML", "JSON"};

class ConfigManager {
    public:
        std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, std::pair<std::string,std::string>>> parse_configuration_file(std::string filename);

        void parse_xml_configuration_file(std::string filename);
        void parse_json_configuration_file(std::string filename);

    private:
        std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, std::pair<std::string,std::string>>> config_info;    
};

#endif