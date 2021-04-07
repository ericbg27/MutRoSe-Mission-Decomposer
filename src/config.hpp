#ifndef __CONFIG
#define __CONFIG

#include <map>
#include <variant>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "parsetree.hpp"

namespace pt = boost::property_tree;

//Class for the semantic mapping part of the configuration
class SemanticMapping { //The only type of semantic mapping we have
    public:
        SemanticMapping(std::string mapping_type, std::string mapped_type);
        SemanticMapping();
        SemanticMapping(const SemanticMapping& sm) = default;

        void add_prop(std::string prop_name, std::variant<std::string, predicate_definition> prop_val);
        
        std::string get_mapping_type();
        std::string get_mapped_type();
        std::map<std::string, std::variant<std::string, predicate_definition>> get_mapping_props();

        void set_mapping_type(std::string mapping_type);
        void set_mapped_type(std::string mapped_type);

        std::variant<std::string,predicate_definition> get_prop(std::string prop_name);

        void operator=(SemanticMapping sm);
            
    private:
        std::string mapping_type;
        std::string mapped_type;
        std::map<std::string, std::variant<std::string, predicate_definition>> mapping_props;
};

class VariableMapping {
    public:
        VariableMapping(std::string task_id, std::string hddl_var, std::string gm_var);

        std::string get_task_id();
        std::string get_hddl_var();
        std::string get_gm_var();

    private:
        std::string task_id;
        std::string hddl_var;
        std::string gm_var;
};

// Afterwards we can use a variant here, depending on what we want to give to the main file
std::map<std::string, std::variant<std::map<std::string,std::string>, std::vector<std::string>, std::vector<SemanticMapping>, std::vector<VariableMapping>, std::pair<std::string,std::string>>> parse_configuration_file(std::string filename);

#endif