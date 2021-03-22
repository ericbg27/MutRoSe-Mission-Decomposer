#ifndef __CONFIG
#define __CONFIG

#include <map>
#include <variant>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "parsetree.hpp"

using namespace std;

namespace pt = boost::property_tree;

//Class for the semantic mapping part of the configuration
class SemanticMapping { //The only type of semantic mapping we have
    public:
        SemanticMapping(string mapping_type, string mapped_type);
        SemanticMapping();
        SemanticMapping(const SemanticMapping& sm) = default;

        void add_prop(string prop_name, variant<string, predicate_definition> prop_val);
        
        string get_mapping_type();
        string get_mapped_type();
        map<string, variant<string, predicate_definition>> get_mapping_props();

        void set_mapping_type(string mapping_type);
        void set_mapped_type(string mapped_type);

        variant<string,predicate_definition> get_prop(string prop_name);

        void operator=(SemanticMapping sm);
            
    private:
        string mapping_type;
        string mapped_type;
        map<string, variant<string, predicate_definition>> mapping_props;
};

class VariableMapping {
    public:
        VariableMapping(string task_id, string hddl_var, string gm_var);

        string get_task_id();
        string get_hddl_var();
        string get_gm_var();

    private:
        string task_id;
        string hddl_var;
        string gm_var;
};

// Afterwards we can use a variant here, depending on what we want to give to the main file
map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> parse_configuration_file(string filename);

#endif