#ifndef __CONFIG_UTILS
#define __CONFIG_UTILS

#include <string>
#include <map>
#include <variant>

#include "parsetree.hpp"

//################################### CONSTANTS DECLARATION ###############################################

//******************* Semantic Mapping related constants ***********************
// XML keys
const std::string type_key = "type";
const std::string name_key = "name";
const std::string relatesto_key = "relates_to";
const std::string mappedtype_key = "mapped_type";
const std::string belongsto_key = "belongs_to";
const std::string map_key = "map";
const std::string pred_map_key = "pred";
const std::string argsorts_map_key = "arg_sorts";
const std::string argsortsnumber_map_key = "number";
const std::string predicatetype_key = "predicate_type";
const std::string owner_key = "owner";
const std::string owned_key = "owned";
const std::string relationshiptype_key = "relationship_type";
const std::string attributename_key = "attribute_name";

// Accepted values
const std::string attribute_mapping_type = "attribute";
const std::string ownership_mapping_type = "ownership";
const std::string predicate_mapped_type = "predicate";
const std::string function_mapped_type = "function";
const std::string attribute_relationship_type = "attribute";
//*******************************************************************************

//##########################################################################################################

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

        bool has_prop(std::string prop_name);

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

#endif