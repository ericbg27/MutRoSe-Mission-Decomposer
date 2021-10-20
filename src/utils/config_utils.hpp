#ifndef __CONFIG_UTILS
#define __CONFIG_UTILS

#include <string>
#include <map>
#include <variant>

#include "parsetree.hpp"

//################################### CONSTANTS DECLARATION ###############################################
const std::string world_db_config_key = "world_db";
const std::string db_type_key = "type";
const std::string db_file_type_key = "file_type";
const std::string db_path_key = "path";
const std::string db_xml_root_key = "xml_root";
const std::string db_unique_id_key = "unique_id";

const std::string output_config_key = "output";
const std::string output_type_key = "output_type";
const std::string output_file_path_key = "file_path";
const std::string output_file_type_key = "file_type";

const std::string location_types_config_key = "location_types";
const std::string agent_types_config_key = "agent_types";

const std::string type_mapping_config_key = "type_mapping";
const std::string hddl_type_key = "hddl_type";
const std::string ocl_type_key = "ocl_type";

const std::string var_mapping_config_key = "var_mapping";
const std::string task_id_key = "task_id";
const std::string map_var_mapping_key = "map";
const std::string gm_var_key = "gm_var";
const std::string hddl_var_key = "hddl_var";

//******************* Semantic Mapping related constants ***********************
const std::string semantic_mapping_config_key = "semantic_mapping";

// XML/JSON keys
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
const std::string mainentity_key = "main_entity";
const std::string relatedentity_key = "related_entity";
const std::string relationshiptype_key = "relationship_type";
const std::string attributename_key = "attribute_name";

// Accepted values
const std::string attribute_mapping_type = "attribute";
const std::string ownership_mapping_type = "ownership";
const std::string relationship_mapping_type = "relationship";
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