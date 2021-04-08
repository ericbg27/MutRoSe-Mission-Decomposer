#include "config.hpp"

#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/optional/optional.hpp>

using namespace std;

SemanticMapping::SemanticMapping(string mapping_type, string mapped_type) {
    this->mapping_type = mapping_type;
    this->mapped_type = mapped_type;
}

SemanticMapping::SemanticMapping() {
    
}

void SemanticMapping::add_prop(string prop_name, variant<string, predicate_definition> prop_val) {
    this->mapping_props[prop_name] = prop_val;
}

string SemanticMapping::get_mapping_type() {
    return this->mapping_type;
}

string SemanticMapping::get_mapped_type() {
    return this->mapped_type;
}

map<string, variant<string, predicate_definition>> SemanticMapping::get_mapping_props() {
    return this->mapping_props;
}

void SemanticMapping::set_mapping_type(string mapping_type) {
    this->mapping_type = mapping_type;
}

void SemanticMapping::set_mapped_type(string mapped_type) {
    this->mapped_type = mapped_type;
}

VariableMapping::VariableMapping(string task_id, string hddl_var, string gm_var) {
    this->task_id = task_id;
    this->hddl_var = hddl_var;
    this->gm_var = gm_var;
}

string VariableMapping::get_task_id() {
    return this->task_id;
}

string VariableMapping::get_hddl_var() {
    return this->hddl_var;
}

string VariableMapping::get_gm_var() {
    return this->gm_var;
}

variant<string,predicate_definition> SemanticMapping::get_prop(string prop) {
    if(this->mapping_props.find(prop) != this->mapping_props.end()) {
        return this->mapping_props[prop];
    } else {
        string no_prop_error = "No property " + prop + " found in semantic mapping";
        throw std::runtime_error(no_prop_error);
    }
}

void SemanticMapping::operator=(SemanticMapping sm) {
    this->mapped_type = sm.get_mapped_type();
    this->mapping_type = sm.get_mapping_type();

    map<string, variant<string, predicate_definition>>::iterator prop_it;

    map<string, variant<string, predicate_definition>> prop_map = sm.get_mapping_props();
    for(prop_it = prop_map.begin();prop_it != prop_map.end();++prop_it) {
        if(holds_alternative<string>(prop_it->second)) {
            this->add_prop(prop_it->first,get<string>(prop_it->second));
        } else if(holds_alternative<predicate_definition>(prop_it->second)) {
            this->add_prop(prop_it->first,get<predicate_definition>(prop_it->second));
        }
    }
}

map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> parse_configuration_file(string filename) {
    map<string,variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> config_info;
    
    pt::ptree config_root;
    pt::read_xml(filename, config_root);

    BOOST_FOREACH(pt::ptree::value_type& config, config_root.get_child("configuration")) {
        //Read Robot and World Database Info
        vector<string> databases {"robots_db","world_db"};
        for(string db : databases) {
            if(config.first == db) {
                string db_type = config.second.get<string>("type");

                if(db_type == "file") { //Only available type for the moment
                    string file_type = config.second.get<string>("file_type");

                    if(file_type == "xml") {
                        string db_path = config.second.get<string>("path");
                        string xml_root = config.second.get<string>("xml_root");

                        map<string,string> map_db;
                        map_db["type"] = db_type;
                        map_db["file_type"] = file_type;
                        map_db["path"] = db_path;
                        map_db["xml_root"] = xml_root;

                        config_info[db] = map_db;
                    }
                }
            }
        }

        /*
            Read output file path and type

            -> FOR NOW THE ONLY ALLOWED TYPE IS XML
        */
        if(config.first == "output") {
            string output_type = config.second.get<string>("file_type");
            string output_path = config.second.get<string>("file_path");

            config_info["output"] = make_pair(output_path,output_type);
        }

        //Read Possible High Level Location Types
        if(config.first == "location_types") {
            vector<string> l_types;

            BOOST_FOREACH(pt::ptree::value_type& type, config.second) {
                string type_name = type.second.get<string>("");
                l_types.push_back(type_name);
            }

            
            config_info["location_types"] = l_types;
        }

        //Read Type Mappings
        if(config.first == "type_mapping") {
            map<string,string> type_mapping;
            BOOST_FOREACH(pt::ptree::value_type& mapping, config.second) {
                string hddl_type = mapping.second.get<string>("hddl_type");
                string actual_type = mapping.second.get<string>("ocl_type");

                type_mapping[actual_type] = hddl_type;
            }

            config_info["type_mapping"] = type_mapping;
        }

        //Read Variable Mappings
        if(config.first == "var_mapping") {
            vector<VariableMapping> var_mappings;
            BOOST_FOREACH(pt::ptree::value_type& mapping, config.second) {
                string task_id, hddl_var, gm_var;

                task_id = mapping.second.get<string>("task_id");
                hddl_var = mapping.second.get<string>("hddl_var");
                gm_var  = mapping.second.get<string>("gm_var");

                VariableMapping var_map(task_id,hddl_var,gm_var);

                var_mappings.push_back(var_map);
            }
            
            config_info["var_mapping"] = var_mappings;
        }

        //Read Semantic Mappings
        if(config.first == "semantic_mapping") {
            vector<SemanticMapping> mappings;
            BOOST_FOREACH(pt::ptree::value_type& mapping, config.second) {
                string mapping_type = mapping.second.get<string>("type");
                string mapped_type = mapping.second.get<string>("mapped_type");
                SemanticMapping sm(mapping_type, mapped_type);

                if(mapping_type == "attribute") {
                    sm.add_prop("name", mapping.second.get<string>("name"));
                    sm.add_prop("relation", mapping.second.get<string>("relation"));
                    if(mapping.second.get<string>("relation") != "robot") {
                        sm.add_prop("belongs_to", mapping.second.get<string>("belongs_to"));
                    }
                }

                if(mapped_type == "predicate") {
                    boost::optional<string> p_type = mapping.second.get_optional<string>("predicate_type");
                    if(p_type) {
                        sm.add_prop("predicate_type", p_type.get());
                    }

                    predicate_definition pred;

                    pt::ptree map = mapping.second.get_child("map"); 
                        
                    pred.name = map.get<string>("pred");
                    pt::ptree sorts_attr = map.get_child("arg_sorts").get_child("<xmlattr>");
                    int sorts_number = sorts_attr.get<int>("number");

                    stringstream ss(map.get_child("arg_sorts").data());
                    for(int i = 0;i < sorts_number;i++) {
                        string sort_name;
                        ss >> sort_name;

                        pred.argument_sorts.push_back(sort_name);
                    }

                    sm.add_prop("map", pred);
                }

                mappings.push_back(sm);
            }

            config_info["semantic_mapping"] = mappings;
        }
    }

    return config_info;
}