#include "config.hpp"

#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/optional/optional.hpp>

using namespace std;

set<string> accepted_output_file_types = {"XML"};

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
                boost::optional<string> db_type_attr = config.second.get_optional<string>("type");

                string db_type = "";
                if(db_type_attr) {
                    db_type = db_type_attr.get();
                    std::transform(db_type.begin(),db_type.end(),db_type.begin(),::toupper);
                } else {
                    string undefined_db_type_error = "Database type for [" + db + "] was not defined";

                    throw std::runtime_error(undefined_db_type_error);
                }

                if(db_type == "FILE") { 
                    boost::optional<string> file_type_attr = config.second.get_optional<string>("file_type");

                    string file_type = "";
                    if(file_type_attr) {
                        file_type = file_type_attr.get();
                        std::transform(file_type.begin(),file_type.end(),file_type.begin(),::toupper);
                    } else {
                        string undefined_file_type_error = "Database file type for [" + db + "] was not defined";

                        throw std::runtime_error(undefined_file_type_error);
                    }

                    vector<string> undefined_attrs;
                    if(file_type == "XML") {
                        
                        boost::optional<string> db_path_attr = config.second.get_optional<string>("path");

                        string db_path = "";
                        if(db_path_attr) {
                            db_path = db_path_attr.get();
                        } else {
                            undefined_attrs.push_back("path");
                        }

                        boost::optional<string> xml_root_attr = config.second.get_optional<string>("xml_root");

                        string xml_root = "";
                        if(xml_root_attr) {
                            xml_root = xml_root_attr.get();
                        } else {
                            string xml_root_missing_warning = "\n\n###################################################################################################################################\n"; 
                            xml_root_missing_warning += "WARNING: attribute [xml_root] is missing for XML file type database [" + db + "]. Unexpected behaviors (or even errors) may happen!";
                            xml_root_missing_warning += "\n###################################################################################################################################\n";

                            std::cout << xml_root_missing_warning << std::endl;

                            std::cout << "[PRESS ENTER TO CONTINUE OR CTRL+C TO END]" << std::endl;

                            getchar();
                        }

                        map<string,string> map_db;

                        map_db["type"] = db_type;
                        map_db["file_type"] = file_type;
                        map_db["path"] = db_path;
                        map_db["xml_root"] = xml_root;

                        config_info[db] = map_db;
                    } else {
                        string unsupported_file_database_type_error = "Database file type [" + file_type + "] is not supported";

                        throw std::runtime_error(unsupported_file_database_type_error);
                    }

                    if(undefined_attrs.size() > 0) {
                        string undefined_db_attrs_error = "Missing attributes ";

                        unsigned int index = 1;
                        for(string attr : undefined_attrs) {
                            if(index == undefined_attrs.size()) {
                                undefined_db_attrs_error += "[" + attr + "] ";
                            } else {
                                undefined_db_attrs_error += "[" + attr + "], ";
                            }
                        }
                        
                        undefined_db_attrs_error += "in " + db + " definition";

                        throw std::runtime_error(undefined_db_attrs_error);
                    }
                } else {
                    string unsupported_db_type_error = "Database type [" + db_type + "] is not supported";

                    throw std::runtime_error(unsupported_db_type_error);
                }
            }
        }

        /*
            Read output file path and type

            -> FOR NOW THE ONLY ALLOWED TYPE IS XML
        */
        if(config.first == "output") {
            vector<string> output_info;

            string output_type = config.second.get<string>("output_type");
            std::transform(output_type.begin(),output_type.end(),output_type.begin(),::toupper);
            output_info.push_back(output_type);

            if(output_type == "FILE") {
                string file_type = config.second.get<string>("file_type");
                std::transform(file_type.begin(),file_type.end(),file_type.begin(),::toupper);

                if(accepted_output_file_types.find(file_type) == accepted_output_file_types.end()) {
                    string unsupported_file_type_error = "File type [" + output_type + "] is not supported";

                    throw std::runtime_error(unsupported_file_type_error);
                }

                string output_path = config.second.get<string>("file_path");
                
                output_info.push_back(output_path);
                output_info.push_back(file_type);
            } else {
                string unsupported_output_type_error = "Output type [" + output_type + "] is not supported";

                throw std::runtime_error(unsupported_output_type_error);
            }

            config_info["output"] = output_info;
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
                string task_id = mapping.second.get<string>("task_id");
                BOOST_FOREACH(pt::ptree::value_type& child, mapping.second) {
                    if(child.first == "map") {
                        string hddl_var, gm_var;

                        pt::ptree var_map_attrs = child.second.get_child("<xmlattr>");

                        gm_var = var_map_attrs.get<string>("gm_var");
                        hddl_var = var_map_attrs.get<string>("hddl_var");

                        VariableMapping v_map(task_id,hddl_var,gm_var);

                        var_mappings.push_back(v_map);
                    }
                }
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

                    string relation_type = mapping.second.get<string>("relation");
                    std::transform(relation_type.begin(),relation_type.end(),relation_type.begin(),::tolower);
                    if(relation_type == "robot") {
                        sm.add_prop("relation", relation_type);
                    } else {
                        sm.add_prop("relation", mapping.second.get<string>("relation"));
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