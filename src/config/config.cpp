#include "config.hpp"

#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/optional/optional.hpp>

using namespace std;

set<string> accepted_output_file_types = {"XML", "JSON"};

map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> ConfigManager::parse_configuration_file(string filename) {
    string cfg_filetype = "XML";
	if(filename.rfind(".") != std::string::npos) {
		string aux = filename.substr(filename.rfind(".")+1);
		std::transform(aux.begin(), aux.end(), aux.begin(), ::toupper);

		if(accepted_file_types.find(aux) != accepted_file_types.end()) {
			cfg_filetype = aux;
		}
	}

	if(cfg_filetype == "XML") {
		parse_xml_configuration_file(filename);
	} else if(cfg_filetype == "JSON") {
		parse_json_configuration_file(filename);
	}

    return config_info;
}

void ConfigManager::parse_xml_configuration_file(string filename) {    
    pt::ptree config_root;
    pt::read_xml(filename, config_root);

    BOOST_FOREACH(pt::ptree::value_type& config, config_root.get_child("configuration")) {
        //Read Databases Info
        vector<string> databases {"world_db"};
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

                        boost::optional<string> unique_id_attr = config.second.get_optional<string>("unique_id");

                        string unique_id = "";
                        if(unique_id_attr) {
                            unique_id = unique_id_attr.get();
                        }

                        if(unique_id == "") {
                            unique_id = "name";
                            string unique_id_missing_warning = "\n\n###################################################################################################################################\n"; 
                            unique_id_missing_warning += "WARNING: attribute [unique_id] is missing for XML file type database [" + db + "]. Defaulting its value to 'name'.";
                            unique_id_missing_warning += "\n###################################################################################################################################\n";
                        }

                        map<string,string> map_db;

                        map_db["type"] = db_type;
                        map_db["file_type"] = file_type;
                        map_db["path"] = db_path;
                        map_db["xml_root"] = xml_root;

                        if(db == "world_db") {
                            map_db["unique_id"] = unique_id;
                        }

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
                    string unsupported_file_type_error = "File type [" + file_type + "] is not supported";

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
                string mapping_type = mapping.second.get<string>(type_key);
                std::transform(mapping_type.begin(),mapping_type.end(),mapping_type.begin(),::tolower);
                string mapped_type = mapping.second.get<string>(mappedtype_key);
                std::transform(mapped_type.begin(),mapped_type.end(),mapped_type.begin(),::tolower);
                SemanticMapping sm(mapping_type, mapped_type);

                if(mapping_type == attribute_mapping_type) {
                    sm.add_prop(name_key, mapping.second.get<string>(name_key));

                    string relation_type = mapping.second.get<string>(relatesto_key);
                    std::transform(relation_type.begin(),relation_type.end(),relation_type.begin(),::tolower);
                    if(relation_type == "robot") {
                        sm.add_prop(relatesto_key, relation_type);
                    } else {
                        sm.add_prop(relatesto_key, mapping.second.get<string>(relatesto_key));
                        sm.add_prop(belongsto_key, mapping.second.get<string>(belongsto_key));
                    }
                } else if(mapping_type == ownership_mapping_type) {
                    sm.add_prop(owner_key, mapping.second.get<string>(owner_key));
                    sm.add_prop(owned_key, mapping.second.get<string>(owned_key));
                    
                    string relationship_type = mapping.second.get<string>(relationshiptype_key);
                    sm.add_prop(relationshiptype_key, relationship_type);
                    if(relationship_type == attribute_relationship_type) {
                        sm.add_prop(attributename_key, mapping.second.get<string>(attributename_key));
                    }

                    sm.add_prop(belongsto_key, mapping.second.get<string>(belongsto_key));
                } else if(mapping_type == relationship_mapping_type) {
                    sm.add_prop(mainentity_key, mapping.second.get<string>(mainentity_key));
                    sm.add_prop(relatedentity_key, mapping.second.get<string>(relatedentity_key));

                    string relationship_type = mapping.second.get<string>(relationshiptype_key);
                    sm.add_prop(relationshiptype_key, relationship_type);
                    if(relationship_type == attribute_relationship_type) {
                        sm.add_prop(attributename_key, mapping.second.get<string>(attributename_key));
                    }

                    sm.add_prop(belongsto_key, mapping.second.get<string>(belongsto_key));
                }

                if(mapped_type == predicate_mapped_type || mapped_type == function_mapped_type) {
                    boost::optional<string> p_type = mapping.second.get_optional<string>(predicatetype_key);
                    if(p_type) {
                        sm.add_prop(predicatetype_key, p_type.get());
                    }

                    predicate_definition pred;

                    pt::ptree map = mapping.second.get_child(map_key); 
                        
                    pred.name = map.get<string>(pred_map_key);
                    pt::ptree sorts_attr = map.get_child(argsorts_map_key).get_child("<xmlattr>");
                    int sorts_number = sorts_attr.get<int>(argsortsnumber_map_key);

                    stringstream ss(map.get_child(argsorts_map_key).data());
                    for(int i = 0;i < sorts_number;i++) {
                        string sort_name;
                        ss >> sort_name;

                        pred.argument_sorts.push_back(sort_name);
                    }

                    sm.add_prop(map_key, pred);
                }

                mappings.push_back(sm);
            }

            config_info["semantic_mapping"] = mappings;
        }
    }
}

void ConfigManager::parse_json_configuration_file(std::string filename) {
    pt::ptree config_root;
    pt::read_json(filename, config_root);

    BOOST_FOREACH(pt::ptree::value_type& config, config_root) {
        //Read Databases Info
        vector<string> databases {"world_db"};
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

                        boost::optional<string> unique_id_attr = config.second.get_optional<string>("unique_id");

                        string unique_id = "";
                        if(unique_id_attr) {
                            unique_id = unique_id_attr.get();
                        }

                        if(unique_id == "") {
                            unique_id = "name";
                            string unique_id_missing_warning = "\n\n###################################################################################################################################\n"; 
                            unique_id_missing_warning += "WARNING: attribute [unique_id] is missing for XML file type database [" + db + "]. Defaulting its value to 'name'.";
                            unique_id_missing_warning += "\n###################################################################################################################################\n";
                        }

                        map<string,string> map_db;

                        map_db["type"] = db_type;
                        map_db["file_type"] = file_type;
                        map_db["path"] = db_path;
                        map_db["xml_root"] = xml_root;

                        if(db == "world_db") {
                            map_db["unique_id"] = unique_id;
                        }

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
                pt::ptree map_ptree = mapping.second.get_child("map");
 
                BOOST_FOREACH(pt::ptree::value_type& child, map_ptree) {
                    string hddl_var, gm_var;

                    gm_var = child.second.get<string>("gm_var");
                    hddl_var = child.second.get<string>("hddl_var");

                    VariableMapping v_map(task_id,hddl_var,gm_var);

                    var_mappings.push_back(v_map);
                }
            }
            
            config_info["var_mapping"] = var_mappings;
        }

        //Read Semantic Mappings
        if(config.first == "semantic_mapping") {
            vector<SemanticMapping> mappings;
            BOOST_FOREACH(pt::ptree::value_type& mapping, config.second) {
                string mapping_type = mapping.second.get<string>(type_key);
                std::transform(mapping_type.begin(),mapping_type.end(),mapping_type.begin(),::tolower);
                string mapped_type = mapping.second.get<string>(mappedtype_key);
                std::transform(mapped_type.begin(),mapped_type.end(),mapped_type.begin(),::tolower);
                SemanticMapping sm(mapping_type, mapped_type);

                if(mapping_type == attribute_mapping_type) {
                    sm.add_prop(name_key, mapping.second.get<string>(name_key));

                    string relation_type = mapping.second.get<string>(relatesto_key);
                    std::transform(relation_type.begin(),relation_type.end(),relation_type.begin(),::tolower);
                    if(relation_type == "robot") {
                        sm.add_prop(relatesto_key, relation_type);
                    } else {
                        sm.add_prop(relatesto_key, mapping.second.get<string>(relatesto_key));
                        sm.add_prop(belongsto_key, mapping.second.get<string>(belongsto_key));
                    }
                } else if(mapping_type == ownership_mapping_type) {
                    sm.add_prop(owner_key, mapping.second.get<string>(owner_key));
                    sm.add_prop(owned_key, mapping.second.get<string>(owned_key));
                    
                    string relationship_type = mapping.second.get<string>(relationshiptype_key);
                    sm.add_prop(relationshiptype_key, relationship_type);
                    if(relationship_type == attribute_relationship_type) {
                        sm.add_prop(attributename_key, mapping.second.get<string>(attributename_key));
                    }

                    sm.add_prop(belongsto_key, mapping.second.get<string>(belongsto_key));
                } else if(mapping_type == relationship_mapping_type) {
                    sm.add_prop(mainentity_key, mapping.second.get<string>(mainentity_key));
                    sm.add_prop(relatedentity_key, mapping.second.get<string>(relatedentity_key));

                    string relationship_type = mapping.second.get<string>(relationshiptype_key);
                    sm.add_prop(relationshiptype_key, relationship_type);
                    if(relationship_type == attribute_relationship_type) {
                        sm.add_prop(attributename_key, mapping.second.get<string>(attributename_key));
                    }

                    sm.add_prop(belongsto_key, mapping.second.get<string>(belongsto_key));
                }

                if(mapped_type == predicate_mapped_type || mapped_type == function_mapped_type) {
                    boost::optional<string> p_type = mapping.second.get_optional<string>(predicatetype_key);
                    if(p_type) {
                        sm.add_prop(predicatetype_key, p_type.get());
                    }

                    predicate_definition pred;

                    pt::ptree map = mapping.second.get_child(map_key); 
                        
                    pred.name = map.get<string>(pred_map_key);

                    pt::ptree sorts_attr = map.get_child(argsorts_map_key);
                    BOOST_FOREACH(pt::ptree::value_type& pred_sort, sorts_attr) {
                        string sort_name = pred_sort.second.get_value<string>();
                        
                        pred.argument_sorts.push_back(sort_name);
                    }

                    sm.add_prop(map_key, pred);
                }

                mappings.push_back(sm);
            }

            config_info["semantic_mapping"] = mappings;
        }
    }
}