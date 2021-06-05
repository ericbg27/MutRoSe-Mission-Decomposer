#include "fileknowledgemanager.hpp"

#include <stack>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>

using namespace std;

/*
    Function: construct_knowledge_base
    Objective: Construct a knowledge base given the database name and the configuration map

    @ Input 1: The database name
    @ Input 2: The configuration map, obtained from the parsing of the configuration file
    @ Output: The constructed KnowledgeBase object

    NOTES: -> For now, the only type allowed is XML file
		   -> When (and if) more types are allowed we need to delegate the task of opening these files to functions
		    passing the configuration file as a function parameter
*/ 
void FileKnowledgeManager::construct_knowledge_base(string db_name, map<string, variant<map<string,string>, vector<string>, vector<SemanticMapping>, vector<VariableMapping>, pair<string,string>>> cfg) {
	string db_file_type = std::get<map<string,string>>(cfg[db_name])["file_type"];
    string db_root = "";
        
    if(db_file_type == "XML") {
        pt::ptree db_knowledge;
        pt::read_xml(std::get<map<string,string>>(cfg[db_name])["path"], db_knowledge);
                    
        db_root = std::get<map<string,string>>(cfg[db_name])["xml_root"];

        if(db_name == "world_db") {
            XMLKnowledgeBase wk(db_name, db_knowledge, db_root);

            world_knowledge = make_shared<XMLKnowledgeBase>(wk);
        }
    } else {
        string unsupported_db_file_type_error = "File type " + db_file_type + " is not supported as a database";

        throw std::runtime_error(unsupported_db_file_type_error);
    }
}

/*
    Function: initialize_objects
    Objective: Initalize the objects on the sorts map based on the knowledge bases (world and robot)

    @ Input 1: The reference to the sorts map
    @ Input 2: The high-level location type (for now it is only one)
    @ Input 3: The map of abstract task instances
    @ Input 4: The mapping between HDDL types and OCL types
    @ Output: Void. The sorts are initialized
*/ 
void FileKnowledgeManager::initialize_objects(map<string,set<string>>& sorts, vector<string> high_level_loc_types, map<string,vector<AbstractTask>>& at_instances) {
    pt::ptree worlddb_root;

    if(world_knowledge->get_root_key() == "") {
        worlddb_root = world_knowledge->get_knowledge(); 
    } else {
        worlddb_root = world_knowledge->get_knowledge().get_child(world_knowledge->get_root_key());
    }
    
    /*
        Here we will add locations. We need to be careful: we need to check what is the type of the location!

        -> For now this means that we need to check if that location is of location_type and present in the World model
        -> If it isn't, it will be added as default location type
    */
    set<string> world_locations; //Locations that must be declared in the DSL

	BOOST_FOREACH(pt::ptree::value_type& child, worlddb_root) {
        vector<string>::iterator loc_it = std::find(high_level_loc_types.begin(), high_level_loc_types.end(), child.first);
		if(loc_it != high_level_loc_types.end()) {
			world_locations.insert(child.second.get<string>("name"));
            string hddl_type;

            hddl_type = type_mapping[*loc_it];
            sorts[hddl_type].insert(child.second.get<string>("name"));
		}
	}

    /*
        Initializing objects that are not of location type but are declared in the configuration file type mappings
    */
    pt::ptree current_tree = worlddb_root;
                    
    stack<pt::ptree> ptree_stack;
                    
    pt::ptree::const_iterator end = current_tree.end();
    pt::ptree::const_iterator it = current_tree.begin();

    while(it != end) {
        bool changed_tree = false;

        pt::ptree::value_type child = *it;

        if(type_mapping.find(child.first) != type_mapping.end()) {
            string hddl_type = type_mapping[child.first];
            sorts[hddl_type].insert(child.second.get<string>("name"));
        }

        string child_data = child.second.data();
        boost::trim(child_data);
        if(!child.second.empty() && child_data == "") {
            current_tree.pop_front();
            if(current_tree.size() > 0) {
                ptree_stack.push(current_tree);
            }
            current_tree = child.second;

            end = current_tree.end();
            it = current_tree.begin();

            changed_tree = true;
        }

        if(!changed_tree && it != end) {
            it++;
            current_tree.pop_front();
        }

        if(it == end && ptree_stack.size() != 0) {
            current_tree = ptree_stack.top();
            ptree_stack.pop();

            it = current_tree.begin();
            end = current_tree.end();
        }
    }
}

void FileKnowledgeManager::initialize_attribute_mapping(SemanticMapping sm, pt::ptree worlddb_root, vector<ground_literal>& init, vector<pair<ground_literal,int>>& init_functions) {
    string attr_name = std::get<string>(sm.get_prop(name_key));
    string relation_type = std::get<string>(sm.get_prop(relatesto_key));
    if(relation_type != "robot") {
        //In addition to only mapping attributes we are only mapping them to predicates
        if(sm.get_mapped_type() == predicate_mapped_type) {
            predicate_definition pred = std::get<predicate_definition>(sm.get_prop(map_key));
                
            pt::ptree used_db;
            used_db = worlddb_root;

            pt::ptree current_tree = used_db;
            
            stack<pt::ptree> ptree_stack;
            
            pt::ptree::const_iterator end = current_tree.end();
            pt::ptree::const_iterator it = current_tree.begin();

            while(it != end) {
                bool changed_tree = false;

                pt::ptree::value_type child = *it;

                if(child.first == relation_type) {
                    //Only insert the predicate if the object exists (was initialized) in the sorts map
                    string hddl_type = type_mapping[relation_type];
                    if(sorts[hddl_type].find(child.second.get<string>("name")) != sorts[hddl_type].end()) {
                        bool val;
                        istringstream(boost::to_lower_copy(child.second.get<string>(attr_name))) >> std::boolalpha >> val;

                        ground_literal l;

                        l.predicate = pred.name;
                        l.positive = val;

                        /*
                            For now, semantic mappings only involve one argument, which is of the hddl_type. With this in mind,
                            we get the name attribute in the xml
                        */
                        for(string sort_type : pred.argument_sorts) {
                            if(sort_type == hddl_type) {
                                l.args.push_back(child.second.get<string>("name"));
                            }
                        }

                        init.push_back(l);
                    }
                } else {
                    string child_data = child.second.data();
                    boost::trim(child_data);
                    if(!child.second.empty() && child_data == "") {
                        current_tree.pop_front();
                        if(current_tree.size() > 0) {
                            ptree_stack.push(current_tree);
                        }
                        current_tree = child.second;

                        end = current_tree.end();
                        it = current_tree.begin();

                        changed_tree = true;
                    }
                }

                if(!changed_tree && it != end) {
                    it++;
                    current_tree.pop_front();
                }

                if(it == end && ptree_stack.size() > 0) {
                    current_tree = ptree_stack.top();
                    ptree_stack.pop();

                    it = current_tree.begin();
                    end = current_tree.end();
                }
            }
        } else if(sm.get_mapped_type() == function_mapped_type) {
            predicate_definition pred = std::get<predicate_definition>(sm.get_prop(map_key));
                
            pt::ptree used_db;
            used_db = worlddb_root;

            pt::ptree current_tree = used_db;
            
            stack<pt::ptree> ptree_stack;
            
            pt::ptree::const_iterator end = current_tree.end();
            pt::ptree::const_iterator it = current_tree.begin();

            while(it != end) {
                bool changed_tree = false;

                pt::ptree::value_type child = *it;

                if(child.first == relation_type) {
                    //Only insert the predicate if the object exists (was initialized) in the sorts map
                    string hddl_type = type_mapping[relation_type];
                    if(sorts[hddl_type].find(child.second.get<string>("name")) != sorts[hddl_type].end()) {
                        int val;
                        try {
                            istringstream(boost::to_lower_copy(child.second.get<string>(attr_name))) >> val;
                        } catch(...) {
                            string wrong_function_initialization_error = "Function initialization with attribute [" + attr_name + "] is not an integer value";

                            throw std::runtime_error(wrong_function_initialization_error);
                        }

                        ground_literal l;

                        l.predicate = pred.name;
                        l.positive = true;
                        
                        /*
                            For now, semantic mappings only involve one argument, which is of the hddl_type. With this in mind,
                            we get the name attribute in the xml
                        */
                        for(string sort_type : pred.argument_sorts) {
                            if(sort_type == hddl_type) {
                                l.args.push_back(child.second.get<string>("name"));
                            }
                        }

                        init_functions.push_back(make_pair(l,val));
                    }
                } else {
                    string child_data = child.second.data();
                    boost::trim(child_data);
                    if(!child.second.empty() && child_data == "") {
                        current_tree.pop_front();
                        if(current_tree.size() > 0) {
                            ptree_stack.push(current_tree);
                        }
                        current_tree = child.second;

                        end = current_tree.end();
                        it = current_tree.begin();

                        changed_tree = true;
                    }
                }

                if(!changed_tree && it != end) {
                    it++;
                    current_tree.pop_front();
                }

                if(it == end && ptree_stack.size() > 0) {
                    current_tree = ptree_stack.top();
                    ptree_stack.pop();

                    it = current_tree.begin();
                    end = current_tree.end();
                }
            }
        }
    }
}

void FileKnowledgeManager::initialize_ownership_mapping(SemanticMapping sm, pt::ptree worlddb_root, vector<ground_literal>& init) {
    string owner_type = std::get<string>(sm.get_prop(owner_key));
    string owned_type = std::get<string>(sm.get_prop(owned_key));

    if(owner_type != "robot" && owned_type != "robot") {
        //In addition to only mapping attributes we are only mapping them to predicates
        if(sm.get_mapped_type() == predicate_mapped_type) {
            predicate_definition pred = std::get<predicate_definition>(sm.get_prop(map_key));
                
            pt::ptree used_db;
            used_db = worlddb_root;

            pt::ptree current_tree = used_db;
            
            stack<pt::ptree> ptree_stack;
            
            pt::ptree::const_iterator end = current_tree.end();
            pt::ptree::const_iterator it = current_tree.begin();

            while(it != end) {
                bool changed_tree = false;

                pt::ptree::value_type child = *it;

                if(child.first == owner_type) {
                    set<string> owned_objects;

                    string owner_hddl_type = type_mapping[owner_type];
                    string owned_hddl_type = type_mapping[owned_type];
                    if(sorts[owner_hddl_type].find(child.second.get<string>("name")) != sorts[owner_hddl_type].end()) {
                        string owner_name = child.second.get<string>("name");

                        string relationship_type = std::get<string>(sm.get_prop(relationshiptype_key));
                        if(relationship_type == attribute_relationship_type) {
                            string attribute_name = std::get<string>(sm.get_prop(attributename_key));

                            pt::ptree attr_tree = child.second.get_child(attribute_name);
                            BOOST_FOREACH(pt::ptree::value_type& attr_child, attr_tree) {
                                if(attr_child.first == owned_type) {
                                    if(sorts[owned_hddl_type].find(attr_child.second.get<string>("name")) != sorts[owned_hddl_type].end()) {
                                        string owned_name = attr_child.second.get<string>("name");
                                        owned_objects.insert(owned_name);

                                        ground_literal l;

                                        l.predicate = pred.name;
                                        l.positive = true;

                                        l.args.push_back(owned_name);
                                        l.args.push_back(owner_name);

                                        init.push_back(l);
                                    }
                                }
                            }
                        }

                        for(string object : sorts[owned_hddl_type]) {
                            if(owned_objects.find(object) == owned_objects.end()) {
                                ground_literal l;

                                l.predicate = pred.name;
                                l.positive = false;

                                l.args.push_back(object);
                                l.args.push_back(owner_name);

                                init.push_back(l);
                            }
                        }
                    }            
                } else {
                    string child_data = child.second.data();
                    boost::trim(child_data);
                    if(!child.second.empty() && child_data == "") {
                        current_tree.pop_front();
                        if(current_tree.size() > 0) {
                            ptree_stack.push(current_tree);
                        }
                        current_tree = child.second;

                        end = current_tree.end();
                        it = current_tree.begin();

                        changed_tree = true;
                    }
                }

                if(!changed_tree && it != end) {
                    it++;
                    current_tree.pop_front();
                }

                if(it == end && ptree_stack.size() > 0) {
                    current_tree = ptree_stack.top();
                    ptree_stack.pop();

                    it = current_tree.begin();
                    end = current_tree.end();
                }
            }
        } else if(sm.get_mapped_type() == function_mapped_type) {
            // TODO
        }
    }
}

void FileKnowledgeManager::initialize_relationship_mapping(SemanticMapping sm, pt::ptree worlddb_root, std::vector<ground_literal>& init) {
    string main_entity_type = std::get<string>(sm.get_prop(mainentity_key));
    string related_entity_type = std::get<string>(sm.get_prop(relatedentity_key));

    if(main_entity_type != "robot" && related_entity_type != "robot") {
        //In addition to only mapping attributes we are only mapping them to predicates
        if(sm.get_mapped_type() == predicate_mapped_type) {
            predicate_definition pred = std::get<predicate_definition>(sm.get_prop(map_key));
                
            pt::ptree used_db;
            used_db = worlddb_root;

            pt::ptree current_tree = used_db;
            
            stack<pt::ptree> ptree_stack;
            
            pt::ptree::const_iterator end = current_tree.end();
            pt::ptree::const_iterator it = current_tree.begin();

            while(it != end) {
                bool changed_tree = false;

                pt::ptree::value_type child = *it;

                if(child.first == main_entity_type) {
                    set<string> owned_objects;

                    string main_entity_hddl_type = type_mapping[main_entity_type];
                    string related_entity_hddl_type = type_mapping[related_entity_type];
                    if(sorts[main_entity_hddl_type].find(child.second.get<string>("name")) != sorts[main_entity_hddl_type].end()) {
                        string owner_name = child.second.get<string>("name");

                        string relationship_type = std::get<string>(sm.get_prop(relationshiptype_key));
                        if(relationship_type == attribute_relationship_type) {
                            string attribute_name = std::get<string>(sm.get_prop(attributename_key));

                            vector<string> related_entities;

                            pt::ptree attr_tree = child.second.get_child(attribute_name);
                            if(attr_tree.empty() && !attr_tree.data().empty()) { //Key is value and not tree
                                stringstream ss(attr_tree.data());

                                string temp;
                                while(ss >> temp) {
                                    related_entities.push_back(temp);
                                }
                            } else if(!attr_tree.empty() && attr_tree.data().empty()) {
                                BOOST_FOREACH(pt::ptree::value_type related_entity, attr_tree) {
                                    related_entities.push_back(related_entity.second.data());
                                }    
                            }
                        }
                    }            
                } else {
                    string child_data = child.second.data();
                    boost::trim(child_data);
                    if(!child.second.empty() && child_data == "") {
                        current_tree.pop_front();
                        if(current_tree.size() > 0) {
                            ptree_stack.push(current_tree);
                        }
                        current_tree = child.second;

                        end = current_tree.end();
                        it = current_tree.begin();

                        changed_tree = true;
                    }
                }

                if(!changed_tree && it != end) {
                    it++;
                    current_tree.pop_front();
                }

                if(it == end && ptree_stack.size() > 0) {
                    current_tree = ptree_stack.top();
                    ptree_stack.pop();

                    it = current_tree.begin();
                    end = current_tree.end();
                }
            }
        } else if(sm.get_mapped_type() == function_mapped_type) {
            // TODO
        }
    }
}

/*
    Function: initialize_world_state
    Objective: Initialize the world state based on the world and robots knowledge. We also use the semantic mappings

    @ Input 1: A reference to the initial world state vector
    @ Input 2: A reference to the initial function values (in the future we may remove this)
    @ Input 3: The vector of semantic mappings given in the configuration file
    @ Input 4: The mapping between HDDL types and OCL types
    @ Input 5: The sorts map with the existing objects
    @ Output: Void. The reference to the initial world state vector is initialized
*/ 
void FileKnowledgeManager::initialize_world_state(vector<ground_literal>& init, vector<pair<ground_literal,int>>& init_functions, vector<SemanticMapping> semantic_mapping, map<string,set<string>> sorts) {
    pt::ptree worlddb_root;
    if(world_knowledge->get_root_key() == "") {
        worlddb_root = world_knowledge->get_knowledge();
    } else {
        worlddb_root = world_knowledge->get_knowledge().get_child(world_knowledge->get_root_key());
    }

    /*
        Initialize predicates from semantic mappings given at the configuration file

        -> Note that when mapping attributes we are only mapping boolean values
    */
    for(SemanticMapping sm : semantic_mapping) {
        //For now we are only mapping attributes
        if(sm.get_mapping_type() == attribute_mapping_type) {
            initialize_attribute_mapping(sm, worlddb_root, init, init_functions);
        } else if(sm.get_mapping_type() == ownership_mapping_type) {
            initialize_ownership_mapping(sm, worlddb_root, init);
        } else if(sm.get_mapping_type() == relationship_mapping_type) {
            initialize_relationship_mapping(sm, worlddb_root, init);
        }
    }
}

shared_ptr<FileKnowledgeBase> FileKnowledgeManager::get_world_knowledge() {
    return this->world_knowledge;
}