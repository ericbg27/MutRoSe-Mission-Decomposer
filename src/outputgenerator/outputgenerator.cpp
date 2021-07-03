#include "outputgenerator.hpp"

#include <iostream>

using namespace std;

void OutputGenerator::set_verbose(bool verb) {
    verbose = verb;
}

void OutputGenerator::set_mission_decomposition(ATGraph md) {
    mission_decomposition = md;
}

void OutputGenerator::set_gm(GMGraph g) {
    gm = g;
}

void OutputGenerator::set_world_state(vector<ground_literal> ws) {
    world_state = ws;
}

void OutputGenerator::set_world_state_functions(vector<pair<ground_literal,variant<int,float>>> wsf) {
    world_state_functions = wsf;
}

void FileOutputGenerator::set_file_output_generator_type(file_output_generator_type fogt) {
    fog_type = fogt;
}

file_output_generator_type FileOutputGenerator::get_file_output_generator_type() {
    return fog_type;
}

void FileOutputGenerator::set_output(pair<string,string> out) {
    output = out;
}

/*
    Function: find_predicate_mapping
    Objective: Find a semantic mapping involving a given predicate

    @ Input 1: The predicate to be evaluated
    @ Input 2: The vector of semantic mappings
    @ Input 3: The sorts map, where objects are declared
    @ Input 4: The var mappings between HDDL and OCL goal model variables
    @ Input 5: The sort definitions
    @ Output: A pair containing the semantic mapping and a boolean flag indicating if a mapping was found
*/
pair<SemanticMapping, bool> find_predicate_mapping(variant<ground_literal,literal> predicate, vector<SemanticMapping> semantic_mappings, map<string,set<string>> sorts,
                                                    map<string,string> vars, vector<sort_definition> sort_definitions) {
    SemanticMapping prec_mapping;
    bool found_mapping = false;

    if(holds_alternative<ground_literal>(predicate)) {
        /*
            If the predicate is grounded we can search in the declared objects for the necessary attributes
        */
        ground_literal p = get<ground_literal>(predicate);

        for(SemanticMapping sm : semantic_mappings) {
            if(sm.get_mapped_type() == "predicate" || sm.get_mapped_type() == "function") {
                predicate_definition map = std::get<predicate_definition>(sm.get_prop("map"));

                if(map.name == p.predicate) {
                    bool found_args = true;
                    int arg_index = 0;
                    for(string sort : map.argument_sorts) {
                        bool found_arg = false;
                        for(string object : sorts[sort]) {
                            if(object == p.args.at(arg_index)) {
                                found_arg = true;
                                break;
                            }
                        }

                        if(!found_arg) {
                            found_args = false;
                            break;
                        }

                        arg_index++;
                    }

                    if(found_args) { 
                        prec_mapping = sm;
                        found_mapping = true;
                        break;
                    }
                }
            }
        }
    } else {
        literal p = get<literal>(predicate);

        for(SemanticMapping sm : semantic_mappings) {
            if(sm.get_mapped_type() == "predicate" || sm.get_mapped_type() == "function") {
                predicate_definition map = get<predicate_definition>(sm.get_prop("map"));

                if(map.name == p.predicate) {
                    bool found_args = true;
                    int arg_index = 0;
                    for(string sort : map.argument_sorts) {
                        bool found_arg = false;
                        
                        /*
                            Here we need to check if the predicate literal is equal to the predicate in the mapping

                            -> In order to be equal, we need to have the same predicate and the same argument types
                        */
                        if(vars[p.arguments.at(arg_index)] == sort) {
                            found_arg = true;
                        } else {
                            bool is_parent_type = false;
                            for(sort_definition s : sort_definitions) {
                                for(string d_sort : s.declared_sorts) {
                                    if(d_sort == vars[p.arguments.at(arg_index)]) {
                                        if(s.has_parent_sort) {
                                            if(s.parent_sort == sort) {
                                                is_parent_type = true;
                                                break;
                                            }
                                        }
                                    }
                                }

                                if(is_parent_type) {
                                    found_arg = true;
                                    break;
                                }
                            }
                        }

                        if(!found_arg) {
                            found_args = false;
                            break;
                        }

                        arg_index++;
                    }

                    if(found_args) { 
                        prec_mapping = sm;
                        found_mapping = true;
                        break;
                    }
                }
            }
        }
    }

    return make_pair(prec_mapping, found_mapping);
}