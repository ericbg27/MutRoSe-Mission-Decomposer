#include "config_utils.hpp"

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

bool SemanticMapping::has_prop(string prop) {
    if(this->mapping_props.find(prop) != this->mapping_props.end()) {
        return true;
    }

    return false;
}

variant<string,predicate_definition> SemanticMapping::get_prop(string prop) {
    if(this->has_prop(prop)) {
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