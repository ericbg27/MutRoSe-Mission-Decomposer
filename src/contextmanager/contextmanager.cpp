#include "contextmanager.hpp"

#include <iostream>

using namespace std;

string Context::get_context_type() {
	return type;
}

void Context::set_context_type(string tp) {
	type = tp;
}

/*
    Function: check_context
    Objective: Check if a given context is active given a world_state

    @ Input 1: The context beign evaluated
	@ Input 2: The world state used to evaluate the context
	@ Input 3: The semantic mapping vector
	@ Input 4: The map of OCL goal model instantiated variables
    @ Output: A boolean value indicating if the context is true or not
*/
bool Context::check_context(vector<ground_literal> world_state, vector<SemanticMapping> semantic_mapping, map<string, variant<pair<string,string>,pair<vector<string>,string>>> var_maps) {	
	bool is_active = false;

	string var_attr_regex = "([!]?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";
	string var_attr_regex2 = "(((\\bnot\\b)[ ]+){1}[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";

	set<string> accepted_regex_patterns = {var_attr_regex, var_attr_regex2};

	ConditionEvaluation* evaluation = Condition::evaluate_condition(var_maps, semantic_mapping, accepted_regex_patterns);
	ConditionExpression* eval_result = evaluation->get_evaluation_predicates();

	vector<pair<ground_literal,variant<int,float>>> world_state_functions; // TODO: Empty for now

	is_active = eval_result->evaluate_expression(world_state, world_state_functions);

	return is_active;
}

ConditionExpression* Context::get_inactive_predicates(map<string, variant<pair<string,string>,pair<vector<string>,string>>> var_maps, pred_vector world_state, vector<SemanticMapping> semantic_mapping) {
	string var_attr_regex = "([!]?[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";
	string var_attr_regex2 = "(((\\bnot\\b)[ ]+){1}[A-Za-z]+[A-Za-z0-9_]*[.][A-Za-z]+[A-Za-z_]*){1}";

	set<string> accepted_regex_patterns = {var_attr_regex, var_attr_regex2};

	ConditionEvaluation* evaluation = Condition::evaluate_condition(var_maps, semantic_mapping, accepted_regex_patterns);
	ConditionExpression* eval_result = evaluation->get_evaluation_predicates();

	vector<pair<ground_literal,variant<int,float>>> world_state_functions; // Empty for now

	return eval_result->check_non_active_predicates(world_state, world_state_functions, semantic_mapping);
}
/*
    Function: get_pred_from_context
    Objective: Return a predicate definition from a context based on the existing semantic mappings

    @ Input 1: The context condition being evaluated
	@ Input 2: The semantic mappings vector
    @ Output: A pair containing:
		- A boolean flag indicating if the generated predicate is positive
		- A pair of the actual variable name and the generated predicate

	NOTE: For now we will accept only the attribute format

		  -> This means that we will have as the context something in the form [variable].[attribute]
*/
pair<bool,pair<string,predicate_definition>> get_pred_from_context(Context context, vector<SemanticMapping> semantic_mapping) {
	predicate_definition pred;
	string var;
	bool positive = true;

	if(context.get_context_type() == condition_context_type) {
		string condition = std::get<string>(context.get_condition());
		string attr;

		if(condition.find("!") != std::string::npos || condition.find("not") != std::string::npos) {
			positive = false;
		}

		size_t cond_sep = condition.find('.');

		var = condition.substr(0,cond_sep);
		if(!positive) {
			if(var.find(" ") != std::string::npos) { // expression has a not
				size_t not_sep = var.find(" ");
				var = var.substr(not_sep+1,var.size()-not_sep-1);
			} else {
				var = var.substr(1,var.size());
			}
		}
		
		attr = condition.substr(cond_sep+1,condition.size()-cond_sep-1);

		/*
			Check attribute value and return it

			-> We need a way to check the variable type here
		*/
		bool found_pred = false;
		for(SemanticMapping map : semantic_mapping) {
			if(map.get_mapping_type() == "attribute") {
				if(get<string>(map.get_prop("name")) == attr) {
					pred = get<predicate_definition>(map.get_prop("map"));
					found_pred = true;
					break;
				}
			}
		}

		if(!found_pred || var == " " || attr == " ") {
			std::string predicate_not_found_err = "Could not build predicate from context: " + condition;
			
			throw std::runtime_error(predicate_not_found_err);
		}
	}

	return make_pair(positive,make_pair(var,pred));
}