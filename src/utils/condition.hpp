#ifndef __CONDITION
#define __CONDITION

#include <string>
#include <map>
#include <vector>

#include "parsetree.hpp"
#include "../config/config.hpp"

class Condition {
    public:
        std::string get_condition();

        void set_condition(std::string cond);

        std::variant<std::pair<std::pair<predicate_definition,std::vector<std::string>>,bool>,std::pair<std::pair<predicate_definition,std::vector<std::string>>,std::pair<std::variant<int,float>,std::variant<bool,std::string>>>,bool> evaluate_condition(std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>> var_value_and_type, std::vector<SemanticMapping> semantic_mapping);

        std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>> get_var_value_and_type(std::map<std::string, std::variant<std::pair<std::string,std::string>,std::pair<std::vector<std::string>,std::string>>> gm_var_map, std::string variable);
    protected:
        std::string condition;
};

#endif