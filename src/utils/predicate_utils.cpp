#include "predicate_utils.hpp"

using namespace std;

bool is_same_predicate(variant<literal,ground_literal> pred1, variant<literal,ground_literal> pred2) {
    if(holds_alternative<literal>(pred1)) {
        if(holds_alternative<literal>(pred2)) {
            literal p1 = std::get<literal>(pred1);
            literal p2 = std::get<literal>(pred2);
            
            if(p1.predicate == p2.predicate) {
                bool equal_args = true;
                
                int arg_index = 0;
                for(string arg : p1.arguments) {
                    if(arg != p2.arguments.at(arg_index)) {
                        equal_args = false;
                        break;
                    }

                    arg_index++;
                }

                return equal_args;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        if(holds_alternative<ground_literal>(pred2)) {
            ground_literal p1 = std::get<ground_literal>(pred1);
            ground_literal p2 = std::get<ground_literal>(pred2);

            if(p1.predicate == p2.predicate) {
                bool equal_args = true;
                
                int arg_index = 0;
                for(string arg : p1.args) {
                    if(arg != p2.args.at(arg_index)) {
                        equal_args = false;
                        break;
                    }

                    arg_index++;
                }

                return equal_args;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    return false;
}