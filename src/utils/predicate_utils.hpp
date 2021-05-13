#ifndef __PREDICATE_UTILS
#define __PREDICATE_UTILS

#include <variant>

#include "parsetree.hpp"
#include "domain.hpp"

bool is_same_predicate(variant<literal,ground_literal> pred1, variant<literal,ground_literal> pred2);

#endif