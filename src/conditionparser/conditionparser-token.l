%option prefix="xx"

%{
	#include <iostream>
	#include <vector>
	#include "../utils/condition.hpp"
	#include "conditionparser.hpp"

	using namespace std;
%}

%option noyywrap

%%
\(				{return '(';}
\)				{return ')';}
\&\&				{return KEY_AND_1;}
\|\|				{return KEY_OR_1;}

[ \t\n\r]         ; // whitespace

[!a-zA-Z]{1}[a-zA-Z0-9_. ="<>]* {xxlval.sval = strdup(yytext); return STRNAME1;}
%%

void set_input_condition(const char* in) {
  yy_scan_string(in);
}

void end_condition_scan(void) {
    yy_delete_buffer(YY_CURRENT_BUFFER);
}