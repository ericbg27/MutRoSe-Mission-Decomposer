%option prefix="xy"

%{
	#include <iostream>
	#include <vector>
	#include "../utils/query.hpp"
	#include "queryparser.hpp"

	using namespace std;
%}

%option noyywrap

%%
\(				{return '(';}
\)				{return ')';}
\&\&				{return KEY_AND;}
\|\|				{return KEY_OR;}

[ \t\n\r]         ; // whitespace

[!a-zA-Z]{1}[a-zA-Z0-9_. ="<>]*	{xylval.sval = strdup(yytext); return STRNAME;}
%%

void set_input_query(const char* in) {
  yy_scan_string(in);
}

void end_scan(void) {
    yy_delete_buffer(YY_CURRENT_BUFFER);
}