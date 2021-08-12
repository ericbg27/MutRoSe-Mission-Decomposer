%define api.prefix {xx}
// Emitted in the header file, after the definition of YYSTYPE.
%code provides
{
  // Tell Flex the expected prototype of yylex.
  #define xx_DECL                             \
    int xxlex ()

  // Declare the scanner.
  xx_DECL;
}

%{
    #include <set>
    #include <map>
    #include <vector>
    #include <iostream>
    #include <string>
    #include <sstream>
    #include <variant>
    #include <regex>

    #include "../utils/condition.hpp"

    using namespace std;

    extern int xxlex();
    extern int xxparse();
    extern const char *xxin;
    char* current_parser_file_name4;

    void xxerror(const char *s);

    Condition* parsed_condition;
%}

%locations
%define parse.error verbose
%define parse.lac full

%locations

%union {
	bool bval;
	int ival;
	float fval;
	char *sval;
    std::vector<std::string>* vstring;
    Condition* condition;
}


%token <sval> STRNAME1
%token <sval> KEY_AND_1 KEY_OR_1

%type <condition> input
%type <condition> expr
%type <condition> expr-or-null
%type <condition> expr-no-pt
%type <condition> expr-and
%type <condition> expr-or
%type <condition> name-no-pt

%left KEY_AND_1
%left KEY_OR_1

%start input

%%
//Filling the map up
input: expr-or-null {parsed_condition = $1;}

expr-or-null: expr {$$ = $1;}
            | {$$ = new Condition();}

expr:  '(' expr-no-pt ')' {$$ = $2;}
    | expr-no-pt {$$ = $1;}
    | '(' name-no-pt ')' {$$ = $2;}
    | name-no-pt {$$ = $1;}

expr-no-pt: expr-and
          | expr-or

expr-and: expr KEY_AND_1 expr {
    $$ = new Condition();

    $$->set_condition(make_pair($1,$3));
    $$->set_is_and(true);
}

expr-or: expr KEY_OR_1 expr {
    $$ = new Condition();

    $$->set_condition(make_pair($1,$3));
    $$->set_is_and(false);
}

name-no-pt: STRNAME1 {
    $$ = new Condition();
    string aux($1);
    aux.erase(std::find_if(aux.rbegin(), aux.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), aux.end());
    $$->set_condition(aux);
}
%%

void set_input_condition(const char* in);
void end_condition_scan(void);

int parse_condition(const char* in) {
  set_input_condition(in);
  int rv = xxparse();
  end_condition_scan();
  return rv;
}

void xxerror(const char *s) {
  //cout << "\x1b[31mParse error\x1b[0m in file " << current_parser_file_name4 << " in line \x1b[1m" << yylloc.first_line << "\x1b[0m" << endl;
  if (strlen(s) >= 14 && (strncmp("syntax error, ",s,14) == 0)){
    s += 14;
  }
  cout << "\x1b[31mParse error\x1b[0m in condition: " << s << endl; 
  //cout << s << endl;
  // might as well halt now:
  exit(-1);
}
