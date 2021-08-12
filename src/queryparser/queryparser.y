%define api.prefix {xy}
// Emitted in the header file, after the definition of YYSTYPE.
%code provides
{
  // Tell Flex the expected prototype of yylex.
  #define XY_DECL                             \
    int xylex ()

  // Declare the scanner.
  XY_DECL;
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

    #include "../utils/query.hpp"
    #include "../utils/condition.hpp"

    using namespace std;

    extern int xylex();
	extern int xyparse();
	extern const char *xyin;
	char* current_parser_file_name3;

    void xyerror(const char *s);

    Query* parsed_query;
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
    Query* query;
}


%token <sval> STRNAME
%token <sval> KEY_AND KEY_OR

%type <query> input
%type <query> expr
%type <query> expr-or-null
%type <query> expr-no-pt
%type <query> expr-and
%type <query> expr-or
%type <query> name-no-pt

%left KEY_AND
%left KEY_OR

%start input

%%
//Filling the map up
input: expr-or-null {parsed_query = $1;}

expr-or-null: expr {$$ = $1;}
            | {$$ = new Query();}

expr:  '(' expr-no-pt ')' {$$ = $2;}
    | expr-no-pt {$$ = $1;}
    | '(' name-no-pt ')' {$$ = $2;}
    | name-no-pt {$$ = $1;}

expr-no-pt: expr-and
          | expr-or

expr-and: expr KEY_AND expr {
    $$ = new Query();

    $$->query = make_pair($1,$3);
    $$->is_and = true;
}

expr-or: expr KEY_OR expr {
    $$ = new Query();

    $$->query = make_pair($1,$3);
    $$->is_and = false;
}

name-no-pt: STRNAME {
    $$ = new Query();
    std::vector<std::string> v;

    std::string aux = $1;

    smatch m;

    std::regex e2("[!a-zA-Z]{1}[\\w.]*"); //This will never fall in e3
    std::regex e3("[a-zA-Z]{1}[\\w.]*");
    std::regex num("[0-9]+");
    if((aux.find(ocl_equal) == std::string::npos) && (aux.find(ocl_different) == std::string::npos) && (aux.find(spaced_ocl_in) == std::string::npos) && 
        (aux.find(ocl_gt) == std::string::npos) && (aux.find(ocl_lt) == std::string::npos) && (aux.find(ocl_geq) == std::string::npos) && (aux.find(ocl_leq) == std::string::npos)) {
        if(std::regex_search(aux,m,e2)) {
            v.push_back(m[0]);
        } else {
            std::regex_search(aux,m,e3);
            v.push_back(m[0]);
        }
    } else {
        if(aux.find(ocl_equal) != std::string::npos || aux.find(ocl_different) != std::string::npos) {
            string left_side = aux.substr(0,aux.find('='));
            std::regex_search(left_side,m,e3);
            v.push_back(m[0]);
            
            if(aux.find(ocl_equal) != std::string::npos) {
                v.push_back(ocl_equal);
            } else {
                v.push_back(ocl_different);
            }

            string right_side = aux.substr(aux.find('='));
            std::regex_search(right_side,m,e3);
            v.push_back(m[0]);
        } else if(aux.find(spaced_ocl_in) != std::string::npos) {
            std::stringstream ss(aux);
            std::vector<std::string> split_query;

            std::string temp;
            while(ss >> temp) {
                split_query.push_back(temp);
            }

            std::regex_search(split_query.at(0),m,e3);
            v.push_back(m[0]);

            v.push_back(split_query.at(1));

            std::regex_search(split_query.at(2),m,e3);
            v.push_back(m[0]);
        } else {
            char op;

            if(aux.find(ocl_gt) != std::string::npos) {
                op = ocl_gt[0];
            } else if(aux.find(ocl_lt) != std::string::npos) {
                op = ocl_lt[0];
            } else if(aux.find(ocl_geq) != std::string::npos) {
                op = ocl_geq[0];
            } else if(aux.find(ocl_leq) != std::string::npos) {
                op = ocl_leq[0];
            }

            string var = aux.substr(0,aux.find(op));
            std::regex_search(var,m,e3);
            v.push_back(m[0]);

            string op_str(1, op);
            v.push_back(op_str);
            
            string number = aux.substr(aux.find(op)+1);
            std::regex_search(number,m,num);
            v.push_back(m[0]);
        }
    }

    $$->query = v;
}
%%

void set_input_query(const char* in);
void end_scan(void);

int parse_query(const char* in) {
  set_input_query(in);
  int rv = xyparse();
  end_scan();
  return rv;
}

void xyerror(const char *s) {
  //cout << "\x1b[31mParse error\x1b[0m in file " << current_parser_file_name3 << " in line \x1b[1m" << yylloc.first_line << "\x1b[0m" << endl;
  if (strlen(s) >= 14 && (strncmp("syntax error, ",s,14) == 0)){
    s += 14;
  }
  cout << "\x1b[31mParse error\x1b[0m in query: " << s << endl; 
  //cout << s << endl;
  // might as well halt now:
  exit(-1);
}
