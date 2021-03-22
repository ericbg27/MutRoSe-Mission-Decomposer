%define api.prefix {zz}
// Emitted in the header file, after the definition of YYSTYPE.
%code provides
{
  // Tell Flex the expected prototype of yylex.
  #define YY_DECL                             \
    int zzlex ()

  // Declare the scanner.
  YY_DECL;
}

%{
    #include <set>
    #include <map>
    #include <vector>
    #include <iostream>

    #include "annotmanager.hpp"

    using namespace std;

    extern int zzlex();
	extern int zzparse();
	extern const char *zzin;
	char* current_parser_file_name2;

    void zzerror(const char *s);
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
    general_annot* annot;
}

%token KEY_END_ID
%token <sval> KEY_SEQ KEY_PAR KEY_FALLBACK KEY_OPT KEY_ALT 
%token <sval> STRNAME 

//%type <annot> annot-list
%type <annot> annot
%type <annot> input
%type <annot> expr
%type <annot> expr-seq
%type <annot> expr-par
%type <annot> expr-fallback
%type <annot> expr-opt
%type <annot> expr-alt
%type <annot> expr-list
%type <annot> simple-expr
%type <annot> simple-seq
%type <annot> simple-par
%type <annot> simple-fallback
%type <annot> simple-opt
%type <annot> simple-alt
%type <vstring> STRNAME-list

%start input

%%
//Filling the map up
input: STRNAME KEY_END_ID STRNAME-list annot {$$ = $4; $$->related_goal = $1; goals_and_rannots[$1] = $$;}

annot: '[' expr ']' {$$ = $2;} | {$$ = new general_annot();}

//Here we have to add nodes to the graph and generate the graph itself
//annot-list: expr {
//    $$ = new RAnnotGraph();
//    /*
//        Use the general_annot structure to generate a RAnnotGraph
//    */
//}

expr-list: expr-list expr {
            $$ = $2;
} | {$$ = new general_annot();}

expr: expr-seq {$$ = $1;}
    | expr-par {$$ = $1;}
    | expr-fallback {$$ = $1;}
    | expr-opt {$$ = $1;}
    | expr-alt {$$ = $1;}
    | simple-expr {$$ = $1;}

expr-seq: '(' expr-list ')' KEY_SEQ '(' expr-list ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;

    $$->content = ";";

    $$->children.push_back($2);
    $$->children.push_back($6);
}
expr-par: '(' expr-list ')' KEY_PAR '(' expr-list ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;

    $$->content = "#";
    
    $$->children.push_back($2);
    $$->children.push_back($6);
}
expr-fallback: KEY_FALLBACK '(' '(' expr-list ')' ',' '(' expr-list ')' ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;

    $$->content = "FALLBACK";
    
    $$->children.push_back($4);
    $$->children.push_back($8);
}
expr-opt: KEY_OPT '(' '(' expr-list ')' ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;

    $$->content = "OPT";
    
    $$->children.push_back($4);
}
expr-alt: '(' expr-list ')' KEY_ALT '(' expr-list ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;

    $$->content = "|";
    
    $$->children.push_back($2);
    $$->children.push_back($6);
}

simple-expr: simple-seq
           | simple-par
           | simple-fallback
           | simple-opt
           | simple-alt

simple-seq: STRNAME KEY_SEQ STRNAME {
    $$ = new general_annot();
    $$->type = OPERATOR;
    $$->content = ";";

    general_annot* op1 = new general_annot();
    general_annot* op2 = new general_annot();

    if($1[0] == 'G') {
        op1->type = GOAL;
    } else {
        op1->type = TASK;
    }

    if($3[0] == 'G') {
        op2->type = GOAL;
    } else {
        op2->type = TASK;
    }

    op1->content = $1;
    op2->content = $3;

    $$->children.push_back(op1);
    $$->children.push_back(op2); 
}
simple-par: STRNAME KEY_PAR STRNAME {
    $$ = new general_annot();
    $$->type = OPERATOR;
    $$->content = "#";

    general_annot* op1 = new general_annot();
    general_annot* op2 = new general_annot();

    if($1[0] == 'G') {
        op1->type = GOAL;
    } else {
        op1->type = TASK;
    }

    if($3[0] == 'G') {
        op2->type = GOAL;
    } else {
        op2->type = TASK;
    }

    op1->content = $1;
    op2->content = $3;

    $$->children.push_back(op1);
    $$->children.push_back(op2);       
}
simple-fallback: KEY_FALLBACK '(' STRNAME ',' STRNAME ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;
    $$->content = "FALLBACK";

    general_annot* op1 = new general_annot();
    general_annot* op2 = new general_annot();

    if($3[0] == 'G') {
        op1->type = GOAL;
    } else {
        op1->type = TASK;
    }

    if($5[0] == 'G') {
        op2->type = GOAL;
    } else {
        op2->type = TASK;
    }

    op1->content = $3;
    op2->content = $5;

    $$->children.push_back(op1);
    $$->children.push_back(op2);       
}
simple-opt: KEY_OPT '(' STRNAME ')' {
    $$ = new general_annot();
    $$->type = OPERATOR;
    $$->content = "OPT";

    general_annot* op1 = new general_annot();

    if($3[0] == 'G') {
        op1->type = GOAL;
    } else {
        op1->type = TASK;
    }

    op1->content = $3;

    $$->children.push_back(op1);    
}
simple-alt: STRNAME KEY_ALT STRNAME {
    $$ = new general_annot();
    $$->type = OPERATOR;
    $$->content = "|";

    general_annot* op1 = new general_annot();
    general_annot* op2 = new general_annot();

    if($1[0] == 'G') {
        op1->type = GOAL;
    } else {
        op1->type = TASK;
    }

    if($3[0] == 'G') {
        op2->type = GOAL;
    } else {
        op2->type = TASK;
    }

    op1->content = $1;
    op2->content = $3;

    $$->children.push_back(op1);
    $$->children.push_back(op2);       
}

STRNAME-list: STRNAME-list STRNAME {string s($2); free($2); $$->push_back(s);}
			|  {$$ = new vector<string>();}
%%

void set_input_string(const char* in);
void end_lexical_scan(void);

int parse_string(const char* in) {
  set_input_string(in);
  int rv = zzparse();
  end_lexical_scan();
  return rv;
}

void zzerror(const char *s) {
  cout << "\x1b[31mParse error\x1b[0m in file " << current_parser_file_name2 << " in line \x1b[1m" << yylloc.first_line << "\x1b[0m" << endl;
  if (strlen(s) >= 14 && (strncmp("syntax error, ",s,14) == 0)){
    s += 14;
  }
  cout << s << endl;
  // might as well halt now:
  exit(-1);
}