/* A Bison parser, made by GNU Bison 3.7.5.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2021 Free Software Foundation,
   Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

#ifndef YY_XY_SRC_QUERYPARSER_QUERYPARSER_HPP_INCLUDED
# define YY_XY_SRC_QUERYPARSER_QUERYPARSER_HPP_INCLUDED
/* Debug traces.  */
#ifndef XYDEBUG
# if defined YYDEBUG
#if YYDEBUG
#   define XYDEBUG 1
#  else
#   define XYDEBUG 0
#  endif
# else /* ! defined YYDEBUG */
#  define XYDEBUG 0
# endif /* ! defined YYDEBUG */
#endif  /* ! defined XYDEBUG */
#if XYDEBUG
extern int xydebug;
#endif

/* Token kinds.  */
#ifndef XYTOKENTYPE
# define XYTOKENTYPE
  enum xytokentype
  {
    XYEMPTY = -2,
    XYEOF = 0,                     /* "end of file"  */
    XYerror = 256,                 /* error  */
    XYUNDEF = 257,                 /* "invalid token"  */
    STRNAME = 258,                 /* STRNAME  */
    KEY_AND = 259,                 /* KEY_AND  */
    KEY_OR = 260                   /* KEY_OR  */
  };
  typedef enum xytokentype xytoken_kind_t;
#endif

/* Value type.  */
#if ! defined XYSTYPE && ! defined XYSTYPE_IS_DECLARED
union XYSTYPE
{
#line 44 "src/queryparser/queryparser.y"

	bool bval;
	int ival;
	float fval;
	char *sval;
    std::vector<std::string>* vstring;
    Query* query;

#line 86 "src/queryparser/queryparser.hpp"

};
typedef union XYSTYPE XYSTYPE;
# define XYSTYPE_IS_TRIVIAL 1
# define XYSTYPE_IS_DECLARED 1
#endif

/* Location type.  */
#if ! defined XYLTYPE && ! defined XYLTYPE_IS_DECLARED
typedef struct XYLTYPE XYLTYPE;
struct XYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define XYLTYPE_IS_DECLARED 1
# define XYLTYPE_IS_TRIVIAL 1
#endif


extern XYSTYPE xylval;
extern XYLTYPE xylloc;
int xyparse (void);
/* "%code provides" blocks.  */
#line 4 "src/queryparser/queryparser.y"

  // Tell Flex the expected prototype of yylex.
  #define XY_DECL                             \
    int xylex ()

  // Declare the scanner.
  XY_DECL;

#line 122 "src/queryparser/queryparser.hpp"

#endif /* !YY_XY_SRC_QUERYPARSER_QUERYPARSER_HPP_INCLUDED  */
