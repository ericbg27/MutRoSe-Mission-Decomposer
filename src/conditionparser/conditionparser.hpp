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

#ifndef YY_XX_SRC_CONDITIONPARSER_CONDITIONPARSER_HPP_INCLUDED
# define YY_XX_SRC_CONDITIONPARSER_CONDITIONPARSER_HPP_INCLUDED
/* Debug traces.  */
#ifndef XXDEBUG
# if defined YYDEBUG
#if YYDEBUG
#   define XXDEBUG 1
#  else
#   define XXDEBUG 0
#  endif
# else /* ! defined YYDEBUG */
#  define XXDEBUG 0
# endif /* ! defined YYDEBUG */
#endif  /* ! defined XXDEBUG */
#if XXDEBUG
extern int xxdebug;
#endif

/* Token kinds.  */
#ifndef XXTOKENTYPE
# define XXTOKENTYPE
  enum xxtokentype
  {
    XXEMPTY = -2,
    XXEOF = 0,                     /* "end of file"  */
    XXerror = 256,                 /* error  */
    XXUNDEF = 257,                 /* "invalid token"  */
    STRNAME1 = 258,                /* STRNAME1  */
    KEY_AND_1 = 259,               /* KEY_AND_1  */
    KEY_OR_1 = 260                 /* KEY_OR_1  */
  };
  typedef enum xxtokentype xxtoken_kind_t;
#endif

/* Value type.  */
#if ! defined XXSTYPE && ! defined XXSTYPE_IS_DECLARED
union XXSTYPE
{
#line 43 "src/conditionparser/conditionparser.y"

	bool bval;
	int ival;
	float fval;
	char *sval;
    std::vector<std::string>* vstring;
    Condition* condition;

#line 86 "src/conditionparser/conditionparser.hpp"

};
typedef union XXSTYPE XXSTYPE;
# define XXSTYPE_IS_TRIVIAL 1
# define XXSTYPE_IS_DECLARED 1
#endif

/* Location type.  */
#if ! defined XXLTYPE && ! defined XXLTYPE_IS_DECLARED
typedef struct XXLTYPE XXLTYPE;
struct XXLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define XXLTYPE_IS_DECLARED 1
# define XXLTYPE_IS_TRIVIAL 1
#endif


extern XXSTYPE xxlval;
extern XXLTYPE xxlloc;
int xxparse (void);
/* "%code provides" blocks.  */
#line 4 "src/conditionparser/conditionparser.y"

  // Tell Flex the expected prototype of yylex.
  #define xx_DECL                             \
    int xxlex ()

  // Declare the scanner.
  xx_DECL;

#line 122 "src/conditionparser/conditionparser.hpp"

#endif /* !YY_XX_SRC_CONDITIONPARSER_CONDITIONPARSER_HPP_INCLUDED  */
