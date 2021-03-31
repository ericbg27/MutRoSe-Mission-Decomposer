/* A Bison parser, made by GNU Bison 3.7.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2020 Free Software Foundation,
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

#ifndef YY_ZZ_SRC_RANNOT_HPP_INCLUDED
# define YY_ZZ_SRC_RANNOT_HPP_INCLUDED
/* Debug traces.  */
#ifndef ZZDEBUG
# if defined YYDEBUG
#if YYDEBUG
#   define ZZDEBUG 1
#  else
#   define ZZDEBUG 0
#  endif
# else /* ! defined YYDEBUG */
#  define ZZDEBUG 0
# endif /* ! defined YYDEBUG */
#endif  /* ! defined ZZDEBUG */
#if ZZDEBUG
extern int zzdebug;
#endif

/* Token kinds.  */
#ifndef ZZTOKENTYPE
# define ZZTOKENTYPE
  enum zztokentype
  {
    ZZEMPTY = -2,
    ZZEOF = 0,                     /* "end of file"  */
    ZZerror = 256,                 /* error  */
    ZZUNDEF = 257,                 /* "invalid token"  */
    KEY_END_ID = 258,              /* KEY_END_ID  */
    KEY_SEQ = 259,                 /* KEY_SEQ  */
    KEY_PAR = 260,                 /* KEY_PAR  */
    KEY_FALLBACK = 261,            /* KEY_FALLBACK  */
    KEY_OPT = 262,                 /* KEY_OPT  */
    KEY_ALT = 263,                 /* KEY_ALT  */
    STRNAME = 264                  /* STRNAME  */
  };
  typedef enum zztokentype zztoken_kind_t;
#endif

/* Value type.  */
#if ! defined ZZSTYPE && ! defined ZZSTYPE_IS_DECLARED
union ZZSTYPE
{
#line 38 "src/rannot.y"

	bool bval;
	int ival;
	float fval;
	char *sval;
    std::vector<std::string>* vstring;
    general_annot* annot;

#line 90 "src/rannot.hpp"

};
typedef union ZZSTYPE ZZSTYPE;
# define ZZSTYPE_IS_TRIVIAL 1
# define ZZSTYPE_IS_DECLARED 1
#endif

/* Location type.  */
#if ! defined ZZLTYPE && ! defined ZZLTYPE_IS_DECLARED
typedef struct ZZLTYPE ZZLTYPE;
struct ZZLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define ZZLTYPE_IS_DECLARED 1
# define ZZLTYPE_IS_TRIVIAL 1
#endif


extern ZZSTYPE zzlval;
extern ZZLTYPE zzlloc;
int zzparse (void);
/* "%code provides" blocks.  */
#line 4 "src/rannot.y"

  // Tell Flex the expected prototype of yylex.
  #define YY_DECL                             \
    int zzlex ()

  // Declare the scanner.
  YY_DECL;

#line 126 "src/rannot.hpp"

#endif /* !YY_ZZ_SRC_RANNOT_HPP_INCLUDED  */
