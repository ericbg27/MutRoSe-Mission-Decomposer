CC=g++

CWARN=-Wno-unused-parameter
CERROR=

COMPILEFLAGS=-O3 -pipe -Wall -Wextra -pedantic -std=c++17 $(CWARN) $(CERROR)
LINKERFLAG=-O3 -lm -flto -static -static-libgcc
#COMPILEFLAGS=-O0 -ggdb -pipe -Wall -Wextra -pedantic -std=c++17 $(CWARN) $(CERROR)
#LINKERFLAG=-O0 -ggdb

.PHONY = parser clean

pandaPIparser: src/hddl-token.o src/hddl.o src/main.o src/sortexpansion.o src/parsetree.o src/util.o src/domain.o src/output.o src/parametersplitting.o src/cwa.o src/typeof.o src/shopWriter.o src/hpdlWriter.o src/hddlWriter.o src/orderingDecomposition.o src/plan.o src/verify.o src/properties.o src/contextmanager.o src/gm.o src/tdg.o src/annotmanager/annotmanager.o src/rannot-token.o src/rannot.o src/atmanager/at_manager.o src/config.o src/knowledgemanager/knowledgemanager.o src/knowledgebase/knowledgebase.o src/missiondecomposer/missiondecomposer.o src/constraintmanager.o src/outputgenerator/outputgenerator.o src/outputgenerator/xmloutputgenerator.o src/outputgenerator/fileoutputgeneratorfactory.o src/configchecker.o
	${CC} ${LINKERFLAG} $^ -o pandaPIparser 

%.o: %.cpp %.hpp src/hddl.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

%.o: %.cpp src/hddl.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

src/hddl-token.cpp: src/hddl.cpp src/hddl-token.l
	flex --yylineno -o src/hddl-token.cpp src/hddl-token.l

src/hddl.cpp: src/hddl.y
	bison -v -d -o src/hddl.cpp src/hddl.y

src/hddl.hpp: src/hddl.cpp

%.o: %.cpp %.hpp src/rannot.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

%.o: %.cpp src/rannot.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

src/rannot-token.cpp: src/rannot.cpp src/rannot-token.l
	flex -o src/rannot-token.cpp src/rannot-token.l

src/rannot.cpp: src/rannot.y
	bison -v -d -o src/rannot.cpp src/rannot.y
	
src/rannot.hpp: src/rannot.cpp

clear:
	rm src/hddl-token.cpp
	rm src/hddl.cpp
	rm src/hddl.hpp
	rm src/*o

clear2:
	rm src/rannot-token.cpp
	rm src/rannot.cpp
	rm src/rannot.hpp
	rm src/*o
