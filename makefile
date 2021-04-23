CC=g++

CWARN=-Wno-unused-parameter
CERROR=

COMPILEFLAGS=-O3 -pipe -Wall -Wextra -pedantic -std=c++17 $(CWARN) $(CERROR)
LINKERFLAG=-O3 -lm -flto -static -static-libgcc
#COMPILEFLAGS=-O0 -ggdb -pipe -Wall -Wextra -pedantic -std=c++17 $(CWARN) $(CERROR)
#LINKERFLAG=-O0 -ggdb

.PHONY = parser clean

pandaPIparser: src/hddl/hddl-token.o src/hddl/hddl.o src/main.o src/utils/sortexpansion.o src/utils/parsetree.o src/utils/util.o src/utils/domain.o src/utils/output.o src/utils/parametersplitting.o src/utils/cwa.o src/utils/typeof.o src/utils/orderingDecomposition.o src/utils/plan.o src/utils/verify.o src/utils/properties.o src/utils/condition.o src/contextmanager/contextmanager.o src/gm/gm.o src/tdg/tdg.o src/annotmanager/annotmanager.o src/rannot/rannot-token.o src/rannot/rannot.o src/atmanager/at_manager.o src/config/config.o src/knowledgemanager/knowledgemanager.o src/knowledgebase/knowledgebase.o src/missiondecomposer/missiondecomposer.o src/constraintmanager/constraintmanager.o src/outputgenerator/outputgenerator.o src/outputgenerator/xmloutputgenerator.o src/outputgenerator/fileoutputgeneratorfactory.o src/configchecker/configchecker.o
	${CC} ${LINKERFLAG} $^ -o pandaPIparser 

%.o: %.cpp %.hpp src/hddl/hddl.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

%.o: %.cpp src/hddl/hddl.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

src/hddl/hddl-token.cpp: src/hddl/hddl.cpp src/hddl/hddl-token.l
	flex --yylineno -o src/hddl/hddl-token.cpp src/hddl/hddl-token.l

src/hddl/hddl.cpp: src/hddl/hddl.y
	bison -v -d -o src/hddl/hddl.cpp src/hddl/hddl.y

src/hddl/hddl.hpp: src/hddl/hddl.cpp

%.o: %.cpp %.hpp src/rannot/rannot.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

%.o: %.cpp src/rannot/rannot.hpp
	${CC} ${COMPILEFLAGS} -o $@ -c $<

src/rannot/rannot-token.cpp: src/rannot/rannot.cpp src/rannot/rannot-token.l
	flex -o src/rannot/rannot-token.cpp src/rannot/rannot-token.l

src/rannot/rannot.cpp: src/rannot/rannot.y
	bison -v -d -o src/rannot/rannot.cpp src/rannot/rannot.y
	
src/rannot/rannot.hpp: src/rannot/rannot.cpp

clear:
	rm src/hddl/hddl-token.cpp
	rm src/hddl/hddl.cpp
	rm src/hddl/hddl.hpp
	rm src/hddl/*o

clear2:
	rm src/rannot/rannot-token.cpp
	rm src/rannot/rannot.cpp
	rm src/rannot/rannot.hpp
	rm src/rannot/rannot/*o
