cd src;

for d in */; do
	cd $d
	rm -rf *.o
	cd ..
done

cd hddl
rm hddl-token.cpp
rm hddl.cpp
rm hddl.hpp
rm hddl.output

cd ..
cd queryparser
rm queryparser-token.cpp
rm queryparser.cpp
rm queryparser.hpp
rm queryparser.output

cd ..
cd rannot
rm rannot-token.cpp
rm rannot.cpp
rm rannot.hpp
rm rannot.output

cd ..
cd conditionparser
rm conditionparser-token.cpp
rm conditionparser.cpp
rm conditionparser.hpp
rm conditionparser.output