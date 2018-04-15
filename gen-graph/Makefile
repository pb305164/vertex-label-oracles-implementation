FLAGS=-Wall -Wextra -pedantic -O3 -std=c++11 -g -I/home/anka/usr/include
LIB_FLAGS=-Wall -Wextra -Werror -pedantic -Wundef -Wshadow -Wcast-align -Wcast-qual -Wold-style-cast -std=c++11 -c -MMD -MP -O3

all: gen-graph common.o pugixml.o KDTree.o parse_osm.o

common.o: common.h common.cpp
	g++ -c ${FLAGS} common.cpp

pugixml.o: pugixml.hpp pugiconfig.hpp pugixml.cpp
	g++ -c ${LIB_FLAGS} pugixml.cpp

KDTree.o: KDTree.cpp KDTree.h
	g++ -c ${FLAGS} KDTree.cpp

make_planar.o: make_planar.cpp make_planar.h
	g++ -c ${FLAGS} make_planar.cpp

parse_osm.o: parse_osm.cpp parse_osm.h
	g++ -c ${FLAGS} parse_osm.cpp

gen-graph: main.cpp common.o pugixml.o KDTree.o make_planar.o parse_osm.o
	g++ -o gen-graph ${FLAGS} -L/home/anka/usr/lib main.cpp parse_osm.o pugixml.o common.o KDTree.o make_planar.o -lGeographic -lmpfr

clean:
	rm -f *.o *.d gen-graph
