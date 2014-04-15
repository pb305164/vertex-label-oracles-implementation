ALL= oracle_main
CXX= g++
//CXXFLAGS= -std=c++11 -O3 -Wall
CXXFLAGS= -std=c++11 -ggdb -Wall -pg

all: $(ALL)

planar.o: src/planar.cpp src/planar.h
	$(CXX) -c $(CXXFLAGS) src/planar.cpp

oracle_internal.o: src/oracle_internal.cpp src/oracle_internal.h src/planar.h
	$(CXX) -c $(CXXFLAGS) src/oracle_internal.cpp

planar_oracle.o: src/planar_oracle.cpp src/planar_oracle.h src/planar.h
	$(CXX) -c $(CXXFLAGS) src/planar_oracle.cpp

oracle_main: src/oracle_main.cpp src/oracle_general_3approx.h src/oracle_general_5approx.h src/oracle_general_approx.h src/oracle_naive.h src/oracle_tester.h src/full_planar_oracle.h planar.o planar_oracle.o oracle_internal.o
	$(CXX) -o oracle_main $(CXXFLAGS) src/oracle_main.cpp planar.o planar_oracle.o oracle_internal.o

clean:
	rm -f *.o $(ALL)
