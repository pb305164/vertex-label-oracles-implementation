ALL= oracle_main
CXX= g++
CXXFLAGS= -std=c++11 -O3
//CXXFLAGS= -std=c++11 -ggdb -Wall -pg

oracle_main: src/oracle_main.cpp src/oracle_general_3approx.h src/oracle_general_5approx.h src/oracle_general_approx.h src/oracle_naive.h src/oracle_tester.h
	$(CXX) -o oracle_main $(CXXFLAGS) src/oracle_main.cpp
