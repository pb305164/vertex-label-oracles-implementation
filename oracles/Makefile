ALL = oracle_main run_tests new_run_test gen_test new_gen_test
CXX = g++
CXXFLAGS = -std=c++14 -O3 -march=native -Wall -g -I/home/anka/usr/include -I./src -I./src/osrm/src -I./src/osrm/include -I./src/osrm/third_party/variant/include -I./src/RoutingKit/include
#CXXFLAGS = -std=c++14 -Wall -g -I/home/anka/usr/include -I./src -I./src/osrm/src -I./src/osrm/include -I./src/osrm/third_party/variant/include
LIBS = -L/home/anka/usr/lib -L/home/anka/downloads/tbb-2018_U2/build/linux_intel64_gcc_cc6.4.0_libc2.17_kernel2.6.32_release -L ./src/RoutingKit/lib -lboost_system -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_date_time -lboost_thread -ltbb -lrt -pthread -lroutingkit


all: $(ALL)

planar.o: src/planar.cpp src/planar.h
	$(CXX) -c $(CXXFLAGS) src/planar.cpp

oracle_internal.o: src/oracle_internal.cpp src/oracle_internal.h src/planar.h
	$(CXX) -c $(CXXFLAGS) src/oracle_internal.cpp

planar_oracle.o: src/planar_oracle.cpp src/planar_oracle.h src/planar.h
	$(CXX) -c $(CXXFLAGS) src/planar_oracle.cpp

oracle_main: src/oracle_main.cpp src/oracle_general_3approx.h src/oracle_general_5approx.h src/oracle_general_approx.h src/oracle_naive.h src/oracle_tester.h src/full_planar_oracle.h planar.o planar_oracle.o oracle_internal.o
	$(CXX) -o oracle_main $(CXXFLAGS) src/oracle_main.cpp planar.o planar_oracle.o oracle_internal.o

lca_tree.o: src/LCATree.h src/LCAOracle.cpp
	$(CXX) -c -o lca_tree.o $(CXXFLAGS) src/LCATree.cpp

lca_oracle.o: src/LCAOracle.h src/LCAOracle.cpp
	$(CXX) -c -o lca_oracle.o $(CXXFLAGS) src/LCAOracle.cpp

osrm_oracle.o: src/osrm_oracle.h src/osrm_oracle.cpp
	$(CXX) -c -o osrm_oracle.o $(CXXFLAGS) src/osrm_oracle.cpp

routing_kit_oracle.o: src/routing_kit_oracle.h src/routing_kit_oracle.cpp
	$(CXX) -c -o routing_kit_oracle.o $(CXXFLAGS) src/routing_kit_oracle.cpp

run_tests: src/run_tests.cpp src/heap.h src/hierarchy_oracle.h src/hierarchy_oracle_base.h src/hierarchy_oracle_light.h src/hierarchy_oracle_light_path.h src/dijkstra_oracle.h src/astar_oracle.h src/oracle_general_3approx.h src/oracle_general_5approx.h src/oracle_general_approx.h src/oracle_naive.h src/oracle_tester.h src/full_planar_oracle.h planar.o planar_oracle.o oracle_internal.o osrm_oracle.o
	$(CXX) -o run_tests $(CXXFLAGS) src/run_tests.cpp planar.o planar_oracle.o oracle_internal.o osrm_oracle.o src/osrm/build/libosrm.a $(LIBS)

new_run_test: src/new_run_test.cpp src/heap.h src/hierarchy_oracle.h src/hierarchy_oracle_base.h src/hierarchy_oracle_light.h src/hierarchy_oracle_light_path.h src/dijkstra_oracle.h src/astar_oracle.h src/oracle_general_3approx.h src/oracle_general_5approx.h src/oracle_general_approx.h src/oracle_naive.h src/oracle_tester.h src/full_planar_oracle.h planar.o planar_oracle.o oracle_internal.o osrm_oracle.o routing_kit_oracle.o lca_tree.o lca_oracle.o
	$(CXX) -o new_run_test $(CXXFLAGS) src/new_run_test.cpp planar.o planar_oracle.o oracle_internal.o osrm_oracle.o routing_kit_oracle.o lca_tree.o lca_oracle.o src/osrm/build/libosrm.a $(LIBS)

gen_test: src/gen_test.cpp src/read_graph.h src/precision.h
	$(CXX) -o gen_test $(CXXFLAGS) src/gen_test.cpp

new_gen_test: src/new_gen_test.cpp src/read_graph.h src/precision.h
	$(CXX) -o new_gen_test $(CXXFLAGS) src/new_gen_test.cpp

clean:
	rm -f *.o $(ALL)
