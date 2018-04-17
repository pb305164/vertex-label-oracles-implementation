#!/bin/bash

city=$1
if [ "$2" != nogen ]; then

cd tests
#wget http://download.bbbike.org/osm/bbbike/$city/$city.osm.gz
#gunzip $city.osm.gz
../gen-graph/gen-graph $city.osm > $city.graph
../oracles/src/osrm/build/osrm-extract -p ../oracles/src/osrm/profiles/profile.lua Graph_$city.osm
../oracles/src/osrm/build/osrm-contract Graph_$city.osrm
cd ..

#./oracles/new_gen_test -S 10 -SS 10000 -Q 100 -T 0 -CS 0 -CE 1 ./tests/$city.graph > ./tests/vv0_$city.test
#./oracles/new_gen_test -S 10 -SS 10000 -Q 010 -T 0 -CS 0 -CE 1 ./tests/$city.graph > ./tests/vl0_$city.test
#./oracles/new_gen_test -S 10 -SS 10000 -Q 100 -T 1 -CS 0 -CE 1 ./tests/$city.graph > ./tests/vv1_$city.test
#./oracles/new_gen_test -S 10 -SS 10000 -Q 010 -T 1 -CS 0 -CE 1 ./tests/$city.graph > ./tests/vl1_$city.test

fi

if [ "$2" != gen ]; then
for ((i=1; i<=5; i++)); do
  /usr/bin/time -v \
    ./oracles/run_tests $i ./tests/$city.osrm ./tests/$city.graph ./tests/$city.test \
    2> >(grep "Maximum resident") | tee -a $city.log
done
fi
