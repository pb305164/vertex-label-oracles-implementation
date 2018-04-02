#!/bin/bash

city=$1
if [ "$2" != nogen ]; then

cd tests
wget http://download.bbbike.org/osm/bbbike/$city/$city.osm.gz
gunzip $city.osm.gz
../oracles/src/osrm/build/osrm-extract -p ../oracles/src/osrm/profiles/profile.lua $city.osm
../oracles/src/osrm/build/osrm-contract $city.osrm
cd ..

./gen-graph/gen-graph ./tests/$city.osm > ./tests/$city.graph
./oracles/gen_test ./tests/$city.graph > ./tests/$city.test

fi

if [ "$2" != gen ]; then
for ((i=1; i<=5; i++)); do
  /usr/bin/time -v \
    ./oracles/run_tests $i ./tests/$city.osrm ./tests/$city.graph ./tests/$city.test \
    2> >(grep "Maximum resident") | tee -a $city.log
done
fi
