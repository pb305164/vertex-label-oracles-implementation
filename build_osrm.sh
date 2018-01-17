#!/bin/bash

cd oracles/src/osrm
mkdir -p build
cd build
cmake ..
cmake --build .
