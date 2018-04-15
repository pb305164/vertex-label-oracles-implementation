#!/bin/bash

cd oracles/src
git clone --depth=1 https://github.com/RoutingKit/RoutingKit.git
cd RoutingKit
make
