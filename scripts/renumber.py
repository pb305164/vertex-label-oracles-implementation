#!/usr/bin/python

import sys

sys.stdin.readline()

vcount = 0
vdict = {}

edges = [line.split() for line in sys.stdin]
for edge in edges:
    for vertex in edge:
        if not vertex in vdict:
             vdict[vertex] = vcount
             vcount += 1

print vcount
for edge in edges:
    print vdict[edge[0]], vdict[edge[1]]
