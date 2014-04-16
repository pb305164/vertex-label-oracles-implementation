#!/usr/bin/python

import sys

if (len(sys.argv) == 5):
    d1File = open(sys.argv[1], 'r')
    g1File = open(sys.argv[2], 'r')
    d2File = open(sys.argv[3], 'w')
    g2File = open(sys.argv[4], 'w')
else:
    d1File = sys.stdin
    d2File = sys.stdout

vcount = 0
vdict = {}

edges = [line.split() for line in d1File]
for edge in edges:
    for vertex in edge:
        if not vertex in vdict:
             vdict[vertex] = vcount
             vcount += 1

print >>d2File, vcount
for edge in edges:
    print >>d2File, vdict[edge[0]], vdict[edge[1]]

if not g1File:
    exit(0)

groups = [line.split() for line in g1File]
for group in groups:
    printstr = ""
    for element in group:
        printstr += str(vdict[element]) + " ";
    print >>g2File, printstr
