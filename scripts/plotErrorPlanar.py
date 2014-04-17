#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import pylab
import sys

fig, ax1 = plt.subplots()
ax2 = ax1.twinx();
ax = [ax1, ax2];

ax1.set_xlabel('logarithm of ratio of update queries to distance queries')
ax1.set_ylabel('time (sec)')

symbols = ['-ro', '-.bs']
colors = ['r', 'b']

plt.grid(True)

a = [line.split() for line in sys.stdin]
a = [list(x) for x in zip(*a)];

labels = a[0];
k = (len(labels)-1)/4;
print(labels);
for i in range(0, len(labels)):
    if (i % k != 0):
        labels[i] = "";

pylab.xticks(range(0, len(labels)), labels)

s = 0
for v in a[1:]:
    ax[s].plot(v, symbols[s])
#    plt.plot(v, colors[s])
    s += 1

if len(sys.argv) == 2:
    plt.savefig(sys.argv[1] + '.png');
else:
    plt.show()
