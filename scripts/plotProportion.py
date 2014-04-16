#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import pylab
import sys

plt.xlabel('ratio of update queries to distance queries (n^eps)')
plt.ylabel('time (sec)')
plt.title('A total time of 7000 consecutive distance and update queries')

symbols = ['ro', 'bs', 'g<', 'y>', 'kx']
colors = ['r', 'b', 'g', 'y', 'k']

plt.grid(True)

a = [line.split() for line in sys.stdin]
#a = [list(x) for x in zip(*data)];

labels = a[0];
k = (len(labels)-1)/4;
print(labels);
for i in range(0, len(labels)):
    if (i % k != 0):
        labels[i] = "";

plt.gca().set_yscale('log')
pylab.xticks(range(0, len(labels)), labels)

s = 0
for v in a[1:]:
    plt.plot(v, symbols[s])
    plt.plot(v, colors[s])
    s += 1

if len(sys.argv) == 2:
    plt.savefig(sys.argv[1] + '.png');
else:
    plt.show()
