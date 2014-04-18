#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import pylab
import sys

plt.xlabel('logarithm of ratio of update queries to distance queries')
plt.ylabel('time (sec)')

symbols = ['-.ro', '-.bs', '-g<', '-y>', ':kx', ':k+']
#symbols = ['-ro', '-.bs', ':kx']

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

legend = ['naive', 'gen. 3-app.', 'gen. 5-app. v1', 'gen. 5-app. v2', 'planar 2-app.']
#legend = ['naive', 'gen. 3-app.', 'planar 2-app.']

s = 0
for v in a[1:]:
    plt.plot(v, symbols[s], label=legend[s])
#    plt.plot(v, colors[s])
    s += 1

plt.legend(loc=4)

if len(sys.argv) == 2:
    plt.savefig(sys.argv[1] + '.png');
else:
    plt.show()
