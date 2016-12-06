"""
runner.py
by Ted Morin

repeatedly runs the simulation projectpart4.py
"""

import sys
import os

n = 2
c = 1

for i in range(n):
    p = 4 + i % 2
    os.system("python projectpart%d.py %d" % (p,c))
