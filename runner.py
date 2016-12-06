"""
runner.py
by Ted Morin

repeatedly runs the simulation projectpart4.py
"""

import sys
import os

n = 1
c = 1

for i in range(n):
    os.system("python projectpart4.py %d" % c)
