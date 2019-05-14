#!/usr/bin/python

import sys

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Placeholder(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Arm(object):
    def __init__(self, x, y, c):
        self.x = x
        self.y = y
        self.cardinal = c
#Arms
arms = [[0] * 5 for i in range(5)]
arms[1][0] = Arm(1, 0, 'n')
arms[3][0] = Arm(3, 0, 'n')
arms[4][1] = Arm(4, 1, 'w')
arms[4][3] = Arm(4, 3, 'w')
arms[3][4] = Arm(3, 4, 's')
arms[1][4] = Arm(1, 4, 's')
arms[0][3] = Arm(0, 3, 'e')
arms[0][1] = Arm(0, 1, 'e')

#Placeholders
placeholders = {}
placeholders['t1'] = Placeholder(0, 0)
placeholders['t2'] = Placeholder(2, 0)
placeholders['t3'] = Placeholder(4, 0)
placeholders['t4'] = Placeholder(4, 2)
placeholders['t5'] = Placeholder(4, 4)
placeholders['t6'] = Placeholder(2, 4)
placeholders['t7'] = Placeholder(0, 4)
placeholders['t8'] = Placeholder(0, 2)

#Input: ta->tb
ta = placeholders[sys.argv[1]]
tb = placeholders[sys.argv[2]]

#Program
dx = abs(ta.x - tb.x)
dy = abs(ta.y - tb.y)

if dx == 2:
    direction = 'h'
elif dy == 2:
    direction = 'v'

if direction == 'h':
    if ta.x > tb.x:
        x = ta.x - 1
        y = ta.y
        arm = arms[x][y]
        if arm.cardinal == 'n':
            r1 = 60
            r2 = 240
            r3 = 150
        elif arm.cardinal == 's':
            r1 = 240
            r2 = 60
            r3 = 150
    elif ta.x < tb.x:
        x = ta.x + 1
        y = ta.y
        arm = arms[x][y]
        if arm.cardinal == 'n':
            r1 = 240
            r2 = 60
            r3 = 150
        elif arm.cardinal == 's':
            r1 = 60
            r2 = 240
            r3 = 150
elif direction == 'v':
    if ta.y > tb.y:
        x = ta.x
        y = ta.y - 1
    else:
        x = ta.x
        y = ta.y + 1

print(str(r1) + str(r2) + str(r3))
