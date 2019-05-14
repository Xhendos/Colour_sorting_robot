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
    def print(self):
        print("(" + str(self.x) + ", " + str(self.y) + ")")
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
placeholders['f1'] = Placeholder(1, 1)
placeholders['f2'] = Placeholder(3, 1)
placeholders['f3'] = Placeholder(3, 3)
placeholders['f4'] = Placeholder(1, 3)

#Input: ta->tb
ta = placeholders[sys.argv[1]]
tb = placeholders[sys.argv[2]]

#Program
dx = ta.x - tb.x
dy = ta.y - tb.y

if abs(dx) == 2:
    direction = 'h' #horizontal
elif abs(dy) == 2:
    direction = 'v' #vertical
elif abs(dx) + abs(dy) == 2:
    direction = 's' #slanted
else:
    exit()

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
        arm = arms[x][y]
        if arm.cardinal == 'e':
            r1 = 240
            r2 = 60
            r3 = 150
        elif arm.cardinal == 'w':
            r1 = 60
            r2 = 240
            r3 = 150
    elif ta.y < tb.y:
        x = ta.x
        y = ta.y + 1
        arm = arms[x][y]
        if arm.cardinal == 'e':
            r1 = 60
            r2 = 240
            r3 = 150
        elif arm.cardinal == 'w':
            r1 = 240
            r2 = 60
            r3 = 150
elif direction == 's':
    p1 = Point(tb.x + dx, tb.y)
    p2 = Point(tb.x, tb.y + dy)
    arm = arms[p1.x][p1.y]
    if arm == 0:
        arm = arms[p2.x][p2.y]
    if arm.cardinal == 'n':
        if arm.x > ta.x:
            r1 = 240
            r2 = 150
            r3 = 150
        elif arm.y < ta.y:
            r1 = 150
            if arm.x < tb.x:
                r2 = 60
            else:
                r2 = 240
            r3 = 150
        elif arm.x < ta.x:
            r1 = 60
            r2 = 150
            r3 = 150
    elif arm.cardinal == 'e':
        if arm.y < ta.y:
            r1 = 240
            r2 = 150
            r3 = 150
        elif arm.x < ta.x:
            r1 = 150
            if arm.y < tb.y:
                r2 = 240
            else:
                r2 = 60
            r3 = 150
        elif arm.y > ta.y:
            r1 = 60
            r2 = 150
            r3 = 150
    elif arm.cardinal == 's':
        if arm.x < ta.x:
            r1 = 240
            r2 = 150
            r3 = 150
        elif arm.y > ta.y:
            r1 = 150
            if arm.x < tb.x:
                r2 = 240
            else:
                r2 = 60
            r3 = 150
        elif arm.x > ta.x:
            r1 = 60
            r2 = 150
            r3 = 150
    elif arm.cardinal == 'w':
        if arm.y > ta.y:
            r1 = 240
            r2 = 150
            r3 = 150
        elif arm.x > ta.x:
            r1 = 150
            if arm.y < tb.y:
                r2 = 60
            else:
                r2 = 240
            r3 = 150
        elif arm.y < ta.y:
            r1 = 60
            r2 = 150
            r3 = 150
arm.print()
print(str(r1) + str(r2) + str(r3))
