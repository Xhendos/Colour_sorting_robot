#!/usr/bin/python

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

arms = [[0] * 5 for i in range(5)]
arms[1][4] = Arm(1, 4, 's')

placeholders = {}
placeholders['t1'] = Placeholder(0, 0)
placeholders['t2'] = Placeholder(2, 0)
placeholders['t6'] = Placeholder(2, 4)
placeholders['t7'] = Placeholder(0, 4)

ta = placeholders['t6']
tb = placeholders['t7']

print(str(ta.x) + str(ta.y))
print(str(tb.x) + str(tb.y))

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
    else:
        x = ta.x + 1
        y = ta.y
elif direction == 'v':
    if ta.y > tb.y:
        x = ta.x
        y = ta.y - 1
    else:
        x = ta.x
        y = ta.y + 1

print(arms[x][y].x)    
