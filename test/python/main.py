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

arms = [[0]*5 for i in range(5)]
arms[1][0] = Arm(1, 0, 'n')

t6 = Placeholder(2, 4)
t7 = Placeholder(0, 4)
a6 = Arm(1, 4, 's')

dx = abs(t6.x - t7.x)
dy = abs(t6.y - t7.y)

if dx == 2:
    direction = 'h'
elif dy == 2:
    direction = 'v'

if direction == 'h':
    if t6.x > t7.x:
        a = Point(t6.x - 1, t6.y)
arm = arms[1][0]
print(str(arm.x) + str(arm.y) + arm.cardinal)
