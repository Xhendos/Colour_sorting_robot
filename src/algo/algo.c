#include "octo.h"
#include <stdlib.h>

typedef enum {
    HORIZONTAL,
    VERTICAL,
    DIAGONAL,
} direction_t;

typedef enum {
    NORTH,
    EAST,
    SOUTH,
    WEST,
} cardinal_t;

typedef struct {
    uint8_t x;
    uint8_t y;
} point_t;

typedef struct {
    cardinal_t cardinal;
    arm_t n;
} Arm_t;

instruction_t edge_to_instruction(placeholders_t ta, placeholders_t tb)
{
    Arm_t *arms[8][8];

    Arm_t arm0 = {NORTH, ARM_0};
    Arm_t arm1 = {EAST, ARM_1};
    Arm_t arm2 = {EAST, ARM_2};
    Arm_t arm3 = {SOUTH, ARM_3};
    Arm_t arm4 = {SOUTH, ARM_4};
    Arm_t arm5 = {WEST, ARM_5};
    Arm_t arm6 = {WEST, ARM_6};
    Arm_t arm7 = {NORTH, ARM_7};

    arms[1][0] = &arm0;
    arms[0][1] = &arm1;
    arms[0][3] = &arm2;
    arms[1][4] = &arm3;
    arms[3][4] = &arm4;
    arms[4][3] = &arm5;
    arms[4][1] = &arm6;
    arms[3][0] = &arm7;

    point_t placeholders[12] = {
        {0, 0},
        {2, 0},
        {4, 0},
        {4, 2},
        {4, 4},
        {2, 4},
        {0, 4},
        {0, 2},
        {1, 1},
        {3, 1},
        {3, 3},
        {1, 3},
    };

    int8_t dx = placeholders[ta].x - placeholders[tb].x;
    int8_t dy = placeholders[ta].y - placeholders[tb].y;
    direction_t direction;

    if (abs(dx) == 2)
    {
        direction = HORIZONTAL;
    }
    else if (abs(dy) == 2)
    {
        direction = VERTICAL;
    }
    else if (abs(dx) + abs(dy) == 2)
    {
        direction = DIAGONAL;
    }
    else
    {
        instruction_t instruction = {1, 0, 0, 0, 255, 0, 0};
        return instruction;
    }

    uint8_t tax = placeholders[ta].x;
    uint8_t tay = placeholders[ta].y;
    uint8_t tbx = placeholders[tb].x;
    uint8_t tby = placeholders[tb].y;

    Arm_t *arm;
    uint8_t x;
    uint8_t y;
    uint16_t r1;
    uint16_t r2;

    if (direction == HORIZONTAL)
    {
        if (tax > tbx)
        {
            x = tax - 1;
            y = tay;
            arm = arms[x][y];
            if (arm->cardinal == NORTH)
            {
                r1 = 60;
                r2 = 240;
            }
            else if (arm->cardinal == SOUTH)
            {
                r1 = 240;
                r2 = 60;
            }
        }
        else if (tax < tbx)
        {
            x = tax + 1;
            y = tay;
            arm = arms[x][y];
            if (arm->cardinal == NORTH)
            {
                r1 = 240;
                r2 = 60;
            }
            else if (arm->cardinal == SOUTH)
            {
                r1 = 60;
                r2 = 240;
            }
        }
    }
    else if (direction == VERTICAL)
    {
        if (tay > tby)
        {
            x = tax;
            y = tay - 1;
            arm = arms[x][y];
            if (arm->cardinal == EAST)
            {
                r1 = 240;
                r2 = 60;
            }
            else if (arm->cardinal == WEST)
            {
                r1 = 60;
                r2 = 240;
            }
        }
        else if (tay < tby)
        {
            x = tax;
            y = tay + 1;
            arm = arms[x][y];
            if (arm->cardinal == EAST)
            {
                r1 = 60;
                r2 = 240;
            }
            else if (arm->cardinal == WEST)
            {
                r1 = 240;
                r2 = 60;
            }
        }
    }
    else if (direction == DIAGONAL)
    {
        point_t p1 = {tbx + dx, tby};
        point_t p2 = {tbx, tby + dy};

        uint8_t armx;
        uint8_t army;

        armx = p1.x;
        army = p1.y;

        arm = arms[armx][army];

        if (arm == NULL)
        {
            armx = p2.x;
            army = p2.y;
            arm = arms[armx][army];
        }

        if (arm->cardinal == NORTH)
        {
            if (armx > tax)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (army < tay)
            {
                r1 = 150;
                if (armx < tbx)
                {
                    r2 = 60;
                }
                else
                {
                    r2 = 240;
                }
            }
            else if (armx < tax)
            {
                r1 = 60;
                r2 = 150;
            }
        }
        else if (arm->cardinal == EAST)
        {
            if (army < tay)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (armx < tax)
            {
                r1 = 150;
                if (army < tby)
                {
                    r2 = 240;
                }
                else
                {
                    r2 = 60;
                }
            }
            else if (army > tay)
            {
                r1 = 60;
                r2 = 150;
            }
        }
        else if (arm->cardinal == SOUTH)
        {
            if (armx < tax)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (army > tay)
            {
                r1 = 150;
                if (armx < tbx)
                {
                    r2 = 240;
                }
                else
                {
                    r2 = 60;
                }
            }
            else if (armx > tax)
            {
                r1 = 60;
                r2 = 150;
            }
        }
        else if (arm->cardinal == WEST)
        {
            if (army > tay)
            {
                r1 = 240;
                r2 = 150;
            }
            else if (armx > tax)
            {
                r1 = 150;
                if (army < tby)
                {
                    r2 = 60;
                }
                else
                {
                    r2 = 240;
                }
            }
            else if (army < tay)
            {
                r1 = 60;
                r2 = 150;
            }
        }
    }

    instruction_t instruction = {0, arm->n, r1, r2, 0, ta, tb};

    return instruction;
}
