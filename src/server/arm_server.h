#ifndef _ARM_SERVER_H
#define _ARM_SERVER_H

#include "FreeRTOS.h"
#include "task.h"

#define ARM0        ( 1 << 0 )
#define ARM1        ( 1 << 1 )
#define ARM2        ( 1 << 2 )
#define ARM3        ( 1 << 3 )
#define ARM4        ( 1 << 4 )
#define ARM5        ( 1 << 5 )
#define ARM6        ( 1 << 6 )
#define ARM7        ( 1 << 7 )
#define ALL_ARMS    ( ARM0 | ARM1 | ARM2 | ARM3 | ARM4 | ARM5 | ARM6 | ARM7 )
#define ODD_ARMS    ( ARM1 | ARM3 | ARM5 | ARM7 )
#define EVEN_ARMS   ( ARM0 | ARM2 | ARM4 | ARM6 )

enum set_of_arms {
    SET0 = ARM0,
    SET1 = ARM1,
    SET2 = ARM2,
    SET3 = ARM3,
    SET4 = ARM4,
    SET5 = ARM5,
    SET6 = ARM6,
    SET7 = ARM7,
    SET8 = ALL_ARMS,
    SET9 = ODD_ARMS,
    SET10 = EVEN_ARMS,
};

enum arm_movement {
    EMPTY = 0,
    REST_POSITION,
    DISPLACE_60_150,
    DISPLACE_60_240,
    DISPLACE_150_60,
    DISPLACE_150_240,
    DISPLACE_240_60,
    DISPLACE_240_150,
};

struct arm_server_message {
    enum set_of_arms arms;
    enum arm_movement movement;
    unsigned char buffer_movement;
    TaskHandle_t sender_of_message;
};

#endif /* _ARM_SERVER_H */

