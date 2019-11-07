#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "stdint.h"

#define MAX_SPEED       600
#define GRIP_MOTOR_ID   201
#define MOVE_MOTOR_ID   202
#define GRIP_MOTOR_INDEX   0
#define MOVE_MOTOR_INDEX   1

#define GRIP_MOTOR_SPEED 50
#define MOVE_MOTOR_SPEED 100

float motorSpeedGet(int i);
void motorInit(void);
void MiningMachine_move(void);
void MiningMachine_goback(void);
void MiningMachine_stop(void);
#endif /* __MOTOR_H_ */