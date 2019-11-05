#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "stdint.h"

#define MAX_SPEED       8000
float motorSpeedGet(int i);
void motorInit(void);
void setSpeed(int i, int target);
#endif /* __MOTOR_H_ */

