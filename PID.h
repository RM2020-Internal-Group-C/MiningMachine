#ifndef __PID_H__
#define __PID_H__
#include <stdint.h>

typedef struct
{
    float dkp;
    float dki;
    float dkd;
    int16_t dset;
    int16_t dget;
    int16_t dlastget;
    float derrNOW;
    float derrLAST;
    int dout;
    float dp;
    float di;
    float dd;
    int direction;

    float skp;
    float ski;
    float skd;
    int sset;
    int sget;
    float serrNOW;
    float serrLAST;
    float sout;
    float smaxOut;
    float sp;
    float si;
    float sd;
} pid_t;
//Initiallize pid: maxOut:maximum outpus, Kp, Ki, Kd 
void PIDdInit(pid_t *pid, float kp, float ki, float kd);
void PIDsInit(pid_t *pid, int maxOut, float kp, float ki, float kd);

int PIDDir(pid_t *pid,int set);
float PIDSpe(pid_t *pid, int get, int set);

int PIDcheck(pid_t *pid);

extern pid_t pidmotor[2];
#endif