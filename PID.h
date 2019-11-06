#ifndef __PID_H__

#define __PID_H__

typedef struct
{
    float kp;
    float ki;
    float kd;
    int set;
    int get;
    int lastget;
    float errNOW;
    float errLAST;
    float out;
    float maxOut;
    float p;
    float i;
    float d;
    int Direction;
} pid_t;
//Initiallize pid: maxOut:maximum outpus, Kp, Ki, Kd 
void PIDInit(pid_t *pid, int maxOut, float kp, float ki, float kd);

float PIDSet(pid_t *pid, int get, int set);

int PIDcheck(pid_t *pid);

extern pid_t pidmotor[2];
#endif