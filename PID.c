#include "PID.h"

pid_t pidmotor[2] = {{0}, {0}};

static void clamp(float *a, float max)
{
    if (*a > max)
    {
        *a = max;
    }
    if (*a < -max)
    {
        *a = -max;
    }
}

void PIDInit(pid_t *pid, int maxOut, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOut = maxOut;
}

// float absp(float i) { return (i < 0) ? -i : i; }

float PIDDir(pid_t *pid, int get, int set)
{
    pid->get = get;
    int a = (pid->get - pid->lastget + 8192) % 8192;
    int b = 8192 - a;
    if(a > b)
    {
        pid->Direction -= a;
    }
    else if(a < b)
    {
        pid->Direction += b;
    }
    pid->set = set;
    pid->errNOW = set - pid->Direction;
    pid->p = pid->errNOW * pid->kp;
    pid->i += pid->errNOW * pid->ki;
    pid->d = (pid->errNOW - pid->errLAST) * pid->kd;
    pid->out = pid->p + pid->i + pid->d;
    pid->errLAST = pid->errNOW;
    pid->lastget = pid->get;
    clamp(&pid->out, pid->maxOut);
    return pid->out;
}



int PIDcheck(pid_t *pid)
{
    return pid->Direction;
}