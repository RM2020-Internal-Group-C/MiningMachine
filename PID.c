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

void PIDdInit(pid_t *pid, float kp, float ki, float kd)
{
    pid->dkp = kp;
    pid->dki = ki;
    pid->dkd = kd;
}

void PIDsInit(pid_t *pid, int maxOut, float kp, float ki, float kd)
{
    pid->skp = kp;
    pid->ski = ki;
    pid->skd = kd;
    pid->smaxOut = maxOut;
}

// float absp(float i) { return (i < 0) ? -i : i; }

int PIDDir(pid_t *pid, int get, int set)
{
    pid->dget = get;
    int a = (pid->dget - pid->dlastget + 8192) % 8192;
    int b = 8192 - a;
    if(pid->dget != pid->dlastget){
        if(a > b)
        {
            pid->direction -= a / 1024;
        }
        else if(a < b)
        {
            pid->direction += b/ 1024;
        }
    }
    pid->dset = set;
    pid->derrNOW = set - pid->direction;
    pid->dp = pid->derrNOW * pid->dkp;
    pid->di += pid->derrNOW * pid->dki;
    clamp(&pid->di, 10000);
    pid->dd = (pid->derrNOW - pid->derrLAST) * pid->dkd;
    pid->dout = pid->dp + pid->di + pid->dd;
    pid->derrLAST = pid->derrNOW;
    pid->dlastget = pid->dget;
    if(pid->dout > 800 || pid->dout < -800)
    {
        return pid->dout;
    }
    else
    {
        return 0;
    }

    

}

float PIDSpe(pid_t *pid, int get, int set)
{
    pid->sget = get;
    pid->sset = set;
    pid->serrNOW = set - get;
    pid->sp = pid->serrNOW * pid->skp;
    pid->si += pid->serrNOW * pid->ski;
    clamp(&pid->si, 10000);
    pid->sd = (pid->serrNOW - pid->serrLAST) * pid->skd;
    pid->sout = pid->sp + pid->si + pid->sd;
    pid->serrLAST = pid->serrNOW;
    clamp(&pid->sout, pid->smaxOut);
    return pid->sout;
}

int PIDcheck(pid_t *pid)
{
    return pid->direction;
}