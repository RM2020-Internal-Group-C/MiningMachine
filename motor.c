#include "motor.h"
#include "PID.h"
#include "ch.h"
#include "dbus.h"
#include "hal.h"

static CANTxFrame txmsg;
static CANRxFrame rxmsg;
static int16_t motorSpeed[2];
int16_t motorDirection[2];
float SpeedResult[2] = {0, 0};
float DirectionResult[2] = {0, 0};
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) | CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

static THD_WORKING_AREA(can_rx_thd_wa, 256);
static THD_FUNCTION(can_rx_thd, p)
{
    (void)p;
    while (true)
    {
        if (canReceiveTimeout(
                   &CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_MS2I(1)) == MSG_OK)
        {
            // receiving rpm
            if (rxmsg.SID == 0x201)
            {
                motorSpeed[0] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                motorDirection[0] = rxmsg.data8[0] << 8 | rxmsg.data8[1];
                pidmotor[0].dget = motorDirection[0];
                int16_t a = (pidmotor[0].dget  - pidmotor[0].dlastget  + 8192 + 4096) % 8192 - 4096;
                pidmotor[0].direction += a;
                pidmotor[0].dlastget = pidmotor[0].dget;
            }

            if (rxmsg.SID == 0x202)
            {
                motorSpeed[1] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                motorDirection[1] = rxmsg.data8[0] << 8 | rxmsg.data8[1];
                pidmotor[1].dget = motorDirection[1];
                int16_t a = (pidmotor[1].dget  - pidmotor[1].dlastget  + 8192 + 4096) % 8192 - 4096;
                pidmotor[1].direction += a;
                pidmotor[1].dlastget = pidmotor[1].dget;
            }
        }
    }
}

static THD_WORKING_AREA(can_tx_thd_wa, 128);
static THD_FUNCTION(can_tx_thd, p)
{
    (void)p;
    while (true)
    {
        txmsg.DLC = 8;
        txmsg.IDE = CAN_IDE_STD;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.SID = 0x200;
        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
        chThdSleepMilliseconds(1);
    }
};

// if (palReadPao(GPIOA, 1) == 0)

void setDistance(int i, int target)
{
    DirectionResult[i] = PIDDir(&pidmotor[i], target);
    SpeedResult[i] = PIDSpe(&pidmotor[i], motorSpeed[i], DirectionResult[i]);
    txmsg.data8[i * 2] = (int)SpeedResult[i] >> 8;
    txmsg.data8[i * 2 + 1] = (int)SpeedResult[i] & 0xFF;
}

float motorSpeedGet(int i)
{
    // chSysLock();
    float value = motorSpeed[i];
    // chSysUnlock();
    return value;
}

void motorInit(void)
{
    PIDsInit(&pidmotor[0], 3500, 10, 0.28, 25);
    PIDdInit(&pidmotor[0], 0.1, 0, 7);
    PIDsInit(&pidmotor[1], 3500, 10, 0.28, 25);
    PIDdInit(&pidmotor[1], 0.1, 0, 7);
    canStart(&CAND1, &cancfg);
    chThdCreateStatic(can_rx_thd_wa,
                      sizeof(can_rx_thd_wa),
                      NORMALPRIO + 15,
                      can_rx_thd,
                      NULL);
    chThdCreateStatic(can_tx_thd_wa,
                      sizeof(can_tx_thd_wa),
                      NORMALPRIO + 20,
                      can_tx_thd,
                      NULL);
}