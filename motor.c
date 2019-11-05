#include "motor.h"
#include "PID.h"
#include "ch.h"
#include "dbus.h"
#include "hal.h"


int16_t check;

CANTxFrame txmsg;
CANRxFrame rxmsg;
int motorSpeed[4];
float result[4] = {0, 0, 0, 0};
uint16_t rxcnt[4] = {0};
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) | CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

static THD_WORKING_AREA(can_rx_thd_wa, 512);
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
                motorSpeed[0] = rxmsg.data8[1] << 8 | rxmsg.data8[0];
                rxcnt[0]++;
            }
            // if (rxmsg.SID == 0x202)
            // {
            //     motorSpeed[1] = rxmsg.data8[1] << 8 | rxmsg.data8[0];
            //     rxcnt[1]++;
            // }

        }
    }
}

static THD_WORKING_AREA(can_tx_thd_wa, 256);
static THD_FUNCTION(can_tx_thd, p)
{
    (void)p;
    while (true)
    {
        txmsg.DLC = 8;
        txmsg.IDE = CAN_IDE_STD;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.SID = 0x200;
        setSpeed(0, 50000);
        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
        chThdSleepMilliseconds(5);
    }
};

void setSpeed(int i, int target)
{
    result[i] = PIDSet(&pidmotor[i], motorSpeed[i], target);
    txmsg.data8[i * 2] = (int)result[i] >> 8;
    txmsg.data8[i * 2 + 1] = (int)result[i] & 0xFF;
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
    PIDInit(&pidmotor[0], MAX_SPEED, 5, 0, 0);
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