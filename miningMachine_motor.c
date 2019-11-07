#include "miningMachine_motor.h"
#include "PID.h"
#include "ch.h"
//#include "dbus.h"
#include "hal.h"

// int16_t check;

CANTxFrame txmsg;
CANRxFrame rxmsg;
int16_t motorSpeed[4];
int16_t result[4] = {0, 0, 0, 0};
uint16_t rxcnt[4] = {0};
int16_t move_motor_targetSpeed = 0;
int16_t grip_motor_targetSpeed = 0;
static const CANConfig cancfg = {CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
                                 CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
                                     CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

static THD_WORKING_AREA(can_rx_thd_wa, 512);
static THD_FUNCTION(can_rx_thd, p) {
  (void)p;
  while (true) {
    if (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_MS2I(1)) ==
        MSG_OK) {
      // receiving rpm
      if (rxmsg.SID == 0x201) {
        motorSpeed[0] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
        rxcnt[0]++;
        motorSpeed[0] = (motorSpeed[0]) / 19;
      }
      if (rxmsg.SID == 0x202) {
        motorSpeed[1] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
        rxcnt[1]++;
        motorSpeed[1] = (motorSpeed[1]) / 19;
      }
      if (rxmsg.SID == 0x203) {
        motorSpeed[2] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
        rxcnt[2]++;
        motorSpeed[2] = (motorSpeed[2]) / 27;
      }
      if (rxmsg.SID == 0x204) {
        motorSpeed[3] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
        rxcnt[3]++;
        motorSpeed[3] = (motorSpeed[3]) / 19;
      }
    }
  }
}

static THD_WORKING_AREA(can_tx_thd_wa, 256);
static THD_FUNCTION(can_tx_thd, p) {
  (void)p;
  while (true) {
    // check = RCGet()->channel3*MAX_SPEED/660;canReceiveTimeout
    txmsg.DLC = 8;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.SID = 0x200;

    setSpeed(MOVE_MOTOR_INDEX, move_motor_targetSpeed);
    setSpeed(GRIP_MOTOR_INDEX, grip_motor_targetSpeed);

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
    chThdSleepMilliseconds(5);
  }
};

void setSpeed(int i, float target) {
  result[i] = PIDSet(&pidWheel[i], motorSpeed[i], target);
  txmsg.data8[i * 2] = (int)result[i] >> 8;
  txmsg.data8[i * 2 + 1] = (int)result[i] & 0xFF;
}

void MiningMachine_move(void) { move_motor_targetSpeed = MOVE_MOTOR_SPEED; }

void MiningMachine_stop() { move_motor_targetSpeed = 0; }

void MiningMachine_goback() { move_motor_targetSpeed = -MOVE_MOTOR_SPEED; }

float motorSpeedGet(int i) {
  // chSysLock();
  float value = motorSpeed[i];
  // chSysUnlock();
  return value;
}

void motorInit(void) {
  PIDInit(&pidWheel[0], MAX_SPEED, 20, 0.1, 0);
  PIDInit(&pidWheel[1], MAX_SPEED, 20, 0.1, 0);
  PIDInit(&pidWheel[2], MAX_SPEED, 20, 0.1, 0);
  PIDInit(&pidWheel[3], MAX_SPEED, 20, 0.1, 0);
  canStart(&CAND1, &cancfg);

  chThdCreateStatic(can_rx_thd_wa, sizeof(can_rx_thd_wa), NORMALPRIO + 15,
                    can_rx_thd, NULL);
  chThdCreateStatic(can_tx_thd_wa, sizeof(can_tx_thd_wa), NORMALPRIO + 20,
                    can_tx_thd, NULL);
}