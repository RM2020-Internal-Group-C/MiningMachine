/**
 * @file main.c
 * @brief Main file
 * @version 0.1
 * @date 2019-09-23
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "PID.h"
#include "ch.h"
#include "uart.h"
#include "hal.h"
#include "miningMachine_motor.h"


// Calculation of BRP:
// Max APB1 clock Frequency: 36MHz
// Target Baudrate: 1MHz
// Nominal bit time: 12t_q

float check;
uint8_t rcValue;
float PIDcheck2;
float PIDcheck3;

//testing GPIO input
static volatile uint8_t inputA1 = 0;
static volatile uint8_t inputA2 = 0;
static volatile uint8_t inputA3 = 0;
static volatile uint8_t inputA4 = 0;

static const PWMConfig pwmcfg = {1000000,
                                 20000,
                                 NULL,
                                 {{PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL}},
                                 0,
                                 0};

void gripperOpen(void)
{
    pwmEnableChannel(&PWMD3, 0, 1778); //open
    chThdSleepMilliseconds(1000);
}

void gripperClose(void)
{
    pwmEnableChannel(&PWMD3, 0, 2150);  //close
    chThdSleepMilliseconds(1000);
}

void gripperReset(void)
{   
    gripperOpen();
    while(palReadPad(GPIOA, 1) != 0)
        MiningMachine_goback();
    MiningMachine_stop();
}

void gripperGoTo(uint8_t location)
{
    if (location == 1) return;
    else
    {
        while(palReadPad(GPIOA, location) != 0 && palReadPad(GPIOA, 4) != 0)
        {
            MiningMachine_move();
        }
        MiningMachine_stop();
    }
    
}

int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();
    // CAN REMAP
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;
    // DBUS REMAP
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
    // Start pwm
    pwmStart(&PWMD3, &pwmcfg);
    // Initialize UART
    UART_Init();

    /***************************************************************
     ****************************四轮***********************************/

    while (true)
    {
        pwmEnableChannel(&PWMD3, 0, 1778); //open
        //chThdSleepMilliseconds(1000);
        //pwmEnableChannel(&PWMD3, 0, 2150);  //close
        //chThdSleepMilliseconds(1000);

        //testing
        rcValue = *UART_Get();
        palSetLine(LINE_LED);
        palSetPad(GPIOA, 8);
        
        //GPIO input testing
        inputA1 = palReadPad(GPIOA, 1);
        inputA2 = palReadPad(GPIOA, 2);
        inputA3 = palReadPad(GPIOA, 3);
        inputA4 = palReadPad(GPIOA, 4);

        if(rcValue != 0)
        {
            gripperReset();             //gripper goes to the starting position
            gripperGoTo(rcValue);       //gripper goes to the designated position
                                        //put down the gripper (rotating the gripper)
            gripperClose();             //close the gripper
            chThdSleepSeconds(3);
                                        //pick up the box (rotating the gripper)
                                        //put down the empty box (rotating the gripper)
            gripperOpen();              //lose the grip
            chThdSleepSeconds(3);       
                                        //raise up the gripper (rotating the gripper)
            gripperReset();             //go back to the starting position
        }
        

    }
}
