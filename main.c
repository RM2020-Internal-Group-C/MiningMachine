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
#include "dbus.h"
#include "hal.h"
#include "motor.h"

// Calculation of BRP:
// Max APB1 clock Frequency: 36MHz
// Target Baudrate: 1MHz
// Nominal bit time: 12t_q

float rccheck;
float PIDcheck2;
float PIDcheck3;


static const PWMConfig pwmcfg = {1000000,
                                 10,
                                 NULL,
                                 {{PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL}},
                                 0,
                                 0};


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

    pwmStart(&PWMD3, &pwmcfg);
    pwmEnableChannel(&PWMD3, 0, 3);

    motorInit();

    // Initialize dbus
    // RCInit();


    while (true)
    {

        // 20000 = 1cm
        if(RCGet()->channel3 >= 600){
            setDistance(0,20000 * (13.2)); // the position of the second box
        }
        
        else if(RCGet()->channel3 <= -600){
            setDistance(0,20000 * (13.2 + 13.2)); // the position of the third box
        }

        else if(RCGet()->channel2 >= 600){
            setDistance(0,20000 * (13.2 + 13.2 + 12.2)); // the position of the fourth box
        } 

        else{
            setDistance(0,0); // the position of the first box
        }

        if(RCGet()->s1 == 2){
            setDistance(1,75000); // the grip position of the gripper
        }
        else if(RCGet()->s1 == 3){
            setDistance(1,23000); // the pull position of the gripper
        }
        else
        {
            setDistance(1,0); // the initial position of the gripper
        }
        


        chThdSleepMilliseconds(1);
    }
}
