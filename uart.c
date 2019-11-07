#include "uart.h"
#include "ch.h"
#include "hal.h"
#include <string.h>

static uint8_t rxbuf[DBUS_BUFFER_SIZE];
//static RC_control_t rcCtrl;
static uint8_t receivedMessage;
static thread_reference_t uart_dbus_thread_handler = NULL;
systime_t updateTime;

static void rxend(UARTDriver *uartp)
{
    (void)uartp;
    chSysLockFromISR();
    chThdResumeI(&uart_dbus_thread_handler, MSG_OK);
    chSysUnlockFromISR();
}
static UARTConfig uartcfg = {
    NULL, NULL, rxend, NULL, NULL, 100000, USART_CR1_PCE, 0, 0};


static void UART_Reset(void)
{
    receivedMessage = 0;
}

//process received data
static void processRxData(void)
{
    receivedMessage = rxbuf[0];
    clearRxBuffer();
}

void clearRxBuffer(void)
{
    memset(rxbuf, 0, sizeof(rxbuf));
}

static THD_WORKING_AREA(uart_dbus_thread_wa, 512);
static THD_FUNCTION(uart_dbus_thread, p)
{
    (void)p;
    chRegSetThreadName("uart dbus receiver");

    uartStart(UART_DRIVER, &uartcfg);
    msg_t rxmsg;
    systime_t timeout = TIME_MS2I(4U);

    while (!chThdShouldTerminateX())
    {
        uartStopReceive(UART_DRIVER);
        uartStartReceive(UART_DRIVER, DBUS_BUFFER_SIZE, rxbuf);
        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&uart_dbus_thread_handler, timeout);
        chSysUnlock();

        if (rxmsg == MSG_OK)
        {
            chSysLock();
            processRxData();
            chSysUnlock();
        }
        else
        {
            timeout = TIME_MS2I(4U);
        }
    }
}
//return uart data
uint8_t *UART_Get(void) { return &receivedMessage; }

void UART_Init(void)
{
    UART_Reset();
    chThdCreateStatic(uart_dbus_thread_wa,
                      sizeof(uart_dbus_thread_wa),
                      NORMALPRIO + 7,
                      uart_dbus_thread,
                      NULL);
}