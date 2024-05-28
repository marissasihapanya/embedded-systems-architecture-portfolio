/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#define DISPLAY(...) \
        { \
        size_t bytesWritten; \
        snprintf(output, 64, __VA_ARGS__); \
        UART2_write(uart, output, strlen(output), &bytesWritten); \
    }

char output[64];
int bytesToSend;
// Driver Handles - Global variables
UART2_Handle uart;
void initUART(void)
{
    UART2_Params uartParams;

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY("Initializing I2C Driver - ");


    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY("Failed\n\r");
        while (1);
    }
    DISPLAY("Passed\n\r");

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY("Is this %s? ", sensors[i].id);
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY("Found\n\r");
            found = true;
            break;
        }
        DISPLAY("No\n\r");
    }
    if(found)
    {
        DISPLAY("Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress);
    }
    else
    {
        DISPLAY("Temperature sensor not found, contact professor\n\r");
    }

}

int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY("Error reading temperature sensor (%d)\n\r", i2cTransaction.status);
        DISPLAY("Please power cycle your board by unplugging USB and plugging back in.\n\r");
    }
    return temperature;
}

bool leftPressed = false;
bool rightPressed = false;

int checkButtons(int setpoint) {
    if (leftPressed) {
        setpoint--; // Decrease setpoint by 1 when left button is pressed
        leftPressed = false; // Reset left button flag
    }
    if (rightPressed) {
        setpoint++; // Increase setpoint by 1 when right button is pressed
        rightPressed = false; // Reset right button flag
    }
    return setpoint;
}

void checkTemp(int temperature, int setpoint, int *heat) {
    if (temperature < setpoint) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        *heat = 1; // Too cold! Heat is on
    }
    else if (temperature >= setpoint) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        *heat = 0; // Just right! Heat is off
    }
}

Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

void initTimer(void) {

    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while(1) {}

    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    leftPressed = true;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    rightPressed = true;
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    initUART();
    initI2C();
    initTimer();
    int temperature = readTemp();
    int setpoint = temperature;
    int heat = 0;
    int seconds = 0;
    // Loop Forever
    // Add flags (like the timer flag) to the button handlers.
    unsigned long button_elapsedTime = 200000;
    unsigned long temp_elapsedTime = 500000;
    unsigned long server_elapsedTime = 10000000;
    const unsigned long timerPeriod = 1000000;
    while (1) {

        // Convert different-period tasks to C (Task Scheduler code)
        // Configure the timer period
        while (!TimerFlag) {} // Wait for timer period
        // Every 200ms check the button flags
        if (button_elapsedTime >= 200000) {
            setpoint = checkButtons(setpoint);
            button_elapsedTime = 0;
        }
        // Every 500ms read the temperature and update the LED
        if (temp_elapsedTime >= 500000) {
            checkTemp(temperature, setpoint, &heat);
            temp_elapsedTime = 0;
        }
        if (server_elapsedTime >= 1000000) {
            // Every 1000ms output the following to the UART
            temperature = readTemp();
            DISPLAY("<%02d, %02d, %d, %04d>\n\r", temperature, setpoint, heat, seconds);
        }
        TimerFlag = 0; // Lower flag raised by timer
        ++timer0;

        button_elapsedTime += timerPeriod;
        temp_elapsedTime += timerPeriod;
        server_elapsedTime += timerPeriod;

    }

    return (NULL);
}
