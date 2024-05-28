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

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>



enum States { DOT, DASH, GAP, BETWEEN_CHAR, BETWEEN_WORD };
enum MessageStates { SOS, OK };

unsigned char state = DOT;
unsigned char messageState = SOS;
bool messageComplete = false;
bool statePending = false;
static uint32_t tick = 0;

#define SOS_LENGTH 34
#define OK_LENGTH 30
unsigned char SOS_SEQUENCE[SOS_LENGTH] = {DOT, GAP, DOT, GAP, DOT, GAP, GAP, GAP, DASH, DASH, DASH, GAP, DASH, DASH, DASH,
GAP, DASH, DASH, DASH, GAP, GAP, GAP, DOT, GAP, DOT, GAP, DOT, GAP, GAP, GAP, GAP, GAP, GAP, GAP};
unsigned char OK_SEQUENCE[OK_LENGTH] = {DASH, DASH, DASH, GAP, DASH, DASH, DASH, GAP, DASH, DASH, DASH, GAP, GAP,
GAP, DASH, DASH, DASH, GAP, DOT, GAP, DASH, DASH, DASH, GAP, GAP, GAP, GAP, GAP, GAP, GAP};

void TickLED(unsigned char *state) {
    switch (*state) {
        case DOT: // red
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            break;
        case DASH: // green
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;
        case GAP:
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;
        default:
            break;
    }
}

void TickMessage(unsigned char *messageState) {
    static int index = 0;
    switch (*messageState) {
        case SOS:
            state = SOS_SEQUENCE[index]; // change state according to our message morse code sequence
            index = (index + 1) % SOS_LENGTH; // increment index until max length of our message
            messageComplete = false;
            break;
        case OK:
            state = OK_SEQUENCE[index];
            index = (index + 1) % OK_LENGTH;
            messageComplete = false;
            break;
        }
    if (index == 0) {
        messageComplete = true;
    } else {
        messageComplete = false;
    }
    // once state is pending (triggered by button) and message is complete, change the message
    if (statePending && messageComplete) {
        if (*messageState == SOS) {
            *messageState = OK;
        } else if (*messageState == OK) {
            *messageState = SOS;
        }
        statePending = false;
    }

}

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TickLED(&state);
    TickMessage(&messageState);

}

void initTimer(void) {
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
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
    statePending = true;
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0); // red


    // Detect the button press
    /* if state = SOS {
     * state = OK}
     *
     */
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
    /* Toggle an LED */
    GPIO_toggle(CONFIG_GPIO_LED_1);
}



/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    // Initialize timer
    initTimer();

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    //GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);



    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

        /*if (state == SOS) {
            state = OK;
        }
        else {
            state = SOS;
        }*/
    }

    return (NULL);
}
