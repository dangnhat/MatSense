/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *  ======== adcBufContinuousSampling.c ========
 */


/* DriverLib Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>

/* Display Header files */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/* Board Header file */
#include "ti_drivers_config.h"

#include "BPLib.h"
/* Debug header */
#define IES_DEBUG_EN (1)
#include "ies_debug.h"

/* MUX CONFIG */
const int32_t ref_r = 150; // Reference resistor = 150kOhm.
const int32_t base_r = 0; // Base resistor value for substraction.
const int32_t adc_max_value = 3785; // 12 bits ADC.
/* --- */

/* Display Driver Handle */
Display_Handle displayHandle;

/* Mux ADC handle */
ADC_Handle   adc;

/*
 *  ======== Initialize a single ADC for MUX ========
 */
void initMuxADC(void) {
    ADC_Params   params;

    ADC_Params_init(&params);
    adc = ADC_open(MUX_ADC_0, &params);

    if (adc == NULL) {
        IES_DEBUG_STATE("initMuxADC: Error initializing ADC0\n");
        while (1);
    }

    IES_DEBUG_STATE("initMuxADC: MUX_ADC_0 is initialized\n");
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    Display_Params displayParams;
    int32_t res;
    uint16_t adc_val;
    uint8_t scaled_res;
    int_fast16_t ret;

    /* Init BTSPP */
    BPLib btspp;
    if (btspp.begin((char *) BP_MODE_SPP, (char *) BP_SPP_SPP) != 1) {
      IES_DEBUG_STATE("mainThread: Failed to initialize BTSPP. Aborting...");
      while (1)
        ;
    }


    /* Call driver init functions */
    GPIO_init();
    ADC_init();
    Display_init();

    /* Configure & open Display driver */
    Display_Params_init(&displayParams);
    displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
    displayHandle = Display_open(Display_Type_UART, &displayParams);
    if (displayHandle == NULL) {
        IES_DEBUG_STATE("Error creating displayHandle\n");
        while (1);
    }
    
    IES_DEBUG_STATE("Welcome to MatSense-BT v0.1\n");

    /* Start the ADC */
    initMuxADC();

    /* Init the initial values for control pins */
    GPIO_write(MUX_OUT_CTRL0, 0);
    GPIO_write(MUX_OUT_CTRL1, 0);
    GPIO_write(MUX_OUT_CTRL2, 0);
    GPIO_write(MUX_OUT_CTRL3, 0);
    GPIO_write(MUX_OUT_CTRL4, 0);
    GPIO_write(MUX_OUT_CTRL5, 0);
    GPIO_write(MUX_OUT_CTRL6, 0);
    GPIO_write(MUX_OUT_CTRL7, 0);

    GPIO_write(MUX_IN_CTRL0, 0);
    GPIO_write(MUX_IN_CTRL1, 0);
    GPIO_write(MUX_IN_CTRL2, 0);
    GPIO_write(MUX_IN_CTRL3, 0);
    GPIO_write(MUX_IN_CTRL4, 0);
    GPIO_write(MUX_IN_CTRL5, 0);
    GPIO_write(MUX_IN_CTRL6, 0);
    GPIO_write(MUX_IN_CTRL7, 0);


    while(1) 
    {
        /*
         * Set the control values.
         */
        GPIO_write(MUX_OUT_CTRL0, 0);
        GPIO_write(MUX_OUT_CTRL1, 0);
        GPIO_write(MUX_OUT_CTRL2, 0);
        GPIO_write(MUX_OUT_CTRL3, 0);
        GPIO_write(MUX_OUT_CTRL4, 0);
        GPIO_write(MUX_OUT_CTRL5, 0);
        GPIO_write(MUX_OUT_CTRL6, 0);
        GPIO_write(MUX_OUT_CTRL7, 0);

        GPIO_write(MUX_IN_CTRL0, 0);
        GPIO_write(MUX_IN_CTRL1, 0);
        GPIO_write(MUX_IN_CTRL2, 0);
        GPIO_write(MUX_IN_CTRL3, 0);
        GPIO_write(MUX_IN_CTRL4, 0);
        GPIO_write(MUX_IN_CTRL5, 0);
        GPIO_write(MUX_IN_CTRL6, 0);
        GPIO_write(MUX_IN_CTRL7, 0);

        /* Read ADC */
        ret = ADC_convert(adc, &adc_val);

        if (ret == ADC_STATUS_SUCCESS) {

            IES_DEBUG_STATE("\r\nADC Values, Resistor(kOhm):");

            res = (ref_r * adc_max_value) / adc_val - ref_r;
            scaled_res = (uint8_t) (res - base_r);
            btspp.sendByte(scaled_res);
            IES_DEBUG_STATE("    %u, %u", adc_val, res);
        }
        else {
            IES_DEBUG_STATE("ADC0 convert failed\n");
        }
    }

}
