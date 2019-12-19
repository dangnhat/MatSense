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
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/GPIO.h>

/* Display Header files */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/* Board Header file */
#include "Board.h"

extern Display_Handle display;

#define ADC1_NUMCHANNELS   (8)
#define ADC2_NUMCHANNELS   (4)

#define ADC1_BUFFERSIZE    (8)
#define ADC2_BUFFERSIZE    (4)

uint16_t sequencer0BufferOne[ADC1_BUFFERSIZE];
uint16_t sequencer0BufferTwo[ADC1_BUFFERSIZE];
uint16_t sequencer1BufferOne[ADC2_BUFFERSIZE];
uint16_t sequencer1BufferTwo[ADC2_BUFFERSIZE];
uint32_t buffersCompletedCounter = 0;
uint16_t outputBuffer[2][ADC1_BUFFERSIZE];

/* ADCBuf semaphore */
sem_t adcbufSem;

/*
 * This function is called whenever a buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 *
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel) {
    uint_fast16_t i;
    uint16_t *completedBuffer = (uint16_t *) completedADCBuffer;
    uint16_t buffer_size = 0, buffer_index = 0;

    switch (completedChannel) {
    case CONFIG_ADCBUF0CHANNEL_8:
    case CONFIG_ADCBUF0CHANNEL_9:
    case CONFIG_ADCBUF0CHANNEL_10:
    case CONFIG_ADCBUF0CHANNEL_11:
        buffer_size = ADC2_BUFFERSIZE;
        buffer_index = 1;
        break;
    default:
        buffer_size = ADC1_BUFFERSIZE;
        buffer_index = 0;
        break;
    }

    for (i = 0; i < buffer_size; i++)
    {
        outputBuffer[buffer_index][i] = completedBuffer[i];
    }
    /* post adcbuf semaphore */
    if (completedChannel == CONFIG_ADCBUF0CHANNEL_8) {
        sem_post(&adcbufSem);
    }
}

/*
 *  ======== mainThread ========
 */
void *adcReadingThread(void *arg0)
{
    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion continuousConversion1[ADC1_NUMCHANNELS];
    ADCBuf_Conversion continuousConversion2[ADC2_NUMCHANNELS];
    uint_fast16_t i, count;
    int32_t status;
    
    int32_t ref_r = 150, res, adc_val; // Reference = 150Kohm

    /* Set MUX input controls to 0x00;
     */
    GPIO_write(MUX_IN_CTRL0, 0);
    GPIO_write(MUX_IN_CTRL1, 0);
    GPIO_write(MUX_IN_CTRL2, 0);
    GPIO_write(MUX_IN_CTRL3, 0);
    GPIO_write(MUX_IN_CTRL4, 0);
    GPIO_write(MUX_IN_CTRL5, 0);
    GPIO_write(MUX_IN_CTRL6, 0);
    GPIO_write(MUX_IN_CTRL7, 0);

    /* Set MUX output controls to 0x00;
     */
    GPIO_write(MUX_OUT_CTRL0, 0);
    GPIO_write(MUX_OUT_CTRL1, 0);
    GPIO_write(MUX_OUT_CTRL2, 0);
    GPIO_write(MUX_OUT_CTRL3, 0);

    /* Call driver init functions */
    ADCBuf_init();

    status = sem_init(&adcbufSem, 0, 0);
    if (status != 0) {
        Display_printf(display, 0, 0, "Error creating adcbufSem\n");
        while(1);
    }

    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADCBuf_Params_init(&adcBufParams);
    adcBufParams.callbackFxn = adcBufCallback;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams.samplingFrequency = 1;
    adcBuf = ADCBuf_open(CONFIG_ADCBUF0, &adcBufParams);

    /* Configure the conversion struct for Sample Sequencer 0 */
    continuousConversion1[0].arg = NULL;
    continuousConversion1[0].adcChannel = CONFIG_ADCBUF0CHANNEL_0;
    continuousConversion1[0].sampleBuffer = sequencer0BufferOne;
    continuousConversion1[0].sampleBufferTwo = sequencer0BufferTwo;
    continuousConversion1[0].samplesRequestedCount = ADC1_BUFFERSIZE;

    continuousConversion1[1].adcChannel = CONFIG_ADCBUF0CHANNEL_1;
    continuousConversion1[2].adcChannel = CONFIG_ADCBUF0CHANNEL_2;
    continuousConversion1[3].adcChannel = CONFIG_ADCBUF0CHANNEL_3;
    continuousConversion1[4].adcChannel = CONFIG_ADCBUF0CHANNEL_4;
    continuousConversion1[5].adcChannel = CONFIG_ADCBUF0CHANNEL_5;
    continuousConversion1[6].adcChannel = CONFIG_ADCBUF0CHANNEL_6;
    continuousConversion1[7].adcChannel = CONFIG_ADCBUF0CHANNEL_7;

    for (count = 1; count < ADC1_NUMCHANNELS; count++) {
        continuousConversion1[count].arg = NULL;
        continuousConversion1[count].sampleBuffer = NULL;
        continuousConversion1[count].sampleBufferTwo = NULL;
        continuousConversion1[count].samplesRequestedCount = ADC1_BUFFERSIZE;
    }

    /* Configure the conversion struct for Sample Sequencer 1*/
    continuousConversion2[0].arg = NULL;
    continuousConversion2[0].adcChannel = CONFIG_ADCBUF0CHANNEL_8;
    continuousConversion2[0].sampleBuffer = sequencer1BufferOne;
    continuousConversion2[0].sampleBufferTwo = sequencer1BufferTwo;
    continuousConversion2[0].samplesRequestedCount = ADC2_BUFFERSIZE;

    continuousConversion2[1].adcChannel = CONFIG_ADCBUF0CHANNEL_9;
    continuousConversion2[2].adcChannel = CONFIG_ADCBUF0CHANNEL_10;
    continuousConversion2[3].adcChannel = CONFIG_ADCBUF0CHANNEL_11;

    for (count = 1; count < ADC2_NUMCHANNELS; count++) {
        continuousConversion2[count].arg = NULL;
        continuousConversion2[count].sampleBuffer = NULL;
        continuousConversion2[count].sampleBufferTwo = NULL;
        continuousConversion2[count].samplesRequestedCount = ADC2_BUFFERSIZE;
    }

    if (!adcBuf){
        /* AdcBuf did not open correctly. */
        while(1);
    }

    /* Start converting sequencer 0. */
    if (ADCBuf_convert(adcBuf, continuousConversion1, 8) !=
        ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        while(1);
    }

    /* Start converting sequencer 1. */
    if (ADCBuf_convert(adcBuf, continuousConversion2, 4) !=
        ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        while(1);
    }

    /*
     * Go to sleep in the foreground thread forever. The data will be collected
     * and transfered in the background thread
     */
    while(1) 
	{
        sem_wait(&adcbufSem);
        
        /*
         * Start with a header message and print current buffer values
         */

        Display_printf(display, 0, 0, "\r\nBuffer %u finished:",
                       (unsigned int)buffersCompletedCounter++);
        Display_printf(display, 0, 0, "\r\nCONFIG_ADCBUF0SEQ_0:");
        Display_printf(display, 0, 0, "\r\nADC Values, Resistor(kOhm):");
        for (i = 0; i < ADC1_BUFFERSIZE; i++)
        {
            adc_val = outputBuffer[0][i];
            res = (ref_r * 4096)/adc_val - ref_r;
            Display_printf(display, 0, 0, "    %u, %u", adc_val, res);
        }
        Display_printf(display, 0, 0, "\r\nCONFIG_ADCBUF0SEQ_1:");
        Display_printf(display, 0, 0, "\r\nADC Values, Resistor(kOhm):");
        for (i = 0; i < ADC2_BUFFERSIZE; i++)
        {
            adc_val = outputBuffer[1][i];
            res = (ref_r * 4096)/adc_val - ref_r;
            Display_printf(display, 0, 0, "    %u, %u", adc_val, res);
        }
        memset(outputBuffer, 0, sizeof(outputBuffer));
    }

}
