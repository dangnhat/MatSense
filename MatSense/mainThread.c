/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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

/* POSIX Header files */
#include <mainThread.h>
#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* TI-Driver includes */
#include <ti/display/Display.h>

/* SlNetSock support */
#include <ti/net/slnetsock.h>
#include <ti/net/slnetif.h>

/* NDK support */
#include <ti/ndk/inc/socketndk.h>
#include <ti/ndk/inc/os/osif.h>

/* Application includes */

Display_Handle display;

extern void * adcReadingThread(void *arg0);
extern void ti_ndk_config_Global_startupFxn();
extern void ti_simplelink_host_config_Global_startupFxn();
extern void * TCPSocket(void *args);

/*
 *  ======== mainThread ========
 *  Thread that initializes the hardware and starts up the NDK and SimpleLink
 *  host stacks.
 */
void * mainThread(void *pvParameters)
{
    int retc;
    int status;

    GPIO_init();
    SPI_init();

    /* Configure & open Display driver */
    Display_Params displayParams;
    Display_init();

    Display_Params_init(&displayParams);
    displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
    display = Display_open(Display_Type_UART, &displayParams);
    if (display == NULL) {
        Display_printf(display, 0, 0, "Error creating displayHandle\n");
        while (1);
    }

    Display_printf(display, 0, 0, "Welcome to MatSense v0.1\n");

    status = SlNetSock_init(0);
    if(status != 0)
    {
        Display_printf(display, 0, 0, "SlNetSock_init fail (%d)\n",
                       status);
    }

    status = SlNetIf_init(0);
    if(status != 0)
    {
        Display_printf(display, 0, 0, "SlNetIf_init fail (%d)\n",
                       status);
    }

    pthread_t handler_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_handler;
    struct sched_param handlerParam;

    pthread_attr_init(&pAttrs_handler);
    handlerParam.sched_priority = 1;
    retc = pthread_attr_setschedparam(&pAttrs_handler, &handlerParam);
    retc |= pthread_attr_setstacksize(&pAttrs_handler, HANDLERSTACKSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs_handler,
                                        PTHREAD_CREATE_DETACHED);
    retc |=
        pthread_create(&handler_thread, &pAttrs_handler,
                       TCPSocket,
                       NULL);

    if(retc != 0)
    {
        /* netOpenHook: Failed to create tcpHandler thread */
        while(1)
        {
            ;
        }
    }

    ti_simplelink_host_config_Global_startupFxn();
    ti_ndk_config_Global_startupFxn();


    /* Create a ADC thread, Prio = 2 */
//    pthread_t adc_handler_thread = (pthread_t) NULL;
//    pthread_attr_t adc_pAttrs_handler;
//    struct sched_param adc_handlerParam;
//
//
//    pthread_attr_init(&adc_pAttrs_handler);
//    adc_handlerParam.sched_priority = 1;
//    retc = pthread_attr_setschedparam(&adc_pAttrs_handler, &adc_handlerParam);
//    retc |= pthread_attr_setstacksize(&adc_pAttrs_handler, ADCTHREADSTACKSIZE);
//    retc |= pthread_attr_setdetachstate(&adc_pAttrs_handler,
//                                        PTHREAD_CREATE_DETACHED);
//    retc |=
//        pthread_create(&adc_handler_thread, &adc_pAttrs_handler,
//                       adcReadingThread,
//                       NULL);
//
//    if(retc != 0)
//    {
//        /* Failed to create adcReadingThread thread */
//        while(1);
//    }

//    /* TODO: sleep for now */
//    while (1) {
//        sleep(1);
//    }

    return (0);
}
