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

/*
 *  ======== MSP_EXP432E401Y.c ========
 *  This file is responsible for setting up the board specific items for the
 *  MSP_EXP432E401Y board.
 */
#include <stdint.h>
#include <stdlib.h>

#ifndef __MSP432E401Y__
#define __MSP432E401Y__
#endif
#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/interrupt.h>
#include <ti/devices/msp432e4/driverlib/pwm.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/udma.h>

#include <ti/drivers/Power.h>

#include "MSP_EXP432E401Y.h"

/*
 *  =============================== ADCBuf ===============================
 */

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufMSP432E4.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#define CONFIG_ADCBUF_COUNT 1

/*
 *  ======== adcbufMSP432E4Objects ========
 */
ADCBufMSP432E4_Object adcbufMSP432E4Objects[CONFIG_ADCBUF_COUNT];

/*
 *  ======== ADCBuf Channels ========
 */
ADCBufMSP432E4_Channels adcBuf0MSP432E4Channels[] = {
    /* CONFIG_ADCBUF0CHANNEL_0 */
    {
        .adcPin = ADCBufMSP432E4_PE_3_A0,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_1 */
    {
        .adcPin = ADCBufMSP432E4_PE_2_A1,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_2 */
    {
        .adcPin = ADCBufMSP432E4_PE_1_A2,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_3 */
    {
        .adcPin = ADCBufMSP432E4_PE_0_A3,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_4 */
    {
        .adcPin = ADCBufMSP432E4_PD_7_A4,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_5 */
    {
        .adcPin = ADCBufMSP432E4_PE_5_A8,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_6 */
    {
        .adcPin = ADCBufMSP432E4_PE_4_A9,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_7 */
    {
        .adcPin = ADCBufMSP432E4_PB_4_A10,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_8 */
    {
        .adcPin = ADCBufMSP432E4_PB_5_A11,
        .adcSequence = ADCBufMSP432E4_Seq_1,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_9 */
    {
        .adcPin = ADCBufMSP432E4_PK_0_A16,
        .adcSequence = ADCBufMSP432E4_Seq_1,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_10 */
    {
        .adcPin = ADCBufMSP432E4_PK_1_A17,
        .adcSequence = ADCBufMSP432E4_Seq_1,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
    /* CONFIG_ADCBUF0CHANNEL_11 */
    {
        .adcPin = ADCBufMSP432E4_PK_2_A18,
        .adcSequence = ADCBufMSP432E4_Seq_1,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
};

/*
 *  ======== ADCBuf Seqeunce Priorities ========
 */
static ADCBufMSP432E4_SequencePriorities seqPriorities0[ADCBufMSP432E4_SEQUENCER_COUNT] = {
    /* Sequencer 0 */
    ADCBufMSP432E4_Priority_0,
    /* Sequencer 1 */
    ADCBufMSP432E4_Priority_0,
    /* Sequencer 2 */
    ADCBufMSP432E4_Seq_Disable,
    /* Sequencer 3 */
    ADCBufMSP432E4_Seq_Disable,
};

/*
 *  ======== ADCBuf Seqeunce Trigger Sources ========
 */
static ADCBufMSP432E4_TriggerSource triggerSource0[ADCBufMSP432E4_SEQUENCER_COUNT] = {
    /* Sequencer 0 trigger source */
    ADCBufMSP432E4_TIMER_TRIGGER,
    /* Sequencer 1 trigger source */
    ADCBufMSP432E4_TIMER_TRIGGER,
    /* Sequencer 2 trigger source */
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER,
    /* Sequencer 3 trigger source */
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER,
};

/*
 *  ======== adcbufMSP432E4HWAttrs ========
 */
const ADCBufMSP432E4_HWAttrsV1 adcbufMSP432E4HWAttrs[CONFIG_ADCBUF_COUNT] = {
    /* CONFIG_ADCBUF0 */
    {
        .intPriority = (~0),
        .adcBase = ADC0_BASE,
        .channelSetting = adcBuf0MSP432E4Channels,
        .sequencePriority = seqPriorities0,
        .adcTriggerSource = triggerSource0,
        .modulePhase = ADCBufMSP432E4_Phase_Delay_0,
        .refSource = ADCBufMSP432E4_VREF_INTERNAL,
        .useDMA = 1,
        .adcTimerSource = TIMER1_BASE
    },
};

/*
 *  ======== ADCBuf_config ========
 */
const ADCBuf_Config ADCBuf_config[CONFIG_ADCBUF_COUNT] = {
    /* CONFIG_ADCBUF0 */
    {
        .fxnTablePtr = &ADCBufMSP432E4_fxnTable,
        .object = &adcbufMSP432E4Objects[CONFIG_ADCBUF0],
        .hwAttrs = &adcbufMSP432E4HWAttrs[CONFIG_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = CONFIG_ADCBUF_COUNT;


/*
 *  ============================= Display =============================
 */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#define MAXPRINTLEN 1024

DisplayUart_Object displayUartObject;

static char displayBuf[MAXPRINTLEN];

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx = MSP_EXP432E401Y_UART0,
    .baudRate = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf = displayBuf,
    .strBufLen = MAXPRINTLEN
};

#ifndef BOARD_DISPLAY_USE_UART_ANSI
#define BOARD_DISPLAY_USE_UART_ANSI 0
#endif

const Display_Config Display_config[] = {
    {
#  if (BOARD_DISPLAY_USE_UART_ANSI)
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
#  else /* Default to minimal UART with no cursor placement */
        .fxnTablePtr = &DisplayUartMin_fxnTable,
#  endif
        .object = &displayUartObject,
        .hwAttrs = &displayUartHWAttrs
    }
};

const uint_least8_t Display_count = sizeof(Display_config) /
                                    sizeof(Display_Config);

/*
 *  =============================== DMA ===============================
 */
#include <ti/drivers/dma/UDMAMSP432E4.h>

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[64];

/*
 *  ======== dmaErrorFxn ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorFxn(uintptr_t arg)
{
    int status = uDMAErrorStatusGet();
    uDMAErrorStatusClear();

    /* Suppress unused variable warning */
    (void)status;

    while(1)
    {
        ;
    }
}

UDMAMSP432E4_Object udmaMSP432E4Object;

const UDMAMSP432E4_HWAttrs udmaMSP432E4HWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMAMSP432E4_ErrorFxn)dmaErrorFxn,
    .intNum = INT_UDMAERR,
    .intPriority = (~0)
};

const UDMAMSP432E4_Config UDMAMSP432E4_config = {
    .object = &udmaMSP432E4Object,
    .hwAttrs = &udmaMSP432E4HWAttrs
};

/*
 *  =============================== General ===============================
 */
/*
 *  ======== MSP_EXP432E401Y_initGeneral ========
 */
void MSP_EXP432E401Y_initGeneral(void)
{
    Power_init();

    /* Grant the DMA access to all FLASH memory */
    FLASH_CTRL->PP |= FLASH_PP_DFA;

    /* Region start address - match FLASH start address */
    FLASH_CTRL->DMAST = 0x00000000;

    /*
     * Access to FLASH is granted to the DMA in 2KB regions.  The value
     * assigned to DMASZ is the amount of 2KB regions to which the DMA will
     * have access.  The value can be determined via the following:
     *     2 * (num_regions + 1) KB
     *
     * To grant full access to entire 1MB of FLASH:
     *     2 * (511 + 1) KB = 1024 KB (1 MB)
     */
    FLASH_CTRL->DMASZ = 511;
}

#ifdef ENABLE_EMAC
/*
 *  =============================== EMAC ===============================
 */
#include <ti/drivers/emac/EMACMSP432E4.h>

/*
 *  Required by the Networking Stack (NDK). This array must be NULL terminated.
 *  This can be removed if NDK is not used.
 *  Double curly braces are needed to avoid GCC bug #944572
 *  https://bugs.launchpad.net/gcc-linaro/+bug/944572
 */
NIMU_DEVICE_TABLE_ENTRY NIMUDeviceTable[2] = {
    {
        /* Default: use Ethernet driver */
        .init = EMACMSP432E4_NIMUInit
    },
    {NULL}
};

/*
 *  EMAC configuration structure
 *  Set user/company specific MAC octates. The following sets the address
 *  to ff-ff-ff-ff-ff-ff. Users need to change this to make the label on
 *  their boards.
 */
unsigned char macAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

const EMACMSP432E4_HWAttrs EMACMSP432E4_hwAttrs = {
    .baseAddr = EMAC0_BASE,
    .intNum = INT_EMAC0,
    .intPriority = (~0),
    .led0Pin = EMACMSP432E4_PF0_EN0LED0,
    .led1Pin = EMACMSP432E4_PF4_EN0LED1,
    .macAddress = macAddress
};
#endif

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432E401Y.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input pins */
    /* MSP_EXP432E401Y_USR_SW1 */
    GPIOMSP432E4_PJ0 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* MSP_EXP432E401Y_USR_SW2 */
    GPIOMSP432E4_PJ1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* MSP_EXP432E401Y_HOST_IRQ */
    GPIOMSP432E4_PM7 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_RISING,

    /* Output pins */
    /* MSP_EXP432E401Y_USR_D1 */
    GPIOMSP432E4_PN1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MSP_EXP432E401Y_USR_D2 */
    GPIOMSP432E4_PN0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MSP_EXP432E401Y_nHIB_pin */
    GPIOMSP432E4_PD4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MSP_EXP432E401Y_CS_pin */
    GPIOMSP432E4_PP5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_HIGH,
    /* MSP_EXP432E401Y_SDSPI_CS */
    GPIOMSP432E4_PC7 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_HIGH,

    /* MUX_IN_CTRL0 */
    GPIOMSP432E4_PL0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_IN_CTRL1 */
    GPIOMSP432E4_PL1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_IN_CTRL2 */
    GPIOMSP432E4_PL2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_IN_CTRL3 */
    GPIOMSP432E4_PL3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_IN_CTRL4 */
    GPIOMSP432E4_PL4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_IN_CTRL5 */
    GPIOMSP432E4_PL5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,

    /* MUX_IN_CTRL6 */
    GPIOMSP432E4_PF1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_IN_CTRL7 */
    GPIOMSP432E4_PF2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,

    /* MUX_OUT_CTRL0 */
    GPIOMSP432E4_PD0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_OUT_CTRL1 */
    GPIOMSP432E4_PD1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_OUT_CTRL2 */
    GPIOMSP432E4_PN2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
    /* MUX_OUT_CTRL3 */
    GPIOMSP432E4_PN3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH |
    GPIO_CFG_OUT_LOW,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432E401Y.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /* MSP_EXP432E401Y_USR_SW1 */
    NULL,  /* MSP_EXP432E401Y_USR_SW2 */
    NULL,  /* MSP_EXP432E401Y_HOST_IRQ */
};

/* The device-specific GPIO_config structure */
const GPIOMSP432E4_Config GPIOMSP432E4_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions) /
                         sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  =============================== I2C ===============================
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>

I2CMSP432E4_Object i2cMSP432E4Objects[MSP_EXP432E401Y_I2CCOUNT];

const I2CMSP432E4_HWAttrs i2cMSP432E4HWAttrs[MSP_EXP432E401Y_I2CCOUNT] = {
    {
        .baseAddr = I2C0_BASE,
        .intNum = INT_I2C0,
        .intPriority = (~0),
        .sclPin = I2CMSP432E4_PB2_I2C0SCL,
        .sdaPin = I2CMSP432E4_PB3_I2C0SDA
    },
};

const I2C_Config I2C_config[MSP_EXP432E401Y_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CMSP432E4_fxnTable,
        .object = &i2cMSP432E4Objects[MSP_EXP432E401Y_I2C0],
        .hwAttrs = &i2cMSP432E4HWAttrs[MSP_EXP432E401Y_I2C0]
    },
};

const uint_least8_t I2C_count = MSP_EXP432E401Y_I2CCOUNT;

/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSMSP432E4.h>

#define SECTORSIZE       (0x4000)
#define NVS_REGIONS_BASE (0xF8000)
#define REGIONSIZE       (SECTORSIZE * 2)

/*
 * Reserve flash sectors for NVS driver use
 * by placing an uninitialized byte array
 * at the desired flash address.
 */
#if defined(__TI_COMPILER_VERSION__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
#pragma LOCATION(flashBuf, NVS_REGIONS_BASE);
#pragma NOINIT(flashBuf);
static char flashBuf[REGIONSIZE];

#elif defined(__IAR_SYSTEMS_ICC__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
__no_init static char flashBuf[REGIONSIZE] @ NVS_REGIONS_BASE;

#elif defined(__GNUC__)

/*
 * Place the flash buffers in the .nvs section created in the gcc linker file.
 * The .nvs section enforces alignment on a sector boundary but may
 * be placed anywhere in flash memory.  If desired the .nvs section can be set
 * to a fixed address by changing the following in the gcc linker file:
 *
 * .nvs (FIXED_FLASH_ADDR) (NOLOAD) : AT (FIXED_FLASH_ADDR) {
 *      *(.nvs)
 * } > REGION_TEXT
 */
__attribute__ ((section (".nvs")))
static char flashBuf[REGIONSIZE];

#endif

NVSMSP432E4_Object nvsMSP432E4Objects[MSP_EXP432E401Y_NVSCOUNT];

const NVSMSP432E4_HWAttrs nvsMSP432E4HWAttrs[MSP_EXP432E401Y_NVSCOUNT] = {
    {
        .regionBase = (void *) flashBuf,
        .regionSize = REGIONSIZE,
    },
};

const NVS_Config NVS_config[MSP_EXP432E401Y_NVSCOUNT] = {
    {
        .fxnTablePtr = &NVSMSP432E4_fxnTable,
        .object = &nvsMSP432E4Objects[MSP_EXP432E401Y_NVSMSP432E40],
        .hwAttrs = &nvsMSP432E4HWAttrs[MSP_EXP432E401Y_NVSMSP432E40],
    },
};

const uint_least8_t NVS_count = MSP_EXP432E401Y_NVSCOUNT;

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/power/PowerMSP432E4.h>
const PowerMSP432E4_Config PowerMSP432E4_config = {
    .policyFxn = &PowerMSP432E4_sleepPolicy,
    .enablePolicy = true
};

/*
 *  =============================== PWM ===============================
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMMSP432E4.h>

PWMMSP432E4_Object pwmMSP432E4Objects[MSP_EXP432E401Y_PWMCOUNT];

const PWMMSP432E4_HWAttrs pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWMCOUNT] = {
    {
        .pwmBaseAddr = PWM0_BASE,
        .pwmOutput = PWM_OUT_0,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
        .pinConfig = PWMMSP432E4_PF0_M0PWM0
    }
};

const PWM_Config PWM_config[] = {
    {
        .fxnTablePtr = &PWMMSP432E4_fxnTable,
        .object = &pwmMSP432E4Objects[MSP_EXP432E401Y_PWM0],
        .hwAttrs = &pwmMSP432E4HWAttrs[MSP_EXP432E401Y_PWM0]
    },
};

const uint_least8_t PWM_count = MSP_EXP432E401Y_PWMCOUNT;

/*
 *  =============================== SDFatFS ===============================
 */
#include <ti/drivers/SD.h>
#include <ti/drivers/SDFatFS.h>

/*
 * Note: The SDFatFS driver provides interface functions to enable FatFs
 * but relies on the SD driver to communicate with SD cards.  Opening a
 * SDFatFs driver instance will internally try to open a SD driver instance
 * reusing the same index number (opening SDFatFs driver at index 0 will try to
 * open SD driver at index 0).  This requires that all SDFatFs driver instances
 * have an accompanying SD driver instance defined with the same index.  It is
 * acceptable to have more SD driver instances than SDFatFs driver instances
 * but the opposite is not supported & the SDFatFs will fail to open.
 */
SDFatFS_Object sdfatfsObjects[MSP_EXP432E401Y_SDFatFSCOUNT];

const SDFatFS_Config SDFatFS_config[MSP_EXP432E401Y_SDFatFSCOUNT] = {
    {
        .object = &sdfatfsObjects[MSP_EXP432E401Y_SDFatFS0]
    }
};

const uint_least8_t SDFatFS_count = MSP_EXP432E401Y_SDFatFSCOUNT;

/*
 *  =============================== SPI ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432E4DMA.h>

SPIMSP432E4DMA_Object spiMSP432E4DMAObjects[MSP_EXP432E401Y_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to
 * (~0) to satisfy the SDSPI driver requirement.
 */
const SPIMSP432E4DMA_HWAttrs spiMSP432E4DMAHWAttrs[MSP_EXP432E401Y_SPICOUNT] =
{
    {
        .baseAddr = SSI2_BASE,
        .intNum = INT_SSI2,
        .intPriority = (~0),
        .defaultTxBufValue = (~0),
        .rxDmaChannel = UDMA_CH12_SSI2RX,
        .txDmaChannel = UDMA_CH13_SSI2TX,
        .minDmaTransferSize = 10,
        .clkPinMask = SPIMSP432E4_PD3_SSI2CLK,
        .fssPinMask = SPIMSP432E4_PD2_SSI2FSS,
        .xdat0PinMask = SPIMSP432E4_PD1_SSI2XDAT0,
        .xdat1PinMask = SPIMSP432E4_PD0_SSI2XDAT1
    },
    {
        .baseAddr = SSI3_BASE,
        .intNum = INT_SSI3,
        .intPriority = (~0),
        .defaultTxBufValue = (~0),
        .minDmaTransferSize = 10,
        .rxDmaChannel = UDMA_CH14_SSI3RX,
        .txDmaChannel = UDMA_CH15_SSI3TX,
        .clkPinMask = SPIMSP432E4_PQ0_SSI3CLK,
        .fssPinMask = SPIMSP432E4_PQ1_SSI3FSS,
        .xdat0PinMask = SPIMSP432E4_PQ2_SSI3XDAT0,
        .xdat1PinMask = SPIMSP432E4_PQ3_SSI3XDAT1
    }
};

const SPI_Config SPI_config[MSP_EXP432E401Y_SPICOUNT] = {
    {
        .fxnTablePtr = &SPIMSP432E4DMA_fxnTable,
        .object = &spiMSP432E4DMAObjects[MSP_EXP432E401Y_SPI2],
        .hwAttrs = &spiMSP432E4DMAHWAttrs[MSP_EXP432E401Y_SPI2]
    },
    {
        .fxnTablePtr = &SPIMSP432E4DMA_fxnTable,
        .object = &spiMSP432E4DMAObjects[MSP_EXP432E401Y_SPI3],
        .hwAttrs = &spiMSP432E4DMAHWAttrs[MSP_EXP432E401Y_SPI3]
    },
};

const uint_least8_t SPI_count = MSP_EXP432E401Y_SPICOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432E4.h>

UARTMSP432E4_Object uartMSP432E4Objects[MSP_EXP432E401Y_UARTCOUNT];
unsigned char uartMSP432E4RingBuffer[MSP_EXP432E401Y_UARTCOUNT][32];

/* UART configuration structure */
const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs[MSP_EXP432E401Y_UARTCOUNT] = {
    {
        .baseAddr = UART0_BASE,
        .intNum = INT_UART0,
        .intPriority = (~0),
        .flowControl = UARTMSP432E4_FLOWCTRL_NONE,
        .ringBufPtr = uartMSP432E4RingBuffer[MSP_EXP432E401Y_UART0],
        .ringBufSize = sizeof(uartMSP432E4RingBuffer[MSP_EXP432E401Y_UART0]),
        .rxPin = UARTMSP432E4_PA0_U0RX,
        .txPin = UARTMSP432E4_PA1_U0TX,
        .ctsPin = UARTMSP432E4_PIN_UNASSIGNED,
        .rtsPin = UARTMSP432E4_PIN_UNASSIGNED
    }
};

const UART_Config UART_config[MSP_EXP432E401Y_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object = &uartMSP432E4Objects[MSP_EXP432E401Y_UART0],
        .hwAttrs = &uartMSP432E4HWAttrs[MSP_EXP432E401Y_UART0]
    }
};

const uint_least8_t UART_count = MSP_EXP432E401Y_UARTCOUNT;

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogMSP432E4.h>

WatchdogMSP432E4_Object watchdogMSP432E4Objects[MSP_EXP432E401Y_WATCHDOGCOUNT];

const WatchdogMSP432E4_HWAttrs watchdogMSP432E4HWAttrs[
    MSP_EXP432E401Y_WATCHDOGCOUNT] = {
    {
        .baseAddr = WATCHDOG0_BASE,
        .intNum = INT_WATCHDOG,
        .intPriority = (~0),
        .reloadValue = 80000000 // 1 second period at default CPU clock freq
    },
};

const Watchdog_Config Watchdog_config[MSP_EXP432E401Y_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogMSP432E4_fxnTable,
        .object = &watchdogMSP432E4Objects[MSP_EXP432E401Y_WATCHDOG0],
        .hwAttrs = &watchdogMSP432E4HWAttrs[MSP_EXP432E401Y_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = MSP_EXP432E401Y_WATCHDOGCOUNT;

/*
 *  =============================== WiFi ===============================
 *
 * This is the configuration structure for the WiFi module that will be used
 * as part of the SimpleLink SDK WiFi plugin. These are configured for SPI mode.
 * Any changes here will need to be configured on the CC31xx device as well
 */
#include <ti/drivers/net/wifi/porting/SIMPLELINKWIFI.h>

const SIMPLELINKWIFI_HWAttrsV1 wifiSimplelinkHWAttrs =
{
    .spiIndex = MSP_EXP432E401Y_SPI3,
    .hostIRQPin = MSP_EXP432E401Y_HOST_IRQ,
    .nHIBPin = MSP_EXP432E401Y_nHIB_pin,
    .csPin = MSP_EXP432E401Y_CS_pin,
    .maxDMASize = 1024,
    .spiBitRate = 3000000
};

const uint_least8_t WiFi_count = 1;

const WiFi_Config WiFi_config[1] =
{
    {
        .hwAttrs = &wifiSimplelinkHWAttrs,
    }
};
