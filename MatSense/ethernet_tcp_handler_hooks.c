/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

#include <mainThread.h>
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/slnetif/slnetifndk.h>
#include <ti/net/slnetif.h>

#include <ti/display/Display.h>

#include <pthread.h>

/*
 *  ======== netIPAddrHook ========
 *  User defined network IP address hook. Starts the EthernetTCPHandler thread.
 *
 */
void netIPAddrHook(uint32_t IPAddr,
                   unsigned int IfIdx,
                   unsigned int fAdd)
{
    static bool addInterface = true;
    int32_t status = 0;

    if(addInterface)
    {
        status = SlNetIf_add(SLNETIF_ID_2, "eth0",
                             (const SlNetIf_Config_t *)&SlNetIfConfigNDK, 4);
    }

    if(status != 0)
    {
        Display_printf(display, 0, 0, "SlNetIf_add fail (%d)\n",
                       status);
    }

    if(fAdd && addInterface)
    {
        addInterface = false;
    }
}

/*
 *  ======== netOpenHook ========
 *  NDK network open hook
 */
void netOpenHook()
{
}
