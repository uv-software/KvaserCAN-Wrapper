/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  CAN Interface API, Version 3 (for Kvaser CAN Interfaces)
 *
 *  Copyright (c) 2017-2021 Uwe Vogt, UV Software, Berlin (info@uv-software.de)
 *  All rights reserved.
 *
 *  This file is part of KvaserCAN-Wrapper.
 *
 *  KvaserCAN-Wrapper is dual-licensed under the BSD 2-Clause "Simplified" License
 *  and under the GNU General Public License v3.0 (or any later version). You can
 *  choose between one of them if you use KvaserCAN-Wrapper in whole or in part.
 *
 *  BSD 2-Clause "Simplified" License:
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  KvaserCAN-Wrapper IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF KvaserCAN-Wrapper, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  GNU General Public License v3.0 or later:
 *  KvaserCAN-Wrapper is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  KvaserCAN-Wrapper is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with KvaserCAN-Wrapper.  If not, see <http://www.gnu.org/licenses/>.
 */
/** @addtogroup  can_api
 *  @{
 */
#include "build_no.h"
#ifdef _MSC_VER
#define VERSION_MAJOR    0
#define VERSION_MINOR    2
#define VERSION_PATCH    1
#else
#define VERSION_MAJOR    0
#define VERSION_MINOR    1
#define VERSION_PATCH    1
#endif
#define VERSION_BUILD    BUILD_NO
#define VERSION_STRING   TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH) " (" TOSTRING(BUILD_NO) ")"
#if defined(_WIN64)
#define PLATFORM        "x64"
#elif defined(_WIN32)
#define PLATFORM        "x86"
#elif defined(__linux__)
#define PLATFORM        "Linux"
#elif defined(__APPLE__)
#define PLATFORM        "macOS"
#else
#error Unsupported architecture
#endif
static const char version[] = "CAN API V3 for Kvaser CAN Interfaces, Version " VERSION_STRING;


/*  -----------  includes  -----------------------------------------------
 */

#ifdef _MSC_VER
//no Microsoft extensions please!
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif
#endif
#include "can_defs.h"
#include "can_api.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <windows.h>

#include "canlib.h"
#include "canstat.h"


/*  -----------  options  ------------------------------------------------
 */


/*  -----------  defines  ------------------------------------------------
 */

#define KVASER_FREQ_DEFAULT     (80000000)
#define KVASER_CHANNEL_DEFAULT  (0)

#ifndef OPTION_KVASER_CiA_BIT_TIMING
#define KVASER_BDR_1000(btr)    do{ btr.baud=canBITRATE_1M; } while(0)
#define KVASER_BDR_500(btr)     do{ btr.baud=canBITRATE_500K; } while(0)
#define KVASER_BDR_250(btr)     do{ btr.baud=canBITRATE_250K; } while(0)
#define KVASER_BDR_125(btr)     do{ btr.baud=canBITRATE_125K; } while(0)
#define KVASER_BDR_100(btr)     do{ btr.baud=canBITRATE_100K; } while(0)
#define KVASER_BDR_50(btr)      do{ btr.baud=canBITRATE_50K; } while(0)
#define KVASER_BDR_20(btr)      do{ btr.baud=20000; btr.tseg1=11; btr.tseg2=4; btr.sjw=1; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_10(btr)      do{ btr.baud=canBITRATE_10K; } while(0)
#else
#define KVASER_BDR_1000(btr)    do{ btr.baud=1000000; btr.tseg1=5 ; btr.tseg2=2; btr.sjw=1; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_800(btr)     do{ btr.baud=800000;  btr.tseg1=7 ; btr.tseg2=2; btr.sjw=1; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_500(btr)     do{ btr.baud=500000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=1; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_250(btr)     do{ btr.baud=250000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=1; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_125(btr)     do{ btr.baud=125000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=1; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_100(btr)     do{ btr.baud=100000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_50(btr)      do{ btr.baud=50000;   btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_20(btr)      do{ btr.baud=20000,   btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.sam=1; btr.sync=0; } while(0)
#define KVASER_BDR_10(btr)      do{ btr.baud=10000;   btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.sam=1; btr.sync=0; } while(0)
#endif
#ifndef KVASER_MAX_HANDLES
#define KVASER_MAX_HANDLES      (8)     // maximum number of open handles
#endif
#define INVALID_HANDLE          (-1)
#define IS_HANDLE_VALID(hnd)    ((0 <= (hnd)) && ((hnd) < KVASER_MAX_HANDLES))
#ifndef DLC2LEN
#define DLC2LEN(x)              dlc_table[(x) & 0xF]
#endif
#ifndef LEN2DLC
#define LEN2DLC(x)              ((x) > 48) ? 0xF : \
                                ((x) > 32) ? 0xE : \
                                ((x) > 24) ? 0xD : \
                                ((x) > 20) ? 0xC : \
                                ((x) > 16) ? 0xB : \
                                ((x) > 12) ? 0xA : \
                                ((x) > 8) ?  0x9 : (x)
#endif

/*  -----------  types  --------------------------------------------------
 */

typedef struct {                        // nominal bit-rate:
    long baud;                          //   bit-rate in [bps]
    unsigned int tseg1;                 //   time segement 1
    unsigned int tseg2;                 //   time segement 2
    unsigned int sjw;                   //   sync. jump width
    unsigned int sam;                   //   no. of samples
    unsigned int sync;                  //   unsupported
}   btr_nominal_t;

typedef struct {                        // data bit-rate:
    long baud;                          //   bit-rate in [bps]
    unsigned int tseg1;                 //   time segement 1
    unsigned int tseg2;                 //   time segement 2
    unsigned int sjw;                   //   sync. jump width
}   btr_data_t;

typedef struct {                        // frame counters:
    uint64_t tx;                        //   number of transmitted CAN frames
    uint64_t rx;                        //   number of received CAN frames
    uint64_t err;                       //   number of receiced error frames
}   can_counter_t;

typedef struct {                        // Kvaser CAN interface:
    CanHandle  handle;                  //   hardware channel handle
    int        channel;                 //   channel number of the CAN board
    long       frequency;               //   frequency of the CAN controller
    can_mode_t mode;                    //   operation mode of the CAN channel
    can_status_t status;                //   8-bit status register
    can_counter_t counters;             //   statistical counters
}   can_interface_t;


/*  -----------  prototypes  ---------------------------------------------
 */

static int kvaser_error(canStatus);    // Kvaser specific errors
static canStatus kvaser_capability(int channel, can_mode_t *capability);

static int index2params(int index, btr_nominal_t *params);
static int bitrate2params(const can_bitrate_t *bitrate, btr_nominal_t *params);
static int params2bitrate(const btr_nominal_t *params, long frequency, can_bitrate_t *bitrate);
static int bitrate2paramsFd(const can_bitrate_t *bitrate, btr_data_t *params);
static int paramsFd2bitrate(const btr_data_t *params, long frequency, can_bitrate_t *bitrate);

static int lib_parameter(uint16_t param, void *value, size_t nbyte);
static int drv_parameter(int handle, uint16_t param, void *value, size_t nbyte);

static int calc_speed(can_bitrate_t *bitrate, can_speed_t *speed, int modify);


/*  -----------  variables  ----------------------------------------------
 */

can_board_t can_boards[KVASER_BOARDS+1]=// list of CAN Interface boards:
{
    {KVASER_CAN_CHANNEL0,                 "Kvaser CAN Channel 0"},
    {KVASER_CAN_CHANNEL1,                 "Kvaser CAN Channel 1"},
    {KVASER_CAN_CHANNEL2,                 "Kvaser CAN Channel 2"},
    {KVASER_CAN_CHANNEL3,                 "Kvaser CAN Channel 3"},
    {KVASER_CAN_CHANNEL4,                 "Kvaser CAN Channel 4"},
    {KVASER_CAN_CHANNEL5,                 "Kvaser CAN Channel 5"},
    {KVASER_CAN_CHANNEL6,                 "Kvaser CAN Channel 6"},
    {KVASER_CAN_CHANNEL7,                 "Kvaser CAN Channel 7"},
    {EOF, NULL}
};
static const uint8_t dlc_table[16] = {  // DLC to length
    0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
};
static can_interface_t can[KVASER_MAX_HANDLES]; // interface handles
static int init = 0;                    // initialization flag


/*  -----------  functions  ----------------------------------------------
 */

int can_test(int32_t board, uint8_t mode, const void *param, int *result)
{
    int feature = 0x0000;               // channel capability
    int flags = 0x0000;                 // channel flags
    can_mode_t capa;                    // channel capability
    canStatus rc;                       // return value
    int i, n;

    if (!init) {                        // when not init before:
        for (i = 0; i < KVASER_MAX_HANDLES; i++) {
            can[i].handle = canINVALID_HANDLE;
            can[i].channel = KVASER_CHANNEL_DEFAULT;
            can[i].frequency = KVASER_FREQ_DEFAULT;
            can[i].mode.byte = CANMODE_DEFAULT;
            can[i].status.byte = CANSTAT_RESET;
            can[i].counters.tx = 0ull;
            can[i].counters.rx = 0ull;
            can[i].counters.err = 0ull;
        }
        canInitializeLibrary();         //   initialize the driver
        init = 1;                       //   set initialization flag
    }
    if ((rc = canGetNumberOfChannels(&n)) != canOK)
        return kvaser_error(rc);
    if (board >= n) {
        if (result)                     // CAN board not present
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
    if ((rc = canGetChannelData(board, canCHANNELDATA_CHANNEL_CAP,
                               (void*)&feature, sizeof(feature))) != canOK)
        return kvaser_error(rc);
#ifndef KVASER_VIRTUAL_CHANNELS
    if ((feature & canCHANNEL_CAP_VIRTUAL)) {
        if (result)                     // declare as not available
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
#endif
#ifndef KVASER_SIMULATED_CHANNELS
    if ((feature & canCHANNEL_CAP_SIMULATED)) {
        if (result)                     // declare as not available
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
#endif
    if ((rc = canGetChannelData(board, canCHANNELDATA_CHANNEL_FLAGS,
                               (void*)&flags, sizeof(flags))) != canOK)
        return kvaser_error(rc);
    if (result) {
#ifndef KVASER_SHARED_ACCESS
        if (flags & canCHANNEL_IS_OPEN)
            *result = CANBRD_OCCUPIED;  // CAN board occupied by another process
        else
            *result = CANBRD_PRESENT;   // CAN board present and available
#else
        if (flags & canCHANNEL_IS_EXCLUSIVE)
            *result = CANBRD_OCCUPIED;  // CAN board occupied by another process
        else if ((flags & canCHANNEL_IS_OPEN) && !(mode & CANMODE_SHRD))
            *result = CANBRD_OCCUPIED;  // CAN board occupied by another process
        else
            *result = CANBRD_PRESENT;   // CAN board present and available
#endif
        for (i = 0; i < KVASER_MAX_HANDLES; i++)
            if ((can[i].handle != canINVALID_HANDLE) &&
               (can[i].channel == board) && !(mode & CANMODE_SHRD))
                *result = CANBRD_OCCUPIED; // CAN board occupied by ourself
    }
#if (0)
    if ((mode & CANMODE_FDOE) && !(feature & canCHANNEL_CAP_CAN_FD))
        return CANERR_ILLPARA; // CAN FD operation requested, but not supported
    if ((mode & CANMODE_BRSE) && !(mode & CANMODE_FDOE))
        return CANERR_ILLPARA; // bit-rate switching requested, but CAN FD not enabled
    if ((mode & CANMODE_NISO) && !(feature & canCHANNEL_CAP_CAN_FD_NONISO))
        return CANERR_ILLPARA; // Non-ISO operation requested, but not supported
    if ((mode & CANMODE_NISO) && !(mode & CANMODE_FDOE))
        return CANERR_ILLPARA; // Non-ISO operation requested, but CAN FD not enabled
    if ((mode & CANMODE_MON) && !(feature & canCHANNEL_CAP_SILENT_MODE))
        return CANERR_ILLPARA; // listen-only mode requested, but not supported
    /*if ((mode & CANMODE_ERR)) {} // error frame reporting can be turned on and off by ioctrl */
    if ((mode & CANMODE_NXTD))  // TODO: how?
        return CANERR_ILLPARA; // suppressing 29-bit id's is not supported
    if ((mode & CANMODE_NRTR))  // TODO: how? + fdoe
        return CANERR_ILLPARA; // suppressing remote frames is not supported
#else
    // get operation capability from CAN board
    if ((rc = kvaser_capability(board, &capa)) != CANERR_NOERROR)
        return kvaser_error(rc);
    // check given operation mode against the operation capability
    if ((mode & ~capa.byte) != 0)
        return CANERR_ILLPARA;
    if ((mode & CANMODE_BRSE) && !(mode & CANMODE_FDOE))
        return CANERR_ILLPARA;
#endif
    (void)param;
    return CANERR_NOERROR;
}

int can_init(int32_t board, uint8_t mode, const void *param)
{
    can_mode_t capa;                    // channel capability
    int flags = 0x0000;                 // flags for canOpenChannel()
    canHandle result;                   // Kvaser handle or error
    canStatus rc;                       // return value
    int i;

    if (!init) {                        // when not init before:
        for (i = 0; i < KVASER_MAX_HANDLES; i++) {
            can[i].handle = canINVALID_HANDLE;
            can[i].channel = KVASER_CHANNEL_DEFAULT;
            can[i].frequency = KVASER_FREQ_DEFAULT;
            can[i].mode.byte = CANMODE_DEFAULT;
            can[i].status.byte = CANSTAT_RESET;
            can[i].counters.tx = 0ull;
            can[i].counters.rx = 0ull;
            can[i].counters.err = 0ull;
        }
        canInitializeLibrary();         // initialize the driver
        init = 1;                       // set initialization flag
    }
    for (i = 0; i < KVASER_MAX_HANDLES; i++) {
        if ((can[i].handle != canINVALID_HANDLE) &&
           (can[i].channel == board))   // channel already in use
            return CANERR_YETINIT;
    }
    for (i = 0; i < KVASER_MAX_HANDLES; i++) {
        if (can[i].handle == canINVALID_HANDLE)
            break;
    }
    if (!IS_HANDLE_VALID(i))            // no free handle found
        return CANERR_HANDLE;

    /* get operation capabilit from channel check with given operation mode */
    if ((rc = kvaser_capability(board, &capa)) != CANERR_NOERROR)
        return kvaser_error(rc);
    if ((mode & ~capa.byte) != 0)
        return CANERR_ILLPARA;
    if ((mode & CANMODE_BRSE) && !(mode & CANMODE_FDOE))
        return CANERR_ILLPARA;
#ifndef KVASER_SHARED_ACCESS
    flags |= canOPEN_EXCLUSIVE;
#else
    flags |= (mode & CANMODE_SHRD)? 0 : canOPEN_EXCLUSIVE;
#endif
    flags |= (mode & CANMODE_FDOE)? canOPEN_CAN_FD : 0;
    flags |= (mode & CANMODE_FDOE)? canOPEN_ACCEPT_LARGE_DLC : 0;// why?
    flags |= (mode & CANMODE_NISO)? canOPEN_CAN_FD_NONISO : 0;
#ifdef KVASER_VIRTUAL_CHANNELS
    // TODO: flags |= canOPEN_ACCEPT_VIRTUAL;
#endif
    if ((result = canOpenChannel(board, flags)) < canOK)
        return kvaser_error(result);

    if (!(mode & CANMODE_MON)) {        // normal operation
        if ((rc = canSetBusOutputControl(result, canDRIVER_NORMAL)) != canOK)
            return kvaser_error(rc);
    }
    else {                              // listen-only
        if ((rc = canSetBusOutputControl(result, canDRIVER_SILENT)) != canOK)
            return kvaser_error(rc);
    }
    can[i].handle = result;             // handle of the CAN board/channel
    can[i].channel = board;             // channel number of the CAN board
    can[i].mode.byte = mode;            // store selected operation mode
    can[i].status.byte = CANSTAT_RESET; // CAN controller not started yet

    return i;                           // return the handle
}

int can_exit(int handle)
{
    canStatus rc;                       // return value
    int i;

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (handle != CANEXIT_ALL) {
        if (!IS_HANDLE_VALID(handle))   // must be a valid handle
            return CANERR_HANDLE;
        if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
            return CANERR_HANDLE;
        /*if (!can[handle].status.can_stopped) // go to CAN INIT mode (bus off)*/
            (void)canBusOff(can[handle].handle);
        if ((rc = canClose(can[handle].handle)) != canOK) // release the CAN interface!
            return kvaser_error(rc);

        can[handle].status.byte |= CANSTAT_RESET;// CAN controller in INIT state
        can[handle].handle = canINVALID_HANDLE; // handle can be used again
    }
    else {
        for (i = 0; i < KVASER_BOARDS; i++) {
            if (can[i].handle != canINVALID_HANDLE) // must be an opened handle
            {
                /*if (!can[handle].status.can_stopped) // go to CAN INIT mode (bus off)*/
                   (void)canBusOff(can[i].handle);
                (void)canClose(can[i].handle);     // resistance is futile!

                can[i].status.byte |= CANSTAT_RESET;// CAN controller in INIT state
                can[i].handle = canINVALID_HANDLE; // handle can be used again
            }
        }
    }
    return CANERR_NOERROR;
}

int can_start(int handle, const can_bitrate_t *bitrate)
{
    unsigned char error_frames;         // error frame reporting
    btr_nominal_t nominal;              // nominal bit-rate
    btr_data_t data;                    // data bit-rate
    canStatus rc;                       // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;
    if (bitrate == NULL)                // check for null-pointer
        return CANERR_NULLPTR;
    if (!can[handle].status.can_stopped) // must be stopped
        return CANERR_ONLINE;

    memset(&nominal, 0, sizeof(btr_nominal_t));
    memset(&data, 0, sizeof(btr_data_t));

    if (bitrate->index <= 0) {          // bit-rate from index
        if ((rc = index2params(bitrate->index, &nominal)) != CANERR_NOERROR)
            return rc;
        if ((rc = canSetBusParams(can[handle].handle, nominal.baud, nominal.tseg1, nominal.tseg2,
                                                     nominal.sjw, nominal.sam, nominal.sync)) != canOK)
            return kvaser_error(rc);
#ifndef OPTION_KVASER_CiA_BIT_TIMING
        can[handle].frequency = KVASER_FREQ_DEFAULT;
#else
        can[handle].frequency = CANBTR_FREQ_SJA1000;
#endif
    }
    else {                              // bit-rate from parameter
        if ((rc = bitrate2params(bitrate, &nominal)) != CANERR_NOERROR)
            return rc;
        if ((rc = canSetBusParams(can[handle].handle, nominal.baud, nominal.tseg1, nominal.tseg2,
                                                     nominal.sjw, nominal.sam, nominal.sync)) != canOK)
            return kvaser_error(rc);
        /* bit-rate for CAN FD data (BRSE) */
        if (can[handle].mode.fdoe && can[handle].mode.brse) {
            if ((rc = bitrate2paramsFd(bitrate, &data)) != CANERR_NOERROR)
                return rc;
            if ((rc = canSetBusParamsFd(can[handle].handle, data.baud, data.tseg1, data.tseg2, data.sjw)) != canOK)
                return kvaser_error(rc);
        }
        can[handle].frequency = bitrate->btr.frequency;
    }
    error_frames = can[handle].mode.err ? 1 : 0; // error frames
    if ((rc = canIoCtl(can[handle].handle, canIOCTL_SET_ERROR_FRAMES_REPORTING,
                     (void*)&error_frames, sizeof(error_frames))) != canOK)
        return kvaser_error(rc);
    if ((rc = canBusOn(can[handle].handle)) != canOK) // go bus on!
        return kvaser_error(rc);

    can[handle].status.byte = 0x00;     // clear old status bits and counters
    can[handle].counters.tx = 0ull;
    can[handle].counters.rx = 0ull;
    can[handle].counters.err = 0ull;
    can[handle].status.can_stopped = 0;  // CAN controller started!

    return CANERR_NOERROR;
}

int can_kill(int handle)
{
    HANDLE hEvent = NULL;               // event object
    int i;

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (handle != CANKILL_ALL) {
        if (!IS_HANDLE_VALID(handle))   // must be a valid handle
            return CANERR_HANDLE;
        if (can[handle].handle != canINVALID_HANDLE) {
            if ((canIoCtl(can[handle].handle, canIOCTL_GET_EVENTHANDLE,
                         (void*)&hEvent, sizeof(hEvent)) == canOK) &&
               (hEvent != NULL)) {
                (void)SetEvent(hEvent); // signal the event object
            }
        }
    }
    else {
        for (i = 0; i < KVASER_MAX_HANDLES; i++) {
            if (can[i].handle != canINVALID_HANDLE) {
                if ((canIoCtl(can[i].handle, canIOCTL_GET_EVENTHANDLE,
                    (void*)&hEvent, sizeof(hEvent)) == canOK) &&
                   (hEvent != NULL)) {
                    (void)SetEvent(hEvent); // signal all event objects
                }
            }
        }
    }
    return CANERR_NOERROR;
}

int can_reset(int handle)
{
    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if (can[handle].status.can_stopped) {  // CAN started, then reset
        (void)canBusOff(can[handle].handle);
    }
    can[handle].status.can_stopped = 1;   // CAN controller stopped!

    return CANERR_NOERROR;
}

int can_write(int handle, const can_msg_t *msg, uint16_t timeout)
{
    long id;                            // the message:
    unsigned int dlc, flags;
    unsigned char data[CANFD_MAX_LEN];
    canStatus rc;                       // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;
    if (msg == NULL)                    // check for null-pointer
        return CANERR_NULLPTR;
    if (can[handle].status.can_stopped) // must be running
        return CANERR_OFFLINE;

    if (msg->id > (uint32_t)(msg->xtd ? CAN_MAX_XTD_ID : CAN_MAX_STD_ID))
        return CANERR_ILLPARA;          // invalid identifier
    if (msg->xtd && can[handle].mode.nxtd)
        return CANERR_ILLPARA;          // suppress extended frames
    if (msg->rtr && can[handle].mode.nrtr)
        return CANERR_ILLPARA;          // suppress remote frames
    if (msg->fdf && !can[handle].mode.fdoe)
        return CANERR_ILLPARA;          // long frames only with CAN FD
    if (msg->brs && !can[handle].mode.brse)
        return CANERR_ILLPARA;          // fast frames only with CAN FD
    if (msg->brs && !msg->fdf)
        return CANERR_ILLPARA;          // bit-rate switching only with CAN FD
    if (msg->sts)
        return CANERR_ILLPARA;          // error frames cannot be sent

    if (!can[handle].mode.fdoe) {
        if (msg->dlc > CAN_MAX_LEN)     //   data length 0 .. 8!
            return CANERR_ILLPARA;
        if (msg->xtd)                   //   29-bit identifier
            flags = canMSG_EXT;
        else                            //   11-bit identifier
            flags = canMSG_STD;
        if (msg->rtr)                   //   request a message
            flags |= canMSG_RTR;
        id = (long)(msg->id);
        dlc = (unsigned int)(msg->dlc);
        memcpy(data, msg->data, msg->dlc);
    }
    else {
        if (msg->dlc > CANFD_MAX_DLC)   //   data length 0 .. 0Fh!
            return CANERR_ILLPARA;
        if (msg->xtd)                   //   29-bit identifier
            flags = canMSG_EXT;
        else                            //   11-bit identifier
            flags = canMSG_STD;
        if (msg->rtr)                   //   request a message
            flags |= canMSG_RTR;
        if (msg->fdf)                   //   CAN FD format
            flags |= canFDMSG_FDF;
        if (msg->brs && can[handle].mode.brse)  // bit-rate switching
            flags |= canFDMSG_BRS;
        id = (long)(msg->id);
        dlc = (unsigned int)(DLC2LEN(msg->dlc)); // why? acc. to docu is dlc DLC, not length
        memcpy(data, msg->data, DLC2LEN(msg->dlc));
    }
#ifdef KVASER_ASYNCHRONOUS_WRITE
    if ((rc = canWrite(can[handle].handle, id, &data, dlc, flags)) != canOK)
#else
    if ((rc = canWriteWait(can[handle].handle, id, &data, dlc, flags, KVASER_TRM_TIMEOUT)) != canOK)
#endif
    {
        if ((rc & canERR_TXBUFOFL)) {   //   transmit queue full?
            can[handle].status.transmitter_busy = 1;
            return CANERR_TX_BUSY;      //     transmitter busy
        }
        else if ((rc & canERR_TIMEOUT)) {//   timed out?
            can[handle].status.transmitter_busy = 1;
            return CANERR_TX_BUSY;      //     transmitter busy
        }
        return kvaser_error(rc);        //   Kvaser specific error?
    }
    can[handle].status.transmitter_busy = 0; // message transmitted
    can[handle].counters.tx++;

    return CANERR_NOERROR;
}

int can_read(int handle, can_msg_t *msg, uint16_t timeout)
{
    long id;                            // the message:
    unsigned int len, flags;            //   length and flags
    unsigned char data[CANFD_MAX_LEN];  //   data payload
    unsigned long timestamp;            // time stamp (in [ms])
    canStatus rc;                       // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;
    if (msg == NULL)                    // check for null-pointer
        return CANERR_NULLPTR;
    if (can[handle].status.can_stopped) // must be running
        return CANERR_OFFLINE;
repeat:
    if ((rc = canRead(can[handle].handle, &id, data, &len, &flags, &timestamp)) == canERR_NOMSG) {
        if (timeout > 0) {
            switch (canWaitForEvent(can[handle].handle, (timeout != CANREAD_INFINITE) ? timeout : INFINITE)) {
            case canOK:
                break;                  //   one or more messages received
            case canERR_TIMEOUT:
                break;                  //   time-out, but look for old messages
            default:
                return CANERR_FATAL;    //   function failed!
            }
            if ((rc = canRead(can[handle].handle, &id, data, &len, &flags, &timestamp)) == canERR_NOMSG) {
                can[handle].status.receiver_empty = 1;
                return CANERR_RX_EMPTY; //   receiver empty
            }
        }
        else {
            can[handle].status.receiver_empty = 1;
            return CANERR_RX_EMPTY;     //   receiver empty
        }
    }
    if (rc < canOK)                     // receive error?
    {
        return kvaser_error(rc);        //   something's wrong
    }
    if ((flags & canMSG_ERROR_FRAME)) { // error frame?
        can[handle].status.receiver_empty = 1;
            can[handle].counters.err++;
            return CANERR_ERR_FRAME;    //   error frame received
    }
    if ((flags & canMSG_EXT) && can[handle].mode.nxtd)
        goto repeat;                    // refuse extended frames
    if ((flags & canMSG_RTR) && can[handle].mode.nrtr)
        goto repeat;                    // refuse remote frames
    msg->id = (int32_t)id;
    msg->xtd = (flags & canMSG_EXT)? 1 : 0;
    msg->rtr = (flags & canMSG_RTR)? 1 : 0;
    msg->fdf = (flags & canFDMSG_FDF)? 1 : 0;
    msg->brs = (flags & canFDMSG_BRS)? 1 : 0;
    msg->esi = (flags & canFDMSG_ESI)? 1 : 0;
    msg->dlc = (uint8_t)LEN2DLC(len); // unclear: is it a length or a DLC?
#ifndef CAN_20_ONLY
    memcpy(msg->data, data, CANFD_MAX_LEN);
#else
    memcpy(msg->data, data, CAN_MAX_LEN);
#endif
    msg->timestamp.tv_sec = (time_t)(timestamp / 1000ul);
    msg->timestamp.tv_nsec = (long)(timestamp % 1000ul) * 1000000l;
    can[handle].status.receiver_empty = 0; // message read
    can[handle].counters.rx++;

    return CANERR_NOERROR;
}

int can_status(int handle, uint8_t *status)
{
    unsigned long flags;                // status flags
    canStatus rc;                       // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if ((rc = canRequestChipStatus(can[handle].handle)) != canOK)
        return kvaser_error(rc);
    if ((rc = canReadStatus(can[handle].handle, &flags)) != canOK)
        return kvaser_error(rc);

    can[handle].status.bus_off = (flags & canSTAT_BUS_OFF)? 1 : 0;
    can[handle].status.bus_error = (flags & canSTAT_ERROR_PASSIVE)? 1 : 0;
    can[handle].status.warning_level = (flags & canSTAT_ERROR_WARNING)? 1 : 0;
    can[handle].status.message_lost |= (flags & canSTAT_RXERR)? 1 : 0;
    can[handle].status.transmitter_busy |= (flags & canSTAT_TX_PENDING)? 1 : 0;
    if (status)                         // status-register
      *status = can[handle].status.byte;

    return CANERR_NOERROR;
}

int can_busload(int handle, uint8_t *load, uint8_t *status)
{
    float busload = 0.0;                // bus-load (in [percent])

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if (!can[handle].status.can_stopped) { // when running get bus load
        (void)busload; //  TODO: measure bus load
    }
    if (load)                           // bus-load (in [percent])
        *load = (uint8_t)busload;
     return can_status(handle, status); // status-register
}

int can_bitrate(int handle, can_bitrate_t *bitrate, can_speed_t *speed)
{
    can_bitrate_t temporary;            // bit-rate settings
    btr_nominal_t nominal;              // nominal bit-rate
    btr_data_t data;                    // data bit-rate
    int rc;                             // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    memset(&temporary, 0, sizeof(can_bitrate_t));
    memset(&nominal, 0, sizeof(btr_nominal_t));
    memset(&data, 0, sizeof(btr_data_t));

    /* bit-rate for CAN 2.0 & CAN FD */
    if ((rc = canGetBusParams(can[handle].handle, &nominal.baud, &nominal.tseg1, &nominal.tseg2,
                                                 &nominal.sjw, &nominal.sam, &nominal.sync)) != canOK)
        return kvaser_error(rc);
    if ((rc = params2bitrate(&nominal, can[handle].frequency, &temporary)) != CANERR_NOERROR)
        return rc;
    /* bit-rate for CAN FD data (BRSE) */
    if (can[handle].mode.fdoe && can[handle].mode.brse) {
        if ((rc = canGetBusParamsFd(can[handle].handle, &data.baud, &data.tseg1, &data.tseg2, &data.sjw)) != canOK)
            return kvaser_error(rc);
        if ((rc = paramsFd2bitrate(&data, can[handle].frequency, &temporary)) != CANERR_NOERROR)
            return rc;
    }
    if (bitrate) {
        memcpy(bitrate, &temporary, sizeof(can_bitrate_t));
    }
    if (speed) {
        if ((rc = calc_speed(&temporary, speed, 0)) != CANERR_NOERROR)
            return rc;
        speed->nominal.fdoe = can[handle].mode.fdoe;
        speed->data.brse = can[handle].mode.brse;
    }
    if (!can[handle].status.can_stopped)
        rc = CANERR_NOERROR;
    else
        rc = CANERR_OFFLINE;
    return rc;
}

int can_property(int handle, uint16_t param, void *value, uint32_t nbyte)
{
    if (!init || !IS_HANDLE_VALID(handle)) {
        return lib_parameter(param, value, (size_t)nbyte);
    }
    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    return drv_parameter(handle, param, value, (size_t)nbyte);
}

char *can_hardware(int handle)
{
    static char hardware[256] = "";     // hardware version

    if (!init)                          // must be initialized
        return NULL;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return NULL;
    if (can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return NULL;

    if (canGetChannelData(can[handle].channel, canCHANNELDATA_CHANNEL_NAME, hardware, 255) != canOK)
        return NULL;

    return (char*)hardware;             // hardware version
}

char *can_software(int handle)
{
    static char software[256] = "";     // software version
    unsigned short version;             // version number

    if (!init)                          // must be initialized
        return NULL;
    (void)handle;                       // handle not needed here

    version = canGetVersion();          // FIXME: check encoding
    snprintf(software, 256, "Kvaser CANlib SDK V%d.%d (canlib32.dll)", (version >> 8), (version & 0xFF));

    return (char*)software;             // software version
}

/*  -----------  local functions  ----------------------------------------
 */

static int kvaser_error(canStatus status)
{
    /* note: all Kvaser CANlib error codes are negative */
    if ((canOK > status) && (status > canERR__RESERVED))
        return KVASER_ERR_OFFSET + (int)status;

    return KVASER_ERR_UNKNOWN;
}

static canStatus kvaser_capability(int channel, can_mode_t *capability)
{
    int feature = 0x0000;               // channel capability
    canStatus rc;                       // return value

    if ((rc = canGetChannelData(channel, canCHANNELDATA_CHANNEL_CAP,
                                (void*)&feature, sizeof(feature))) != canOK)
        return rc;

    capability->fdoe = (feature & canCHANNEL_CAP_CAN_FD) ? 1 : 0;
    capability->brse = (feature & canCHANNEL_CAP_CAN_FD) ? 1 : 0;
    capability->niso = (feature & canCHANNEL_CAP_CAN_FD_NONISO) ? 1 : 0;
    capability->shrd = 1; // shared access is supported (all ifaces?)
    capability->nxtd = 1; // suppressing 29-bit id's is supported since v0.2
    capability->nrtr = 1; // suppressing remote frames is supported since v0.2
    capability->err = 1;  // error frame reporting can be turned on and off by ioctrl
    capability->mon = (feature & canCHANNEL_CAP_SILENT_MODE) ? 1 : 0;

    return canOK;
}

static int index2params(int index, btr_nominal_t *params)
{
    switch (index) {
    case CANBTR_INDEX_1M: KVASER_BDR_1000((*params)); break;
#ifndef OPTION_KVASER_CiA_BIT_TIMING
    case CANBTR_INDEX_800K: return CANERR_BAUDRATE;
#else
    case CANBTR_INDEX_800K: KVASER_BDR_800((*params)); break;
#endif
    case CANBTR_INDEX_500K: KVASER_BDR_500((*params)); break;
    case CANBTR_INDEX_250K: KVASER_BDR_250((*params)); break;
    case CANBTR_INDEX_125K: KVASER_BDR_125((*params)); break;
    case CANBTR_INDEX_100K: KVASER_BDR_100((*params)); break;
    case CANBTR_INDEX_50K: KVASER_BDR_50((*params)); break;
    case CANBTR_INDEX_20K: KVASER_BDR_20((*params)); break;
    case CANBTR_INDEX_10K: KVASER_BDR_10((*params)); break;
    default: return CANERR_BAUDRATE;
    }
    return CANERR_NOERROR;
}

static int bitrate2params(const can_bitrate_t *bitrate, btr_nominal_t *params)
{
    if ((bitrate->btr.nominal.brp < CANBTR_NOMINAL_BRP_MIN) || (CANBTR_NOMINAL_BRP_MAX < bitrate->btr.nominal.brp))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.tseg1 < CANBTR_NOMINAL_TSEG1_MIN) || (CANBTR_NOMINAL_TSEG1_MAX < bitrate->btr.nominal.tseg1))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.tseg2 < CANBTR_NOMINAL_TSEG2_MIN) || (CANBTR_NOMINAL_TSEG2_MAX < bitrate->btr.nominal.tseg2))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.sjw < CANBTR_NOMINAL_SJW_MIN) || (CANBTR_NOMINAL_SJW_MAX < bitrate->btr.nominal.sjw))
        return CANERR_BAUDRATE;
    if (bitrate->btr.nominal.brp == 0)  // devide-by-zero!
        return CANERR_BAUDRATE;

    /* nominal bit-rate = frequency / (brp * (1 + tseg1 + tseg2)) */
    params->baud = bitrate->btr.frequency
                 / ((long)bitrate->btr.nominal.brp * (1l + (long)bitrate->btr.nominal.tseg1 + (long)bitrate->btr.nominal.tseg2));
    params->tseg1 = bitrate->btr.nominal.tseg1;
    params->tseg2 = bitrate->btr.nominal.tseg2;
    params->sjw = bitrate->btr.nominal.sjw;
    params->sam = bitrate->btr.nominal.sam;

    return CANERR_NOERROR;
}

static int params2bitrate(const btr_nominal_t *params, long frequency, can_bitrate_t *bitrate)
{
    /* Kvaser canLin32 doesn't offer the used controller frequency and bit-rate prescaler.
     * We suppose it's running with 80MHz and calculate the bit-rate prescaler as follows:
     *
     * (1) brp = 80HHz / baud * ((1- tseg1 + tseq2))
     */
    if (params->baud == 0)              // divide-by-zero!
        return CANERR_BAUDRATE;
    bitrate->btr.frequency = frequency;
    bitrate->btr.nominal.brp = (unsigned short)(frequency
                             / (params->baud * (long)(1u + params->tseg1 + params->tseg2)));
    bitrate->btr.nominal.tseg1 = params->tseg1;
    bitrate->btr.nominal.tseg2 = params->tseg2;
    bitrate->btr.nominal.sjw = params->sjw;
    bitrate->btr.nominal.sam = params->sam;

    return CANERR_NOERROR;
}

static int bitrate2paramsFd(const can_bitrate_t *bitrate, btr_data_t *params)
{
    if ((bitrate->btr.data.brp < CANBTR_DATA_BRP_MIN) || (CANBTR_DATA_BRP_MAX < bitrate->btr.data.brp))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.data.tseg1 < CANBTR_DATA_TSEG1_MIN) || (CANBTR_DATA_TSEG1_MAX < bitrate->btr.data.tseg1))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.data.tseg2 < CANBTR_DATA_TSEG2_MIN) || (CANBTR_DATA_TSEG2_MAX < bitrate->btr.data.tseg2))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.data.sjw < CANBTR_DATA_SJW_MIN) || (CANBTR_DATA_SJW_MAX < bitrate->btr.data.sjw))
        return CANERR_BAUDRATE;
    if (bitrate->btr.nominal.brp == 0)  // divide-by-zero!
        return CANERR_BAUDRATE;

    /* data bit-rate = frequency / (brp * (1 + tseg1 + tseg2)) */
    params->baud = bitrate->btr.frequency
                 / ((long)bitrate->btr.data.brp * (1l + (long)bitrate->btr.data.tseg1 + (long)bitrate->btr.data.tseg2));
    params->tseg1 = bitrate->btr.data.tseg1;
    params->tseg2 = bitrate->btr.data.tseg2;
    params->sjw = bitrate->btr.data.sjw;

    return CANERR_NOERROR;
}

static int paramsFd2bitrate(const btr_data_t *params, long frequency, can_bitrate_t *bitrate)
{
    /* Kvaser canLin32 doesn't offer the used controller frequency and bit-rate prescaler.
     * We suppose it's running with 80MHz and calculate the bit-rate prescaler as follows:
     *
     * (1) brp = 80HHz / baud * ((1- tseg1 + tseq2))
     */
    if (params->baud == 0)              // divide-by-zero!
        return CANERR_BAUDRATE;
    bitrate->btr.data.brp = (unsigned short)(frequency
                          / (params->baud * (long)(1u + params->tseg1 + params->tseg2)));
    bitrate->btr.data.tseg1 = params->tseg1;
    bitrate->btr.data.tseg2 = params->tseg2;
    bitrate->btr.data.sjw = params->sjw;

    return CANERR_NOERROR;
}

/*  - - - - - -  CAN API V3 properties  - - - - - - - - - - - - - - - - -
 */
static int lib_parameter(uint16_t param, void *value, size_t nbyte)
{
    int rc = CANERR_ILLPARA;            // suppose an invalid parameter
    int i = 0;                          // one always needs an i

    static int idx_board = EOF;         // actual index in the interface list

    if (value == NULL) {                // check for null-pointer
        if ((param != CANPROP_SET_FIRST_CHANNEL) &&
           (param != CANPROP_SET_NEXT_CHANNEL))
            return CANERR_NULLPTR;
    }
    /* CAN library properties */
    switch (param) {
    case CANPROP_GET_SPEC:              // version of the wrapper specification (uint16_t)
        if (nbyte >= sizeof(uint16_t)) {
            *(uint16_t*)value = (uint16_t)CAN_API_SPEC;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_VERSION:           // version number of the library (uint16_t)
        if (nbyte >= sizeof(uint16_t)) {
            *(uint16_t*)value = ((uint16_t)VERSION_MAJOR << 8)
                              | ((uint16_t)VERSION_MINOR & 0xFu);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_PATCH_NO:          // patch number of the library (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)VERSION_PATCH;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_BUILD_NO:          // build number of the library (uint32_t)
        if (nbyte >= sizeof(uint32_t)) {
            *(uint32_t*)value = (uint32_t)VERSION_BUILD;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_ID:        // library id of the library (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            *(int32_t*)value = (int32_t)KVASER_LIB_ID;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_VENDOR:    // vendor name of the library (char[256])
        if ((nbyte > strlen(CAN_API_VENDOR)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, CAN_API_VENDOR);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_DLLNAME:   // file name of the library (char[256])
        if ((nbyte > strlen(KVASER_LIB_WRAPPER)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, KVASER_LIB_WRAPPER);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_VENDOR:     // vendor name of the CAN interface (char[256])
        if ((nbyte > strlen(KVASER_LIB_VENDOR)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, KVASER_LIB_VENDOR);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_DLLNAME:    // file name of the CAN interface (char[256])
        if ((nbyte > strlen(KVASER_LIB_CANLIB)) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            strcpy((char*)value, KVASER_LIB_CANLIB);
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_SET_FIRST_CHANNEL:     // set index to the first entry in the interface list (NULL)
        idx_board = 0;
        rc = (can_boards[idx_board].type != EOF) ? CANERR_NOERROR : CANERR_RESOURCE;
        break;
    case CANPROP_SET_NEXT_CHANNEL:      // set index to the next entry in the interface list (NULL)
        if ((0 <= idx_board) && (idx_board < KVASER_BOARDS)) {
            if (can_boards[idx_board].type != EOF)
                idx_board++;
            rc = (can_boards[idx_board].type != EOF) ? CANERR_NOERROR : CANERR_RESOURCE;
        }
        else
            rc = CANERR_RESOURCE;
        break;
    case CANPROP_GET_CHANNEL_TYPE:      // get device type at actual index in the interface list (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            if ((0 <= idx_board) && (idx_board < KVASER_BOARDS) &&
                (can_boards[idx_board].type != EOF)) {
                *(int32_t*)value = (int32_t)can_boards[idx_board].type;
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_NAME:      // get device name at actual index in the interface list (char[256])
        if (nbyte <= CANPROP_MAX_BUFFER_SIZE) {
            if ((0 <= idx_board) && (idx_board < KVASER_BOARDS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, can_boards[idx_board].name, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_DLLNAME:   // get file name of the DLL at actual index in the interface list (char[256])
        if (nbyte <= CANPROP_MAX_BUFFER_SIZE) {
            strncpy((char*)value, KVASER_LIB_CANLIB, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_CHANNEL_VENDOR_ID: // get library id at actual index in the interface list (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            *(int32_t*)value = (int32_t)KVASER_LIB_ID;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_CHANNEL_VENDOR_NAME: // get vendor name at actual index in the interface list (char[256])
        if (nbyte <= CANPROP_MAX_BUFFER_SIZE) {
            strncpy((char*)value, KVASER_LIB_VENDOR, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    default:
        rc = CANERR_NOTSUPP;
        break;
    }
    return rc;
}

static int drv_parameter(int handle, uint16_t param, void *value, size_t nbyte)
{
    int rc = CANERR_ILLPARA;            // suppose an invalid parameter
    can_bitrate_t bitrate;
    can_speed_t speed;
    can_mode_t mode;
    uint8_t status;
    uint8_t load;
    canStatus sts;

    assert(IS_HANDLE_VALID(handle));    // just to make sure

    if (value == NULL)                  // check for null-pointer
        return CANERR_NULLPTR;

    /* CAN interface properties */
    switch (param) {
    case CANPROP_GET_DEVICE_TYPE:       // device type of the CAN interface (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            *(int32_t*)value = (int32_t)can[handle].channel;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_NAME:       // device name of the CAN interface (char[256])
        if ((sts = canGetChannelData(can[handle].channel, canCHANNELDATA_CHANNEL_NAME,
                                   (void*)value, (DWORD)nbyte)) == canOK)
            rc = CANERR_NOERROR;
        else
            rc = kvaser_error(sts);
        break;
    case CANPROP_GET_OP_CAPABILITY:     // supported operation modes of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            if ((sts = kvaser_capability(can[handle].channel, &mode)) == canOK) {
                *(uint8_t*)value = (uint8_t)mode.byte;
                rc = CANERR_NOERROR;
            } else
                rc = kvaser_error(sts);
        }
        break;
    case CANPROP_GET_OP_MODE:           // active operation mode of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            *(uint8_t*)value = (uint8_t)can[handle].mode.byte;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_BITRATE:           // active bit-rate of the CAN controller (can_bitrate_t)
        if (nbyte >= sizeof(can_bitrate_t)) {
            if (((rc = can_bitrate(handle, &bitrate, NULL)) == CANERR_NOERROR) || (rc == CANERR_OFFLINE)) {
                memcpy(value, &bitrate, sizeof(can_bitrate_t));
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_SPEED:             // active bus speed of the CAN controller (can_speed_t)
        if (nbyte >= sizeof(can_speed_t)) {
            if (((rc = can_bitrate(handle, NULL, &speed)) == CANERR_NOERROR) || (rc == CANERR_OFFLINE)) {
                memcpy(value, &speed, sizeof(can_speed_t));
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_STATUS:            // current status register of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            if ((rc = can_status(handle, &status)) == CANERR_NOERROR) {
                *(uint8_t*)value = (uint8_t)status;
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_BUSLOAD:           // current bus load of the CAN controller (uint8_t)
        if (nbyte >= sizeof(uint8_t)) {
            if ((rc = can_busload(handle, &load, NULL)) == CANERR_NOERROR) {
                *(uint8_t*)value = (uint8_t)load;
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_TX_COUNTER:        // total number of sent messages (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)can[handle].counters.tx;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_RX_COUNTER:        // total number of reveiced messages (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)can[handle].counters.rx;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_ERR_COUNTER:       // total number of reveiced error frames (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            *(uint64_t*)value = (uint64_t)can[handle].counters.err;
            rc = CANERR_NOERROR;
        }
        break;
    default:
        if ((CANPROP_GET_VENDOR_PROP <= param) &&  // get a vendor-specific property value (void*)
           (param < (CANPROP_GET_VENDOR_PROP + CANPROP_VENDOR_PROP_RANGE))) {
            if ((sts = canIoCtl(can[handle].handle, (unsigned int)(param - CANPROP_GET_VENDOR_PROP),
                                                           (void*)value, (DWORD)nbyte)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        else if ((CANPROP_SET_VENDOR_PROP <= param) &&  // set a vendor-specific property value (void*)
                (param < (CANPROP_SET_VENDOR_PROP + CANPROP_VENDOR_PROP_RANGE))) {
            if ((sts = canIoCtl(can[handle].handle, (unsigned int)(param - CANPROP_SET_VENDOR_PROP),
                                                           (void*)value, (DWORD)nbyte)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        else                            // or general library properties (see lib_parameter)
            rc = lib_parameter(param, value, nbyte);
        break;
    }
    return rc;
}

/*  - - - - - -  Bus-speed calculator  - - - - - - - - - - - - - - - - - -
 */
static int calc_speed(can_bitrate_t *bitrate, can_speed_t *speed, int modify)
{
    can_bitrate_t temporary;            // bit-rate settings
    btr_nominal_t nominal;              // nominal bit-rate
    int rc;

    memset(&temporary, 0, sizeof(can_bitrate_t));
    memset(&nominal, 0, sizeof(btr_nominal_t));

    if (bitrate->index <= 0) {
        if ((rc = index2params(bitrate->index, &nominal)) != CANERR_NOERROR)
            return rc;
        if (nominal.baud < 0) {         // translate Kvaser defaults
            if ((rc = canTranslateBaud(&nominal.baud, &nominal.tseg1, &nominal.tseg2,
                                      &nominal.sjw, &nominal.sam, &nominal.sync)) != canOK)
                return rc;
        }
#ifndef OPTION_KVASER_CiA_BIT_TIMING
        if ((rc = params2bitrate(&nominal, KVASER_FREQ_DEFAULT, &temporary)) != CANERR_NOERROR)
#else
        if ((rc = params2bitrate(&nominal, CANBTR_FREQ_SJA1000, &temporary)) != CANERR_NOERROR)
#endif
            return rc;

        if (modify)                     // translate index to bit-rate
            memcpy(bitrate, &temporary, sizeof(can_bitrate_t));

        speed->nominal.fdoe = 0;
        speed->data.brse = 0;
    }
    else {
        memcpy(&temporary, bitrate, sizeof(can_bitrate_t));

        speed->data.brse = temporary.btr.data.brp ? 1 : 0;
    }
    /* nominal bit-rate:
     *
     * (1) speed = freq / (brp * (1 + tseg1 +tseg2))
     *
     * (2) sp = (1 + tseg1) / (1 + tseg1 +tseg2)
     */
    if ((temporary.btr.nominal.brp < CANBTR_NOMINAL_BRP_MIN) || (CANBTR_NOMINAL_BRP_MAX < temporary.btr.nominal.brp))
        return CANERR_BAUDRATE;
    if ((temporary.btr.nominal.tseg1 < CANBTR_NOMINAL_TSEG1_MIN) || (CANBTR_NOMINAL_TSEG1_MAX < temporary.btr.nominal.tseg1))
        return CANERR_BAUDRATE;
    if ((temporary.btr.nominal.tseg2 < CANBTR_NOMINAL_TSEG2_MIN) || (CANBTR_NOMINAL_TSEG2_MAX < temporary.btr.nominal.tseg2))
        return CANERR_BAUDRATE;
    if ((temporary.btr.nominal.sjw < CANBTR_NOMINAL_SJW_MIN) || (CANBTR_NOMINAL_SJW_MAX < temporary.btr.nominal.sjw))
        return CANERR_BAUDRATE;
    speed->nominal.speed = (float)(temporary.btr.frequency)
                         / (float)(temporary.btr.nominal.brp * (1u + temporary.btr.nominal.tseg1 + temporary.btr.nominal.tseg2));
    speed->nominal.samplepoint = (float)(1u + temporary.btr.nominal.tseg1)
                               / (float)(1u + temporary.btr.nominal.tseg1 + temporary.btr.nominal.tseg2);

    /* data bit-rate (CAN FD only):
     *
     * (1) speed = freq / (brp * (1 + tseg1 +tseg2))
     *
     * (2) sp = (1 + tseg1) / (1 + tseg1 +tseg2)
     */
    if (speed->data.brse) {
        if ((temporary.btr.data.brp < CANBTR_DATA_BRP_MIN) || (CANBTR_DATA_BRP_MAX < temporary.btr.data.brp))
            return CANERR_BAUDRATE;
        if ((temporary.btr.data.tseg1 < CANBTR_DATA_TSEG1_MIN) || (CANBTR_DATA_TSEG1_MAX < temporary.btr.data.tseg1))
            return CANERR_BAUDRATE;
        if ((temporary.btr.data.tseg2 < CANBTR_DATA_TSEG2_MIN) || (CANBTR_DATA_TSEG2_MAX < temporary.btr.data.tseg2))
            return CANERR_BAUDRATE;
        if ((temporary.btr.data.sjw < CANBTR_DATA_SJW_MIN) || (CANBTR_DATA_SJW_MAX < temporary.btr.data.sjw))
            return CANERR_BAUDRATE;
        speed->data.speed = (float)(temporary.btr.frequency)
                          / (float)(temporary.btr.data.brp * (1u + temporary.btr.data.tseg1 + temporary.btr.data.tseg2));
        speed->data.samplepoint = (float)(1u + temporary.btr.data.tseg1)
                                / (float)(1u + temporary.btr.data.tseg1 + temporary.btr.data.tseg2);
    }
    else {
        speed->data.speed = 0.0;
        speed->data.samplepoint = 0.0;
    }
    return CANERR_NOERROR;
}

/*  -----------  revision control  ---------------------------------------
 */

char* can_version(void)
{
    return (char*)version;
}
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
