/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  CAN Interface API, Version 3 (for Kvaser CAN Interfaces)
 *
 *  Copyright (c) 2017-2024 Uwe Vogt, UV Software, Berlin (info@uv-software.de)
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
 *  along with KvaserCAN-Wrapper.  If not, see <https://www.gnu.org/licenses/>.
 */
/** @addtogroup  can_api
 *  @{
 */
#ifdef _MSC_VER
//no Microsoft extensions please!
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif
#endif
#if defined(_WIN64)
#define PLATFORM  "x64"
#elif defined(_WIN32)
#define PLATFORM  "x86"
#else
#error Platform not supported
#endif

/*  -----------  includes  -----------------------------------------------
 */
#include "can_defs.h"
#include "can_api.h"
#include "can_btr.h"

#include <windows.h>

#include "canlib.h"
#include "canstat.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>

/*  -----------  options  ------------------------------------------------
 */
#if (OPTION_CAN_2_0_ONLY != 0)
#error Compilation with legacy CAN 2.0 frame format!
#endif

/*  -----------  defines  ------------------------------------------------
 */
#ifndef CAN_MAX_HANDLES
#define CAN_MAX_HANDLES         (16)    // maximum number of open handles
#endif
#define INVALID_HANDLE          (-1)
#define IS_HANDLE_VALID(hnd)    ((0 <= (hnd)) && ((hnd) < CAN_MAX_HANDLES))
#define IS_HANDLE_OPENED(hnd)   (can[(hnd)].handle != canINVALID_HANDLE)
#ifndef DLC2LEN
#define DLC2LEN(x)              dlc_table[((x) < 16) ? (x) : 15]
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
#if (OPTION_KVASER_BIT_TIMING != OPTION_DISABLED)
#define KVASER_BDR_1000(btr)    do{ btr.bitRate=canBITRATE_1M; } while(0)
#define KVASER_BDR_500(btr)     do{ btr.bitRate=canBITRATE_500K; } while(0)
#define KVASER_BDR_250(btr)     do{ btr.bitRate=canBITRATE_250K; } while(0)
#define KVASER_BDR_125(btr)     do{ btr.bitRate=canBITRATE_125K; } while(0)
#define KVASER_BDR_100(btr)     do{ btr.bitRate=canBITRATE_100K; } while(0)
#define KVASER_BDR_50(btr)      do{ btr.bitRate=canBITRATE_50K; } while(0)
#define KVASER_BDR_20(btr)      do{ btr.bitRate=20000; btr.tseg1=11; btr.tseg2=4; btr.sjw=1; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_10(btr)      do{ btr.bitRate=canBITRATE_10K; } while(0)
#else
#define KVASER_BDR_1000(btr)    do{ btr.bitRate=1000000; btr.tseg1=5 ; btr.tseg2=2; btr.sjw=1; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_800(btr)     do{ btr.bitRate=800000;  btr.tseg1=7 ; btr.tseg2=2; btr.sjw=1; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_500(btr)     do{ btr.bitRate=500000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=1; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_250(btr)     do{ btr.bitRate=250000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=1; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_125(btr)     do{ btr.bitRate=125000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=1; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_100(btr)     do{ btr.bitRate=100000;  btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_50(btr)      do{ btr.bitRate=50000;   btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_20(btr)      do{ btr.bitRate=20000,   btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.noSamp=1; btr.syncmode=0; } while(0)
#define KVASER_BDR_10(btr)      do{ btr.bitRate=10000;   btr.tseg1=13; btr.tseg2=2; btr.sjw=2; btr.noSamp=1; btr.syncmode=0; } while(0)
#endif
#define KVASER_FREQ_DEFAULT     (80000000)
#define KVASER_CHANNEL_DEFAULT  (0)

#define FILTER_STD_CODE         (uint32_t)(0x000)
#define FILTER_STD_MASK         (uint32_t)(0x000)
#define FILTER_XTD_CODE         (uint32_t)(0x00000000)
#define FILTER_XTD_MASK         (uint32_t)(0x00000000)
#define FILTER_STD_VALID_MASK   (uint64_t)(0x000007FF000007FF)
#define FILTER_XTD_VALID_MASK   (uint64_t)(0x1FFFFFFF1FFFFFFF)
#define FILTER_RESET_VALUE      (uint64_t)(0x0000000000000000)
#ifndef SYSERR_OFFSET
#define SYSERR_OFFSET           (-10000)
#endif
#define LIB_ID                  KVASER_LIB_ID
#define LIB_DLLNAME             KVASER_LIB_WRAPPER
#define DEV_VENDOR              KVASER_LIB_VENDOR
#define DEV_DLLNAME             KVASER_LIB_CANLIB
#define NUM_CHANNELS            KVASER_BOARDS

/*  -----------  types  --------------------------------------------------
 */
typedef struct {                        // nominal bit-rate:
    long bitRate;                       //   bit-rate in [bps]
    unsigned int tseg1;                 //   time segement 1
    unsigned int tseg2;                 //   time segement 2
    unsigned int sjw;                   //   sync. jump width
    unsigned int noSamp;                //   no. of samples
    unsigned int syncmode;              //   (unsupported)
}   btr_nominal_t;

typedef struct {                        // data bit-rate:
    long bitRate;                       //   bit-rate in [bps]
    unsigned int tseg1;                 //   time segement 1
    unsigned int tseg2;                 //   time segement 2
    unsigned int sjw;                   //   sync. jump width
}   btr_data_t;

typedef struct {                        // message filtering:
    struct {                            //   acceptance filter:
        uint32_t code;                  //     acceptance code
        uint32_t mask;                  //     acceptance mask
    } std, xtd;                         //   for standard and extended frames
}   can_filter_t;

typedef struct {                        // frame counters:
    uint64_t tx;                        //   number of transmitted CAN frames
    uint64_t rx;                        //   number of received CAN frames
    uint64_t err;                       //   number of receiced error frames
}   can_counter_t;

typedef struct {                        // error code capture:
    uint8_t lec;                        //   last error code
    uint8_t rx_err;                     //   receive error counter
    uint8_t tx_err;                     //   transmit error counter
}   can_error_t;

typedef struct {                        // Kvaser CAN interface:
    CanHandle  handle;                  //   hardware channel handle
    int        channel;                 //   channel number of the CAN board
    long       frequency;               //   frequency of the CAN controller
    can_mode_t mode;                    //   operation mode of the CAN channel
    can_filter_t filter;                //   message filtering settings
    can_status_t status;                //   8-bit status register
    can_error_t error;                  //   error code capture
    can_counter_t counters;             //   statistical counters
}   can_interface_t;

/*  -----------  prototypes  ---------------------------------------------
 */
static void var_init(void);             // initialize all variables
static int all_closed(void);            // check if all handles closed

static int exit_channel(int handle);    // teardown a single channel
static int kill_channel(int handle);    // signal a single channel

static void can_message(long id, unsigned int flags, unsigned int len, const unsigned char *data, can_message_t *msg);
static void can_message_sts(can_status_t status, can_error_t error, can_message_t *msg);
static void can_timestamp(unsigned long timestamp, can_message_t *msg);

static int kvaser_error(canStatus);     // Kvaser specific errors
/*static int kvaser_compatibility(void);   Kvaser compatibility check*/

static canStatus kvaser_capability(int channel, can_mode_t *capability);
static canStatus kvaser_get_filter(int handle, uint64_t *filter, int xtd);
static canStatus kvaser_set_filter(int handle, uint64_t filter, int xtd);
static canStatus kvaser_reset_filter(int handle);

static int map_index2params(int index, btr_nominal_t *busParams);
static int map_bitrate2params(const can_bitrate_t *bitrate, btr_nominal_t *busParams);
static int map_params2bitrate(const btr_nominal_t *busParams, long canClock, can_bitrate_t *bitrate);
static int map_bitrate2paramsFd(const can_bitrate_t *bitrate, btr_data_t *busParams);
static int map_paramsFd2bitrate(const btr_data_t *busParams, long canClock, can_bitrate_t *bitrate);

static int lib_parameter(uint16_t param, void *value, size_t nbyte);
static int drv_parameter(int handle, uint16_t param, void *value, size_t nbyte);

/*  -----------  variables  ----------------------------------------------
 */
static const char version[] = "CAN API V3 for Kvaser CAN Interfaces, Version " VERSION_STRING;

can_board_t can_boards[NUM_CHANNELS+1] = {  // list of supported CAN channels
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
static can_interface_t can[CAN_MAX_HANDLES];  // interface handles
static int init = 0;                    // initialization flag

/*  -----------  functions  ----------------------------------------------
 */
int can_test(int32_t board, uint8_t mode, const void *param, int *result)
{
    int channels = 0;                   // number of channels
    int feature = 0x0000;               // channel capability
    int flags = 0x0000;                 // channel flags
    can_mode_t capa;                    // channel capability
    canStatus sts;                      // return value
    int i;                              // loop variable

    if (!init) {                        // if not initialized:
        var_init();                     //   initialize all variables
        canInitializeLibrary();         //   initialize the driver
        init = 1;                       //   set initialization flag
    }
    // check if the CAN channel is available
    if ((sts = canGetNumberOfChannels(&channels)) != canOK)
        return kvaser_error(sts);
    if (board >= channels) {
        if (result)                     // CAN channel not present
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
    // get channel capability from CAN channel
    if ((sts = canGetChannelData(board, canCHANNELDATA_CHANNEL_CAP,
                               (void*)&feature, sizeof(feature))) != canOK)
        return kvaser_error(sts);
    if ((feature & canCHANNEL_CAP_VIRTUAL)) {
        if (result)                     // declare as not available
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
    if ((feature & canCHANNEL_CAP_SIMULATED)) {
        if (result)                     // declare as not available
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
    // get channel access flags from CAN channel
    if ((sts = canGetChannelData(board, canCHANNELDATA_CHANNEL_FLAGS,
                                (void*)&flags, sizeof(flags))) != canOK)
        return kvaser_error(sts);
    // check if the CAN channel is present and available
    if (result) {
#ifndef KVASER_SHARED_ACCESS
        if (flags & canCHANNEL_IS_OPEN)
            *result = CANBRD_OCCUPIED;  // CAN channel occupied by another process
        else
            *result = CANBRD_PRESENT;   // CAN channel present and available
#else
        if (flags & canCHANNEL_IS_EXCLUSIVE)
            *result = CANBRD_OCCUPIED;  // CAN channel occupied by another process
        else if ((flags & canCHANNEL_IS_OPEN) && !(mode & CANMODE_SHRD))
            *result = CANBRD_OCCUPIED;  // CAN channel occupied by another process
        else
            *result = CANBRD_PRESENT;   // CAN channel present and available
#endif
        for (i = 0; i < CAN_MAX_HANDLES; i++)
            if ((can[i].handle != canINVALID_HANDLE) &&
                (can[i].channel == board) && !(mode & CANMODE_SHRD))
                *result = CANBRD_OCCUPIED; // CAN channel occupied by ourself
    }
    // get operation capabilities from CAN channel
    if ((sts = kvaser_capability(board, &capa)) != CANERR_NOERROR)
        return kvaser_error(sts);
    // check given operation mode against the operation capability
    if ((mode & ~capa.byte) != 0)
        return CANERR_ILLPARA;
    if (!(mode & CANMODE_FDOE) && ((mode & CANMODE_BRSE) || (mode & CANMODE_NISO)))
        return CANERR_ILLPARA;
    // when the music is over, turn out the lights
    if (all_closed()) {                 // if no open handle then
		init = 0;                       //   clear initialization flag
	}
    (void)param;
    return CANERR_NOERROR;
}

int can_init(int32_t board, uint8_t mode, const void *param)
{
    can_mode_t capa;                    // channel capability
    int flags = 0x0000;                 // flags for canOpenChannel()
    canHandle result;                   // Kvaser handle or error
    canStatus sts;                      // return value
    int handle;                         // handle index

    if (!init) {                        // if not initialized:
        var_init();                     //   initialize all variables
        canInitializeLibrary();         //   initialize the driver
        init = 1;                       //   set initialization flag
    }
    for (handle = 0; handle < CAN_MAX_HANDLES; handle++) {
        if ((can[handle].handle != canINVALID_HANDLE) &&
            (can[handle].channel == board))  // channel already in use
            return CANERR_YETINIT;
    }
    for (handle = 0; handle < CAN_MAX_HANDLES; handle++) {
        if (can[handle].handle == canINVALID_HANDLE)  // get an unused handle, if any
            break;
    }
    if (!IS_HANDLE_VALID(handle)) {     // no free handle found
        return CANERR_NOTINIT;
    }
    // get operation capabilities from channel and check with given operation mode
    if ((sts = kvaser_capability(board, &capa)) != CANERR_NOERROR)
        return kvaser_error(sts);
    if ((mode & ~capa.byte) != 0)
        return CANERR_ILLPARA;
    if (!(mode & CANMODE_FDOE) && ((mode & CANMODE_BRSE) || (mode & CANMODE_NISO)))
        return CANERR_ILLPARA;
    // initialize the CAN channel with given operation mode
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
    if ((result = canOpenChannel(board, flags)) < canOK) {
        return kvaser_error(result);
    }
    if (!(mode & CANMODE_MON)) {        // normal operation mode
        if ((sts = canSetBusOutputControl(result, canDRIVER_NORMAL)) != canOK)
            return kvaser_error(sts);
    }
    else {                              // listen-onlyl operation mode
        if ((sts = canSetBusOutputControl(result, canDRIVER_SILENT)) != canOK)
            return kvaser_error(sts);
    }
    // store the handle and the operation mode
    can[handle].handle = result;        // handle of the CAN board/channel
    can[handle].channel = board;        // channel number of the CAN board
    can[handle].mode.byte = mode;       // store selected operation mode
    can[handle].status.byte = CANSTAT_RESET; // CAN controller not started yet
    return handle;                      // return the handle
}

static int exit_channel(int handle)
{
    canStatus sts;                      // return value

    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;
    /*if (!can[handle].status.can_stopped) // go to CAN INIT mode (bus off)*/
        (void)canBusOff(can[handle].handle);
    if ((sts = canClose(can[handle].handle)) != canOK) // release the CAN interface!
        return kvaser_error(sts);

    can[handle].status.byte |= CANSTAT_RESET;// CAN controller in INIT state
    can[handle].handle = canINVALID_HANDLE; // handle can be used again
    return CANERR_NOERROR;
}

int can_exit(int handle)
{
    int rc;                             // return value
    int i;                              // loop variable

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (handle != CANEXIT_ALL) {        // close a single handle
        if ((rc = exit_channel(handle)) != CANERR_NOERROR)
            return rc;
    }
    else {
        for (i = 0; i < CAN_MAX_HANDLES; i++) {
            (void)exit_channel(i);      // close all open handles
        }
    }
    // when the music is over, turn out the lights
    if (all_closed()) {                 // if no open handle then
        init = 0;                       //   clear initialization flag
    }
    return CANERR_NOERROR;
}

static int kill_channel(int handle)
{
    HANDLE hEvent = NULL;               // event object

    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;
    if ((canIoCtl(can[handle].handle, canIOCTL_GET_EVENTHANDLE,
                    (void*)&hEvent, sizeof(hEvent)) == canOK) &&
        (hEvent != NULL)) {
        if (!SetEvent(hEvent))  // signal event object
            return SYSERR_OFFSET - (int)GetLastError();
    }
    return CANERR_NOERROR;
}

int can_kill(int handle)
{
    int rc;                             // return value
    int i;                              // loop variable

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (handle != CANKILL_ALL) {        // signal a single handle
        if ((rc = kill_channel(handle)) != CANERR_NOERROR)
            return rc;
    }
    else {
        for (i = 0; i < CAN_MAX_HANDLES; i++) {
            (void)kill_channel(i);      // signal all open handles
        }
    }
    return CANERR_NOERROR;
}

int can_start(int handle, const can_bitrate_t *bitrate)
{
    unsigned char error_frames;         // error frame reporting
    btr_nominal_t nominal;              // nominal bit-rate
    btr_data_t data;                    // data bit-rate
    canStatus sts;                      // return value
    int rc;

    memset(&nominal, 0, sizeof(btr_nominal_t));
    memset(&data, 0, sizeof(btr_data_t));

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;
    if (bitrate == NULL)                // check for null-pointer
        return CANERR_NULLPTR;
    if (!can[handle].status.can_stopped) // must be stopped
        return CANERR_ONLINE;

    // convert CAN API bit-rate to Kvaser bit-rate
    if (bitrate->index <= 0) {          // bit-rate from index
        if (can[handle].mode.fdoe)
            return CANERR_BAUDRATE;     //   CAN 2.0 only
        if ((rc = map_index2params(bitrate->index, &nominal)) != CANERR_NOERROR)
            return rc;
        if ((sts = canSetBusParams(can[handle].handle, nominal.bitRate, nominal.tseg1, nominal.tseg2,
                                                       nominal.sjw, nominal.noSamp, nominal.syncmode)) != canOK)
            return (sts != canERR_PARAM) ? kvaser_error(sts) : CANERR_BAUDRATE;
#if (OPTION_KVASER_BIT_TIMING != OPTION_DISABLED)
        can[handle].frequency = KVASER_FREQ_DEFAULT;
#else
        can[handle].frequency = CANBTR_FREQ_SJA1000;
#endif
    }
    else {                              // bit-rate from parameter
        if ((rc = map_bitrate2params(bitrate, &nominal)) != CANERR_NOERROR)
            return rc;
        if ((sts = canSetBusParams(can[handle].handle, nominal.bitRate, nominal.tseg1, nominal.tseg2,
                                                       nominal.sjw, nominal.noSamp, nominal.syncmode)) != canOK)
            return (sts != canERR_PARAM) ? kvaser_error(sts) : CANERR_BAUDRATE;
        /* bit-rate for CAN FD data (BRSE) */
        if (can[handle].mode.fdoe && can[handle].mode.brse) {
            if ((rc = map_bitrate2paramsFd(bitrate, &data)) != CANERR_NOERROR)
                return rc;
            if ((sts = canSetBusParamsFd(can[handle].handle, data.bitRate, data.tseg1, data.tseg2, data.sjw)) != canOK)
                return (sts != canERR_PARAM) ? kvaser_error(sts) : CANERR_BAUDRATE;
        }
        can[handle].frequency = bitrate->btr.frequency;
    }
    // enable error frame reception if selected
    // TODO: move this to can_init()
    error_frames = can[handle].mode.err ? 1 : 0; // error frames
    if ((sts = canIoCtl(can[handle].handle, canIOCTL_SET_ERROR_FRAMES_REPORTING,
                       (void*)&error_frames, sizeof(error_frames))) != canOK)
        return kvaser_error(sts);
    // start the CAN controller
    if ((sts = canBusOn(can[handle].handle)) != canOK) {
        return kvaser_error(sts);
    }
    // clear old status, errors and counters
    can[handle].status.byte = 0x00u;
    can[handle].error.lec = 0x00u;
    can[handle].error.rx_err = 0u;
    can[handle].error.tx_err = 0u;
    can[handle].counters.tx = 0ull;
    can[handle].counters.rx = 0ull;
    can[handle].counters.err = 0ull;
    can[handle].status.can_stopped = 0;
    // CAN controller started!
    return CANERR_NOERROR;
}

int can_reset(int handle)
{
    int rc = CANERR_FATAL;              // return code

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;
    if (can[handle].status.can_stopped) // must be running
#if (OPTION_CANAPI_RETVALS == OPTION_DISABLED)
        return CANERR_OFFLINE;
#else
        // note: can_reset shall return CANERR_NOERROR even if
        //       the CAN controller has not been started
        return CANERR_NOERROR;
#endif
    // stop the CAN controller (bus off)
    rc = canBusOff(can[handle].handle);
    can[handle].status.can_stopped = (rc == CANERR_NOERROR) ? 1 : 0;
    return rc;
}

int can_write(int handle, const can_message_t *msg, uint16_t timeout)
{
    long id;                            // the message:
    unsigned int dlc, flags;
    unsigned char data[CANFD_MAX_LEN];
    canStatus sts;                      // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
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
        if (msg->dlc > CAN_MAX_LEN)     //   data length 0 .. 8
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
        if (msg->dlc > CANFD_MAX_DLC)   //   data length 0 .. 0Fh
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
    // transmit the message (with or w/o acknowledge)
    if (timeout == 0U)
        sts = canWrite(can[handle].handle, id, &data, dlc, flags);
    else
        sts = canWriteWait(can[handle].handle, id, &data, dlc, flags,
                (timeout < CANWAIT_INFINITE) ? (unsigned long)timeout : 0xFFFFFFFFul);
    // check for errors
    if (sts != canOK) {
        if ((sts & canERR_TXBUFOFL)) {  //   transmit queue full?
            can[handle].status.transmitter_busy = 1;
            return CANERR_TX_BUSY;      //     transmitter busy
        }
        else if ((sts & canERR_TIMEOUT)) {  // timed out?
            can[handle].status.transmitter_busy = 1;
            return CANERR_TX_BUSY;      //     transmitter busy
        }
        return kvaser_error(sts);       //   Kvaser specific error?
    }
    can[handle].status.transmitter_busy = 0; // message transmitted
    can[handle].counters.tx++;         // increment transmit counter
    return CANERR_NOERROR;
}

int can_read(int handle, can_message_t *msg, uint16_t timeout)
{
    long id;                            // the message:
    unsigned int len, flags;            //   length and flags
    unsigned char data[CANFD_MAX_LEN];  //   data payload
    unsigned long timestamp;            // time stamp (in [ms])
    canStatus sts;                      // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;
    if (msg == NULL)                    // check for null-pointer
        return CANERR_NULLPTR;
    if (can[handle].status.can_stopped) // must be running
        return CANERR_OFFLINE;

    memset(msg, 0, sizeof(can_message_t));
    msg->id = 0xFFFFFFFFu;
    msg->sts = 1;
repeat:
    // try to read a message
    if ((sts = canRead(can[handle].handle, &id, data, &len, &flags, &timestamp)) == canERR_NOMSG) {
        // blocking read or polling
        if (timeout > 0) {
            switch (canWaitForEvent(can[handle].handle, (timeout != CANWAIT_INFINITE) ? (DWORD)timeout : (DWORD)INFINITE)) {
            case canOK:
                break;                  //   one or more messages received
            case canERR_TIMEOUT:
                break;                  //   time-out, but look for old messages
            default:
                return CANERR_FATAL;    //   function failed!
            }
            if ((sts = canRead(can[handle].handle, &id, data, &len, &flags, &timestamp)) == canERR_NOMSG) {
                can[handle].status.receiver_empty = 1;
                return CANERR_RX_EMPTY; //   receiver empty
            }
        }
        else {
            can[handle].status.receiver_empty = 1;
            return CANERR_RX_EMPTY;     //   receiver empty
        }
    }
    // check for errors
    if (sts < canOK)                    // receive error?
    {
        return kvaser_error(sts);       //   something's wrong
    }
    if ((flags & canMSGERR_OVERRUN)) {  // queue overrun?
        can[handle].status.queue_overrun = 1;
        /* note: queue has overrun, but we have a message */
    }
    // convert Kvaser message to CAN API message
    // TODO: move this into separate functions if possible
    if ((flags & canMSG_EXT) && can[handle].mode.nxtd)
        goto repeat;                    // refuse extended frames
    if ((flags & canMSG_RTR) && can[handle].mode.nrtr)
        goto repeat;                    // refuse remote frames
    if ((flags & canMSG_ERROR_FRAME)) {
        unsigned int txErr = 0, rxErr = 0;
        (void)canReadErrorCounters(can[handle].handle, &txErr, &rxErr, NULL);
        (void)can_status(can[handle].handle, NULL);
        // update error and status register from error frame
        can[handle].error.lec = (uint8_t)((flags & canMSGERR_BUSERR) >> 8);
        can[handle].error.rx_err = (uint8_t)(rxErr & 0x00FF);
        can[handle].error.tx_err = (uint8_t)(txErr & 0x00FF);
        can[handle].status.bus_error = can[handle].error.lec ? 1 : 0;
        // refuse status message if suppressed by user
        if (!can[handle].mode.err)
            goto repeat;
        // status message: ID=000h, DLC=4 (status, lec, rx errors, tx errors)
        can_message_sts(can[handle].status, can[handle].error, msg);
        can[handle].counters.err++;
    }
    else {
        // decode Kvaser CAN message and increment receive counter
        can_message(id, flags, len, data, msg);
        can[handle].counters.rx++;
    }
    // time-stamp in nanoseconds
    can_timestamp(timestamp, msg);
    can[handle].status.receiver_empty = 0;
    // one message read
    return CANERR_NOERROR;
}

int can_status(int handle, uint8_t *status)
{
    unsigned long flags = 0ul;          // status flags
    canStatus sts;                      // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;

    if ((sts = canRequestChipStatus(can[handle].handle)) != canOK)
        return kvaser_error(sts);
    if ((sts = canReadStatus(can[handle].handle, &flags)) != canOK)
        return kvaser_error(sts);

    can[handle].status.bus_off = (flags & canSTAT_BUS_OFF)? 1 : 0;
    can[handle].status.bus_error = can[handle].error.lec ? 1 : 0;  // last eror code from error code capture (ECC)
    can[handle].status.warning_level = (flags & (canSTAT_ERROR_WARNING | canSTAT_ERROR_PASSIVE))? 1 : 0;
    can[handle].status.transmitter_busy |= (flags & canSTAT_TX_PENDING)? 1 : 0;
    can[handle].status.message_lost |= (flags & (canSTAT_OVERRUN /*| canSTAT_RXERR*/)) ? 1 : 0;  // FIXME: how?
    can[handle].status.queue_overrun |= (flags & canSTAT_OVERRUN) ? 1 : 0;
    if (status)                         // status-register
        *status = can[handle].status.byte;
    return CANERR_NOERROR;
}

int can_busload(int handle, uint8_t *load, uint8_t *status)
{
    float busLoad = 0.0;                // bus-load (in [percent])
    int rc;                             // return value

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;

    if (!can[handle].status.can_stopped) { // if running get bus load
        (void)busLoad; //  TODO: measure bus load
    }
    if (load)                           // bus-load (in [percent])
        *load = (uint8_t)busLoad;
    // get status-register from device
    rc = can_status(handle, status);
#if (OPTION_CANAPI_RETVALS == OPTION_DISABLED)
    if (rc == CANERR_NOERROR)
        rc = !can[handle].status.can_stopped ? CANERR_NOERROR : CANERR_OFFLINE;
#else
    // note: can_busload shall return CANERR_NOERROR if
    //       the CAN controller has not been started
#endif
    return rc;
}

int can_bitrate(int handle, can_bitrate_t *bitrate, can_speed_t *speed)
{
    can_bitrate_t tmpBitrate;           // bit-rate settings
    btr_nominal_t nominal;              // nominal bit-rate
    btr_data_t data;                    // data bit-rate
    canStatus sts;                      // return value

    memset(&tmpBitrate, 0, sizeof(can_bitrate_t));
    memset(&nominal, 0, sizeof(btr_nominal_t));
    memset(&data, 0, sizeof(btr_data_t));

    if (!init)                          // must be initialized
        return CANERR_NOTINIT;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return CANERR_HANDLE;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;

    /* bit-rate for CAN 2.0 & CAN FD */
    if ((sts = canGetBusParams(can[handle].handle, &nominal.bitRate, &nominal.tseg1, &nominal.tseg2,
                                                   &nominal.sjw, &nominal.noSamp, &nominal.syncmode)) == canOK)
        (void)map_params2bitrate(&nominal, can[handle].frequency, &tmpBitrate);
    else
        return kvaser_error(sts);
    /* bit-rate for CAN FD data (BRSE) */
    if (can[handle].mode.fdoe && can[handle].mode.brse) {
        if ((sts = canGetBusParamsFd(can[handle].handle, &data.bitRate, &data.tseg1, &data.tseg2, &data.sjw)) == canOK)
            (void)map_paramsFd2bitrate(&data, can[handle].frequency, &tmpBitrate);
        else
            return kvaser_error(sts);
    }
    /* note: 'bitrate' as well as 'speed' are optional */
    if (bitrate)
        memcpy(bitrate, &tmpBitrate, sizeof(can_bitrate_t));
    if (speed)
        (void)btr_bitrate2speed(&tmpBitrate, speed);
#if (OPTION_CANAPI_RETVALS == OPTION_DISABLED)
    return !can[handle].status.can_stopped ? CANERR_NOERROR : CANERR_OFFLINE;
#else
    // note: can_bitrate shall return CANERR_NOERROR if
    //       the CAN controller has not been started
    return CANERR_NOERROR;
#endif
}

int can_property(int handle, uint16_t param, void *value, uint32_t nbyte)
{
    if (!init || !IS_HANDLE_VALID(handle)) {
        // note: library properties can be queried w/o a handle
        return lib_parameter(param, value, (size_t)nbyte);
    }
    // note: library is initialized and handle is valid

    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return CANERR_HANDLE;
    // note: device properties must be queried with a valid handle
    return drv_parameter(handle, param, value, (size_t)nbyte);
}

char *can_hardware(int handle)
{
    static char hardware[CANPROP_MAX_BUFFER_SIZE+1] = "";
    char str[CANPROP_MAX_BUFFER_SIZE+1] = "";
    uint64_t rev = 0ull;

    if (!init)                          // must be initialized
        return NULL;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return NULL;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return NULL;

    // get hardware name and revision from CAN channel
    if (canGetChannelData(can[handle].channel, canCHANNELDATA_CHANNEL_NAME, (void*)str, CANPROP_MAX_BUFFER_SIZE) != canOK)
        return NULL;
    if (canGetChannelData(can[handle].channel, canCHANNELDATA_CARD_HARDWARE_REV, (void*)&rev, sizeof(uint64_t)) != canOK)
        return NULL;
    snprintf(hardware, CANPROP_MAX_BUFFER_SIZE, "%s, hardware revision %u.%u", str,
            (uint16_t)((rev & 0x00000000FFFF0000UL) >> 16),
            (uint16_t)((rev & 0x000000000000FFFFUL) >> 0));
    hardware[CANPROP_MAX_BUFFER_SIZE] = '\0';
    return (char*)hardware;
}

char *can_firmware(int handle)
{
    static char firmware[CANPROP_MAX_BUFFER_SIZE+1] = "";
    char str[CANPROP_MAX_BUFFER_SIZE+1] = "";
    uint64_t rev = 0ull;

    if (!init)                          // must be initialized
        return NULL;
    if (!IS_HANDLE_VALID(handle))       // must be a valid handle
        return NULL;
    if (!IS_HANDLE_OPENED(handle))      // must be an open handle
        return NULL;

    // get firmware name and revision from CAN channel
    if (canGetChannelData(can[handle].channel, canCHANNELDATA_CHANNEL_NAME, (void*)str, CANPROP_MAX_BUFFER_SIZE) != canOK)
        return NULL;
    if (canGetChannelData(can[handle].channel, canCHANNELDATA_CARD_FIRMWARE_REV, (void*)&rev, sizeof(uint64_t)) != canOK)
        return NULL;
    snprintf(firmware, CANPROP_MAX_BUFFER_SIZE, "%s, firmware version %u.%u.%u", str,
            (uint16_t)((rev & 0xFFFF000000000000UL) >> 48),
            (uint16_t)((rev & 0x0000FFFF00000000UL) >> 32),
            (uint16_t)((rev & 0x00000000FFFF0000UL) >> 16));
    firmware[CANPROP_MAX_BUFFER_SIZE] = '\0';
    return (char*)firmware;
}

/*  -----------  local functions  ----------------------------------------
 */
static void var_init(void)
{
    int i;

    for (i = 0; i < CAN_MAX_HANDLES; i++) {
        can[i].handle = canINVALID_HANDLE;
        can[i].channel = KVASER_CHANNEL_DEFAULT;
        can[i].frequency = KVASER_FREQ_DEFAULT;
        can[i].mode.byte = CANMODE_DEFAULT;
        can[i].status.byte = CANSTAT_RESET;
        can[i].filter.std.code = FILTER_STD_CODE;
        can[i].filter.std.mask = FILTER_STD_MASK;
        can[i].filter.xtd.code = FILTER_XTD_CODE;
        can[i].filter.xtd.mask = FILTER_XTD_MASK;
        can[i].error.lec = 0x00u;
        can[i].error.rx_err = 0u;
        can[i].error.tx_err = 0u;
        can[i].counters.tx = 0ull;
        can[i].counters.rx = 0ull;
        can[i].counters.err = 0ull;
    }
}

static int all_closed(void)
{
    int handle;

    if (!init)
        return 1;
    for (handle = 0; handle < CAN_MAX_HANDLES; handle++) {
        if (IS_HANDLE_OPENED(handle))
            return 0;
    }
    return 1;
}

static void can_message(long id, unsigned int flags, unsigned int len, const unsigned char *data, can_message_t *msg)
{
    assert(msg);
    assert(data);
    msg->id = (int32_t)id;
    msg->xtd = (flags & canMSG_EXT) ? 1 : 0;
    msg->rtr = (flags & canMSG_RTR) ? 1 : 0;
    msg->fdf = (flags & canFDMSG_FDF) ? 1 : 0;
    msg->brs = (flags & canFDMSG_BRS) ? 1 : 0;
    msg->esi = (flags & canFDMSG_ESI) ? 1 : 0;
    msg->sts = 0;
    msg->dlc = (uint8_t)LEN2DLC(len);
    memcpy(msg->data, data, CANFD_MAX_LEN);
}

static void can_message_sts(can_status_t status, can_error_t error, can_message_t *msg)
{
    assert(msg);
    msg->id = (int32_t)0;
    msg->xtd = 0;
    msg->rtr = 0;
    msg->fdf = 0;
    msg->brs = 0;
    msg->esi = 0;
    msg->sts = 1;
    msg->dlc = (uint8_t)4;
    msg->data[0] = (uint8_t)status.byte;
    msg->data[1] = (uint8_t)error.lec;
    msg->data[2] = (uint8_t)error.rx_err;
    msg->data[4] = (uint8_t)error.tx_err;
}

static void can_timestamp(unsigned long timestamp, can_message_t *msg)
{
    assert(msg);
    msg->timestamp.tv_sec = (time_t)(timestamp / 1000ul);
    msg->timestamp.tv_nsec = (long)(timestamp % 1000ul) * (long)1000000;
}

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
    canStatus sts;                      // return value

    if ((sts = canGetChannelData(channel, canCHANNELDATA_CHANNEL_CAP,
                                (void*)&feature, sizeof(feature))) != canOK)
        return sts;

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

static canStatus kvaser_get_filter(int handle, uint64_t *filter, int xtd)
{
    assert(filter);

    if (!xtd)
        *filter = ((uint64_t)can[handle].filter.std.code << 32)
                | ((uint64_t)can[handle].filter.std.mask);
    else
        *filter = ((uint64_t)can[handle].filter.xtd.code << 32)
                | ((uint64_t)can[handle].filter.xtd.mask);

    return canOK;
}

static canStatus kvaser_set_filter(int handle, uint64_t filter, int xtd)
{
    canStatus sts;                      // return value

    /* the acceptance code and mask are coded together in a 64-bit value, each of them using 4 bytes
     * the acceptance code is in the most significant bytes, the mask in the least significant bytes
     */
    unsigned int code = (unsigned int)((filter >> 32)) & (unsigned int)(xtd ? CAN_MAX_XTD_ID : CAN_MAX_STD_ID);
    unsigned int mask = (unsigned int)((filter >>  0)) & (unsigned int)(xtd ? CAN_MAX_XTD_ID : CAN_MAX_STD_ID);

    /* set the acceptance filter */
    if ((sts = canSetAcceptanceFilter(handle, code, mask, xtd)) != canOK)
        return sts;

    /* store the acceptance filter values (they cannot be read from the channel) */
    if (!xtd) {
        can[handle].filter.std.code = code;
        can[handle].filter.std.mask = mask;
    } else {
        can[handle].filter.xtd.code = code;
        can[handle].filter.xtd.mask = mask;
    }
    return canOK;
}

static canStatus kvaser_reset_filter(int handle)
{
    canStatus sts;                      // return value

    /* reset the acceptance filter for extended identifier */
    if ((sts = canSetAcceptanceFilter(handle, FILTER_XTD_CODE, FILTER_XTD_MASK, 1)) != canOK)
        return sts;
    can[handle].filter.xtd.code = FILTER_XTD_CODE;
    can[handle].filter.xtd.mask = FILTER_XTD_MASK;

    /* reset the acceptance filter for standard identifier */
    if ((sts = canSetAcceptanceFilter(handle, FILTER_STD_CODE, FILTER_STD_MASK, 0)) != canOK)
        return sts;
    can[handle].filter.std.code = FILTER_STD_CODE;
    can[handle].filter.std.mask = FILTER_STD_MASK;

    return canOK;
}

static int map_index2params(int index, btr_nominal_t *busParams)
{
    switch (index) {
    case CANBTR_INDEX_1M: KVASER_BDR_1000((*busParams)); break;
#if (OPTION_KVASER_BIT_TIMING != OPTION_DISABLED)
    case CANBTR_INDEX_800K: return CANERR_BAUDRATE;
#else
    case CANBTR_INDEX_800K: KVASER_BDR_800((*busParams)); break;
#endif
    case CANBTR_INDEX_500K: KVASER_BDR_500((*busParams)); break;
    case CANBTR_INDEX_250K: KVASER_BDR_250((*busParams)); break;
    case CANBTR_INDEX_125K: KVASER_BDR_125((*busParams)); break;
    case CANBTR_INDEX_100K: KVASER_BDR_100((*busParams)); break;
    case CANBTR_INDEX_50K: KVASER_BDR_50((*busParams)); break;
    case CANBTR_INDEX_20K: KVASER_BDR_20((*busParams)); break;
    case CANBTR_INDEX_10K: KVASER_BDR_10((*busParams)); break;
    default: return CANERR_BAUDRATE;
    }
    return CANERR_NOERROR;
}

static int map_bitrate2params(const can_bitrate_t *bitrate, btr_nominal_t *busParams)
{
    // sanity check
    if (!bitrate || !busParams)
        return CANERR_NULLPTR;
    if ((bitrate->btr.nominal.brp < CANBTR_NOMINAL_BRP_MIN) || (CANBTR_NOMINAL_BRP_MAX < bitrate->btr.nominal.brp))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.tseg1 < CANBTR_NOMINAL_TSEG1_MIN) || (CANBTR_NOMINAL_TSEG1_MAX < bitrate->btr.nominal.tseg1))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.tseg2 < CANBTR_NOMINAL_TSEG2_MIN) || (CANBTR_NOMINAL_TSEG2_MAX < bitrate->btr.nominal.tseg2))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.sjw < CANBTR_NOMINAL_SJW_MIN) || (CANBTR_NOMINAL_SJW_MAX < bitrate->btr.nominal.sjw))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.nominal.sam != CANBTR_NOMINAL_SAM_SINGLE) && (CANBTR_NOMINAL_SAM_TRIPLE != bitrate->btr.nominal.sam))
        return CANERR_BAUDRATE;
    if (bitrate->btr.nominal.brp == 0)   // devide-by-zero!
        return CANERR_BAUDRATE;

    /* bit-rate = frequency / (brp * (1 + tseg1 + tseg2)) */
    busParams->bitRate = (long)bitrate->btr.frequency
                       / ((long)bitrate->btr.nominal.brp * (1l + (long)bitrate->btr.nominal.tseg1 + (long)bitrate->btr.nominal.tseg2));
    busParams->tseg1 = (unsigned int)bitrate->btr.nominal.tseg1;
    busParams->tseg2 = (unsigned int)bitrate->btr.nominal.tseg2;
    busParams->sjw = (unsigned int)bitrate->btr.nominal.sjw;
    busParams->noSamp = (unsigned int)((bitrate->btr.nominal.sam != 0) ? 3 : 1);  // SJA1000: single or triple sampling
    busParams->syncmode = (unsigned int)0;

    return CANERR_NOERROR;
}

static int map_params2bitrate(const btr_nominal_t *busParams, long canClock, can_bitrate_t *bitrate)
{
    // sanity check
    if (!busParams || !bitrate)
        return CANERR_NULLPTR;

    /* Kvaser canLin32 doesn't offer the used controller frequency and bit-rate prescaler.
     * We suppose it's running with 80MHz and calculate the bit-rate prescaler as follows:
     *
     * (1) brp = frequency / (bit-rate * (1- tseg1 + tseq2))
     */
    if (busParams->bitRate == 0)   // divide-by-zero!
        return CANERR_BAUDRATE;
    bitrate->btr.frequency = (int32_t)canClock;
    bitrate->btr.nominal.brp = (uint16_t)(bitrate->btr.frequency
                             / ((int32_t)busParams->bitRate * (int32_t)(1u + busParams->tseg1 + busParams->tseg2)));
    bitrate->btr.nominal.tseg1 = (uint16_t)busParams->tseg1;
    bitrate->btr.nominal.tseg2 = (uint16_t)busParams->tseg2;
    bitrate->btr.nominal.sjw = (uint16_t)busParams->sjw;
    bitrate->btr.nominal.sam = (uint8_t)((busParams->noSamp < 3) ? 0 : 1);  // SJA1000: single or triple sampling

    return CANERR_NOERROR;
}

static int map_bitrate2paramsFd(const can_bitrate_t *bitrate, btr_data_t *busParams)
{
    // sanity check
    if (!bitrate || !busParams)
        return CANERR_NULLPTR;
    if ((bitrate->btr.data.brp < CANBTR_DATA_BRP_MIN) || (CANBTR_DATA_BRP_MAX < bitrate->btr.data.brp))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.data.tseg1 < CANBTR_DATA_TSEG1_MIN) || (CANBTR_DATA_TSEG1_MAX < bitrate->btr.data.tseg1))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.data.tseg2 < CANBTR_DATA_TSEG2_MIN) || (CANBTR_DATA_TSEG2_MAX < bitrate->btr.data.tseg2))
        return CANERR_BAUDRATE;
    if ((bitrate->btr.data.sjw < CANBTR_DATA_SJW_MIN) || (CANBTR_DATA_SJW_MAX < bitrate->btr.data.sjw))
        return CANERR_BAUDRATE;
    if (bitrate->btr.data.brp == 0)  // divide-by-zero!
        return CANERR_BAUDRATE;

    /* data phase: bit-rate = frequency / (brp * (1 + tseg1 + tseg2)) */
    busParams->bitRate = (long)bitrate->btr.frequency
                       / ((long)bitrate->btr.data.brp * (1l + (long)bitrate->btr.data.tseg1 + (long)bitrate->btr.data.tseg2));
    busParams->tseg1 = (unsigned int)bitrate->btr.data.tseg1;
    busParams->tseg2 = (unsigned int)bitrate->btr.data.tseg2;
    busParams->sjw = (unsigned int)bitrate->btr.data.sjw;

    return CANERR_NOERROR;
}

static int map_paramsFd2bitrate(const btr_data_t *busParams, long canClock, can_bitrate_t *bitrate)
{
    // sanity check
    if (!busParams || !bitrate)
        return CANERR_NULLPTR;

    /* Kvaser canLin32 doesn't offer the used controller frequency and bit-rate prescaler.
     * We suppose it's running with 80MHz and calculate the bit-rate prescaler as follows:
     *
     * (1) brp = frequency / bit-rate * ((1- tseg1 + tseq2))
     */
    if (busParams->bitRate == 0)  // divide-by-zero!
        return CANERR_BAUDRATE;
    bitrate->btr.data.brp = (int32_t)(canClock
                          / ((long)busParams->bitRate * (long)(1u + busParams->tseg1 + busParams->tseg2)));
    bitrate->btr.data.tseg1 = (uint16_t)busParams->tseg1;
    bitrate->btr.data.tseg2 = (uint16_t)busParams->tseg2;
    bitrate->btr.data.sjw = (uint16_t)busParams->sjw;

    return CANERR_NOERROR;
}

/*  - - - - - -  CAN API V3 properties  - - - - - - - - - - - - - - - - -
 */
static int lib_parameter(uint16_t param, void *value, size_t nbyte)
{
    int rc = CANERR_ILLPARA;            // suppose an invalid parameter
    static int idx_board = EOF;         // actual index in the interface list

    if (value == NULL) {                // check for null-pointer
        if ((param != CANPROP_SET_FIRST_CHANNEL) &&
            (param != CANPROP_SET_NEXT_CHANNEL) &&
            (param != CANPROP_SET_FILTER_RESET))
            return CANERR_NULLPTR;
    }
    // query or modify a CAN library properties
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
            *(int32_t*)value = (int32_t)LIB_ID;
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_VENDOR:    // vendor name of the library (char[])
        if (nbyte >= 1u) {
            strncpy((char*)value, CAN_API_VENDOR, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_LIBRARY_DLLNAME:   // file name of the library (char[])
        if (nbyte >= 1u) {
            strncpy((char*)value, LIB_DLLNAME, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_VENDOR:     // vendor name of the CAN interface (char[])
        if (nbyte >= 1u) {
            strncpy((char*)value, DEV_VENDOR, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_DLLNAME:    // file name of the CAN interface DLL (char[])
        if (nbyte >= 1u) {
            strncpy((char*)value, DEV_DLLNAME, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_SET_FIRST_CHANNEL:     // set index to the first entry in the interface list (NULL)
        idx_board = 0;
        rc = (can_boards[idx_board].type != EOF) ? CANERR_NOERROR : CANERR_RESOURCE;
        break;
    case CANPROP_SET_NEXT_CHANNEL:      // set index to the next entry in the interface list (NULL)
        if ((0 <= idx_board) && (idx_board < NUM_CHANNELS)) {
            if (can_boards[idx_board].type != EOF)
                idx_board++;
            rc = (can_boards[idx_board].type != EOF) ? CANERR_NOERROR : CANERR_RESOURCE;
        }
        else
            rc = CANERR_RESOURCE;
        break;
    case CANPROP_GET_CHANNEL_NO:        // get channel no. at actual index in the interface list (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                *(int32_t*)value = (int32_t)can_boards[idx_board].type;
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_NAME:      // get channel name at actual index in the interface list (char[])
        if (nbyte >= 1u) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, can_boards[idx_board].name, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_DLLNAME:   // get file name of the DLL at actual index in the interface list (char[])
        if (nbyte >= 1u) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, DEV_DLLNAME, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_VENDOR_ID: // get library id at actual index in the interface list (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                *(int32_t*)value = (int32_t)LIB_ID;
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_CHANNEL_VENDOR_NAME: // get vendor name at actual index in the interface list (char[])
        if (nbyte >= 1u) {
            if ((0 <= idx_board) && (idx_board < NUM_CHANNELS) &&
                (can_boards[idx_board].type != EOF)) {
                strncpy((char*)value, DEV_VENDOR, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            }
            else
                rc = CANERR_RESOURCE;
        }
        break;
    case CANPROP_GET_DEVICE_TYPE:       // device type of the CAN interface (int32_t)
    case CANPROP_GET_DEVICE_NAME:       // device name of the CAN interface (char[])
    case CANPROP_GET_OP_CAPABILITY:     // supported operation modes of the CAN controller (uint8_t)
    case CANPROP_GET_OP_MODE:           // active operation mode of the CAN controller (uint8_t)
    case CANPROP_GET_BITRATE:           // active bit-rate of the CAN controller (can_bitrate_t)
    case CANPROP_GET_SPEED:             // active bus speed of the CAN controller (can_speed_t)
    case CANPROP_GET_STATUS:            // current status register of the CAN controller (uint8_t)
    case CANPROP_GET_BUSLOAD:           // current bus load of the CAN controller (uint16_t)
    case CANPROP_GET_NUM_CHANNELS:      // numbers of CAN channels on the CAN interface (uint8_t)
    case CANPROP_GET_CAN_CHANNEL:       // active CAN channel on the CAN interface (uint8_t)
    case CANPROP_GET_CAN_CLOCK:         // frequency of the CAN controller clock in [Hz] (int32_t)
    case CANPROP_GET_TX_COUNTER:        // total number of sent messages (uint64_t)
    case CANPROP_GET_RX_COUNTER:        // total number of reveiced messages (uint64_t)
    case CANPROP_GET_ERR_COUNTER:       // total number of reveiced error frames (uint64_t)
    case CANPROP_GET_RCV_QUEUE_SIZE:    // maximum number of message the receive queue can hold (uint32_t)
    case CANPROP_GET_RCV_QUEUE_HIGH:    // maximum number of message the receive queue has hold (uint32_t)
    case CANPROP_GET_RCV_QUEUE_OVFL:    // overflow counter of the receive queue (uint64_t)
    case CANPROP_GET_FILTER_11BIT:      // acceptance filter code and mask for 11-bit identifier (uint64_t)
    case CANPROP_GET_FILTER_29BIT:      // acceptance filter code and mask for 29-bit identifier (uint64_t)
    case CANPROP_SET_FILTER_11BIT:      // set value for acceptance filter code and mask for 11-bit identifier (uint64_t)
    case CANPROP_SET_FILTER_29BIT:      // set value for acceptance filter code and mask for 29-bit identifier (uint64_t)
    case CANPROP_SET_FILTER_RESET:      // reset acceptance filter code and mask to default values (NULL)
        // note: a device parameter requires a valid handle.
        if (!init)
            rc = CANERR_NOTINIT;
        else
            rc = CANERR_HANDLE;
        break;
    case KVASER_CANLIB_VERSION:         // version number of the CANlib DLL (uint16_t)
        if (nbyte >= sizeof(uint16_t)) {
            *(uint16_t*)value = (uint16_t)canGetVersionEx(canVERSION_CANLIB32_PRODVER);
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
    can_bitrate_t bitrate;              // bit-rate settings
    can_speed_t speed;                  // current bus speed
    can_mode_t mode;                    // current operation mode
    uint8_t status = 0u;                // status register
    uint8_t load = 0u;                  // bus load
    char str[CANPROP_MAX_BUFFER_SIZE+1];// device name
    canStatus sts;                      // status or error code

    assert(IS_HANDLE_VALID(handle));    // just to make sure

    if (value == NULL) {                // check for null-pointer
        if ((param != CANPROP_SET_FIRST_CHANNEL) &&
            (param != CANPROP_SET_NEXT_CHANNEL) &&
            (param != CANPROP_SET_FILTER_RESET))
            return CANERR_NULLPTR;
    }
    // query or modify a CAN interface property
    switch (param) {
    case CANPROP_GET_DEVICE_TYPE:       // device type of the CAN interface (int32_t)
        if (nbyte >= sizeof(int32_t)) {
            if ((sts = canGetChannelData(can[handle].channel, canCHANNELDATA_CARD_TYPE,
                                        (void*)value, (size_t)nbyte)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        break;
    case CANPROP_GET_DEVICE_NAME:       // device name of the CAN interface (char[])
        if (nbyte >= 1u) {
            if ((sts = canGetChannelData(can[handle].channel, canCHANNELDATA_DEVDESCR_ASCII,  // deprecated: canCHANNELDATA_CHANNEL_NAME
                                        (void*)str, (size_t)CANPROP_MAX_BUFFER_SIZE)) == canOK) {
                str[CANPROP_MAX_BUFFER_SIZE] = '\0';
                strncpy((char*)value, str, nbyte);
                ((char*)value)[(nbyte - 1)] = '\0';
                rc = CANERR_NOERROR;
            } else
                rc = kvaser_error(sts);
        }
        break;
    case CANPROP_GET_DEVICE_VENDOR:     // vendor name of the CAN interface (char[])
        if (nbyte >= 1u) {
            strncpy((char*)value, DEV_VENDOR, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
        break;
    case CANPROP_GET_DEVICE_DLLNAME:    // file name of the CAN interface DLL (char[])
        if (nbyte >= 1u) {
            strncpy((char*)value, DEV_DLLNAME, nbyte);
            ((char*)value)[(nbyte - 1)] = '\0';
            rc = CANERR_NOERROR;
        }
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
    case CANPROP_GET_BUSLOAD:           // current bus load of the CAN controller (uint16_t)
        if (nbyte >= sizeof(uint8_t)) {
            if (((rc = can_busload(handle, &load, NULL)) == CANERR_NOERROR) || (rc == CANERR_OFFLINE)) {
                if (nbyte > sizeof(uint8_t))
                    *(uint16_t*)value = (uint16_t)load * 100u;  // 0..10000 ==> 0.00%..100.00%
                else
                    *(uint8_t*)value = (uint8_t)load;           // 0..100% (note: legacy resolution)
                rc = CANERR_NOERROR;
            }
        }
        break;
    case CANPROP_GET_NUM_CHANNELS:      // numbers of CAN channels on the CAN interface (uint8_t)
    case CANPROP_GET_CAN_CHANNEL:       // active CAN channel on the CAN interface (uint8_t)
    case CANPROP_GET_CAN_CLOCK:         // frequency of the CAN controller clock in [Hz] (int32_t)
        // TODO: insert coin here
        rc = CANERR_NOTSUPP;
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
    case CANPROP_GET_RCV_QUEUE_SIZE:    // maximum number of message the receive queue can hold (uint32_t)
    case CANPROP_GET_RCV_QUEUE_HIGH:    // maximum number of message the receive queue has hold (uint32_t)
    case CANPROP_GET_RCV_QUEUE_OVFL:    // overflow counter of the receive queue (uint64_t)
        // note: cannot be determined
        rc = CANERR_NOTSUPP;
        break;
    case CANPROP_GET_FILTER_11BIT:      // acceptance filter code and mask for 11-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if ((sts = kvaser_get_filter(handle, (uint64_t*)value, 0)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        break;
    case CANPROP_GET_FILTER_29BIT:      // acceptance filter code and mask for 29-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if ((sts = kvaser_get_filter(handle, (uint64_t*)value, 1)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        break;
    case CANPROP_SET_FILTER_11BIT:      // set value for acceptance filter code and mask for 11-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if (!(*(uint64_t*)value & ~FILTER_STD_VALID_MASK)) {
                // note: code and mask must not exceed 11-bit identifier
                if (can[handle].status.can_stopped) {
                    // note: set filter only if the CAN controller is in INIT mode
                    if ((sts = kvaser_set_filter(handle, *(uint64_t*)value, 0)) == canOK)
                        rc = CANERR_NOERROR;
                    else
                        rc = kvaser_error(sts);
                }
                else
                    rc = CANERR_ONLINE;
            }
            else
                rc = CANERR_ILLPARA;
        }
        break;
    case CANPROP_SET_FILTER_29BIT:      // set value for acceptance filter code and mask for 29-bit identifier (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if (!(*(uint64_t*)value & ~FILTER_XTD_VALID_MASK) && !can[handle].mode.nxtd) {
                // note: code and mask must not exceed 29-bit identifier and
                //       extended frame format mode must not be suppressed
                if (can[handle].status.can_stopped) {
                    // note: set filter only if the CAN controller is in INIT mode
                    if ((sts = kvaser_set_filter(handle, *(uint64_t*)value, 1)) == canOK)
                        rc = CANERR_NOERROR;
                    else
                        rc = kvaser_error(sts);
                }
                else
                    rc = CANERR_ONLINE;
            }
            else
                rc = CANERR_ILLPARA;
        }
        break;
    case CANPROP_SET_FILTER_RESET:      // reset acceptance filter code and mask to default values (NULL)
        if (can[handle].status.can_stopped) {
            // note: reset filter only if the CAN controller is in INIT mode
            if ((sts = kvaser_reset_filter(handle)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        else
            rc = CANERR_ONLINE;
        break;
    case KVASER_DRIVER_VERSION:         // Kvaser's version number of the driver (uint64_t)
        if (nbyte >= sizeof(uint64_t)) {
            if ((sts = canGetChannelData(can[handle].channel, canCHANNELDATA_DRIVER_PRODUCT_VERSION,
                                        (void*)value, (size_t)nbyte)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
        }
        break;
    case KVASER_DRIVER_NAME:            // Kvaser's name of the driver (char[256])
        if ((nbyte > 0U) && (nbyte <= CANPROP_MAX_BUFFER_SIZE)) {
            if ((sts = canGetChannelData(can[handle].channel, canCHANNELDATA_DRIVER_NAME,
                                        (void*)value, (size_t)nbyte)) == canOK)
                rc = CANERR_NOERROR;
            else
                rc = kvaser_error(sts);
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

/*  -----------  revision control  ---------------------------------------
 */

char *can_version(void)
{
    return (char*)version;
}
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de, Homepage: https://www.uv-software.de/
 */
