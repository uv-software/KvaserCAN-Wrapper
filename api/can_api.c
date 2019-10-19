/*  -- $HeadURL$ --
 *
 *  project   :  CAN - Controller Area Network
 *
 *  purpose   :  CAN Interface API, Version 3 (Kvaser canlib32)
 *
 *  copyright :  (C) 2017-2019, UV Software, Berlin
 *
 *  compiler  :  Microsoft Visual C/C++ Compiler (Version 19.16)
 *
 *  export    :  (see header file)
 *
 *  includes  :  can_api.h (can_defs.h), can_vers.h, canlib.h, canstat.h
 *
 *  author    :  Uwe Vogt, UV Software
 *
 *  e-mail    :  uwe.vogt@uv-software.de
 *
 *
 *  -----------  description  --------------------------------------------
 */
/** @file        can_api.c
 *
 *  @brief       CAN API V3 for Kvaser CAN Interfaces - API
 *
 *  @author      $Author$
 *
 *  @version     $Rev$
 *
 *  @addtogroup  can_api
 *  @{
 */

#define VERSION_MAJOR     1
#define VERSION_MINOR     1
#define VERSION_REVISION  0
#define VERSION_STRING    TOSTRING(VERSION_MAJOR)"." TOSTRING(VERSION_MINOR)"."TOSTRING(VERSION_REVISION)
#if defined(_WIN64)
#define PLATFORM    "x64"
#elif defined(_WIN32)
#define PLATFORM    "x86"
#else
#error Unsupported architecture
#endif
#include "can_vers.h"
#ifdef _DEBUG
static char _id[] = "CAN API V3 for Kvaser CAN Interfaces, Version "VERSION_STRING"-"TOSTRING(BUILD_NO)" ("PLATFORM") _DEBUG";
#else
static char _id[] = "CAN API V3 for Kvaser CAN Interfaces, Version "VERSION_STRING"-"TOSTRING(BUILD_NO)" ("PLATFORM")";
#endif

/*  -----------  includes  -----------------------------------------------
 */

#include "can_api.h"

#include <stdio.h>

#include "canlib.h"
#include "canstat.h"


 /*  -----------  options  ------------------------------------------------
  */

#define _BLOCKING_READ                  // blocking read via wait event
#define _OPEN_EXCLUSIVE                 // permit only exclusive access
#define _VIRTUAL_CHANNELS               // support of virtual channels
//#define _SIMULATED_CHANNELS             // support of simulated channels


/*  -----------  defines  ------------------------------------------------
 */

#ifndef KVASER_MAX_HANDLES
#define KVASER_MAX_HANDLES      (8)     // maximum number of open handles
#endif
#define INVALID_HANDLE          (-1)
#define IS_HANDLE_VALID(hnd)    ((0 <= (hnd)) && ((hnd) < KVASER_MAX_HANDLES))
#ifndef DLC2LEN
#define DLC2LEN(x)              dlc_table[x & 0xF]
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

typedef union {
    BYTE byte;                          // byte access
    struct {                            // bit access:
        BYTE mon : 1;                   //   monitor mode enable/disable
        BYTE : 3;                       //   (reserved)
        BYTE niso : 1;                  //   Non-ISO CAN FD enable
        BYTE : 1;                       //   (reserved)
        BYTE brse : 1;                  //   bit-rate switch enable
        BYTE fdoe : 1;                  //   CAN FD operation enable
    }   b;
}   can_mode_t;

typedef struct {
    CanHandle  handle;                  // hardware channel handle (Kvaser)
    int        channel;                 // channel number ot the CAN board
    can_mode_t mode;                    // operation mode of the CAN channel
    can_status_t status;                // 8-bit status register
    can_bitrate_t bitrate;              // bit-rate setting 
}   can_interface_t;


/*  -----------  prototypes  ---------------------------------------------
 */

static int kvaser_error(canStatus);    // Kvaser specific errors


/*  -----------  variables  ----------------------------------------------
 */

#ifdef _CANAPI_EXPORTS
#define ATTRIB  __declspec(dllexport)
#else
#define ATTRIB
#endif
ATTRIB can_board_t can_board[KVASER_BOARDS]=// list of CAN Interface boards:
{
    {KVASER_CAN_CHANNEL0,                 "Kvaser CAN Channel 0"},
    {KVASER_CAN_CHANNEL1,                 "Kvaser CAN Channel 1"},
    {KVASER_CAN_CHANNEL2,                 "Kvaser CAN Channel 2"},
    {KVASER_CAN_CHANNEL3,                 "Kvaser CAN Channel 3"},
    {KVASER_CAN_CHANNEL4,                 "Kvaser CAN Channel 4"},
    {KVASER_CAN_CHANNEL5,                 "Kvaser CAN Channel 5"},
    {KVASER_CAN_CHANNEL6,                 "Kvaser CAN Channel 6"},
    {KVASER_CAN_CHANNEL7,                 "Kvaser CAN Channel 7"}
};
static const BYTE dlc_table[16] = {     // DLC to length
    0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64
};
static can_interface_t can[KVASER_MAX_HANDLES]; // interface handles 
static char hardware[256] = "";         // hardware version of the CAN interface board
static char software[256] = "";         // software version of the CAN interface driver
static int  init = 0;                   // initialization flag


/*  -----------  functions  ----------------------------------------------
 */

int can_test(int board, unsigned char mode, const void *param, int *result)
{
    int capacity = 0x0000;              // channel capacity
    int flags = 0x0000;                 // channel flags
    canStatus rc;                       // return value
    int i, n;

    if(!init) {                         // when not init before:
        for(i = 0; i < KVASER_MAX_HANDLES; i++) {
            can[i].handle = canINVALID_HANDLE;
            can[i].mode.byte = CANMODE_DEFAULT;
            can[i].status.byte = CANSTAT_RESET;
            can[i].bitrate.index = -CANBDR_250;
        }
        canInitializeLibrary();         //   initialize the driver
        init = 1;                       //   set initialization flag
    }
    if((rc = canGetNumberOfChannels(&n)) != canOK)
        return kvaser_error(rc);
    if(board >= n) {
        if(result)                      // CAN board not present
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
    if((rc = canGetChannelData(board, canCHANNELDATA_CHANNEL_CAP, 
                               (void*)&capacity, sizeof(capacity))) != canOK)
        return kvaser_error(rc);
#ifndef _VIRTUAL_CHANNELS
    if((capacity & canCHANNEL_CAP_VIRTUAL)) {
        if(result)                     // declare as not available
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
#endif
#ifndef _SIMULATED_CHANNELS
    if((capacity & canCHANNEL_CAP_SIMULATED)) {
        if(result)                     // declare as not available
            *result = CANBRD_NOT_PRESENT;
        return CANERR_NOERROR;
    }
#endif
    if((rc = canGetChannelData(board, canCHANNELDATA_CHANNEL_FLAGS,
                               (void*)&flags, sizeof(flags))) != canOK)
        return kvaser_error(rc);
    if(result) {
#ifdef _OPEN_EXCLUSIVE
        if(flags & canCHANNEL_IS_OPEN)
#else
        if(flags & canCHANNEL_IS_EXCLUSIVE)
#endif
            *result = CANBRD_OCCUPIED;  // CAN board occupied by another process
        else
            *result = CANBRD_PRESENT;   // CAN board present and available

        for(i = 0; i < KVASER_MAX_HANDLES; i++)
            if(can[i].channel == board) // CAN board occupied by ourself
                *result = CANBRD_OCCUPIED;
    }
    if((mode & CANMODE_FDOE) && !(capacity & canCHANNEL_CAP_CAN_FD))
        return CANERR_ILLPARA; // CAN FD operation requested, but not supported
    if((mode & CANMODE_BRSE) && !(mode & CANMODE_FDOE))
        return CANERR_ILLPARA; // bit-rate switching requested, but CAN FD not enabled
    if((mode & CANMODE_NISO) && !(capacity & canCHANNEL_CAP_CAN_FD_NONISO))
        return CANERR_ILLPARA; // Non-ISO operation requested, but not supported
    if((mode & CANMODE_NISO) && !(mode & CANMODE_FDOE))
        return CANERR_ILLPARA; // Non-ISO operation requested, but CAN FD not enabled
    if((mode & CANMODE_MON) && !(capacity & canCHANNEL_CAP_SILENT_MODE))
        return CANERR_ILLPARA; // listen-only mode requested, but not supported
    (void)param;
    return CANERR_NOERROR;
}

int can_init(int board, unsigned char mode, const void *param)
{
    int flags = 0x0000;                 // flags for canOpenChannel()
    canHandle result;                   // Kvaser handle or error
    canStatus rc;                       // return value
    int i;

    if(!init) {                         // when not init before:
        for(i = 0; i < KVASER_MAX_HANDLES; i++) {
            can[i].handle = canINVALID_HANDLE;
            can[i].mode.byte = CANMODE_DEFAULT;
            can[i].status.byte = CANSTAT_RESET;
            can[i].bitrate.index = -CANBDR_250;
        }
        canInitializeLibrary();         // initialize the driver
        init = 1;                       // set initialization flag
    }
    for(i = 0; i < KVASER_MAX_HANDLES; i++) {
        if((can[i].handle != canINVALID_HANDLE) && 
           (can[i].channel == board))   // channel already in use
            return CANERR_YETINIT;
    }
    for(i = 0; i < KVASER_MAX_HANDLES; i++) {
        if(can[i].handle == canINVALID_HANDLE)
            break;
    }
    if(!IS_HANDLE_VALID(i))             // no free handle found
        return CANERR_HANDLE;

#ifdef _OPEN_EXCLUSIVE
    flags |= canOPEN_EXCLUSIVE;
#endif
    flags |= (mode & CANMODE_FDOE)? canOPEN_CAN_FD : 0;
    flags |= (mode & CANMODE_FDOE)? canOPEN_ACCEPT_LARGE_DLC : 0;// why?
    flags |= (mode & CANMODE_NISO)? canOPEN_CAN_FD_NONISO : 0;
#ifdef _VIRTUAL_CHANNELS
    // TODO: flags |= (mode & CANMODE_VIRT) ? canOPEN_ACCEPT_VIRTUAL : 0;
#endif
    if((result = canOpenChannel(board, flags)) < canOK)
        return kvaser_error(result);

    if(!(mode & CANMODE_MON)) {         // normal operation
        if((rc = canSetBusOutputControl(result, canDRIVER_NORMAL)) != canOK)
            return kvaser_error(rc);
    }
    else {                              // listen-only
        if((rc = canSetBusOutputControl(result, canDRIVER_SILENT)) != canOK)
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
    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    /*if(!can[handle].status.b.can_stopped) // go to CAN INIT mode (bus off)*/
        (void)canBusOff(can[handle].handle);
    (void)canClose(can[handle].handle);     // release the CAN interface!

    can[handle].status.byte |= CANSTAT_RESET;// CAN controller in INIT state
    can[handle].handle = canINVALID_HANDLE; // handle can be used again

    return CANERR_NOERROR;
}

int can_start(int handle, const can_bitrate_t *bitrate)
{
    long slow_baud = 0;                 // bausrate in [bps] (nominal)
    unsigned int slow_tseg1 = 0;        // Time segement 1 (nominal)
    unsigned int slow_tseg2 = 0;        // Time segement 2 (nominal)
    unsigned int slow_sjw = 0;          // sync. jump width (nominal)
    unsigned int slow_sam = 0;          // no. of samples (nominal)
    unsigned int slow_sync = 0;         // unsupported (nominal)
    long fast_baud = 0;                 // bausrate in [bps] (data)
    unsigned int fast_tseg1 = 0;        // Time segement 1 (data)
    unsigned int fast_tseg2 = 0;        // Time segement 2 (data)
    unsigned int fast_sjw = 0;          // sync. jump width (data)
    canStatus rc;                       // return value

    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;
    if(bitrate == NULL)                 // check for null-pointer
        return CANERR_NULLPTR;
    if(!can[handle].status.b.can_stopped) // must be stopped
        return CANERR_ONLINE;

    if(bitrate->index <= 0) {           // bit-rate from index
        switch(bitrate->index * (-1)) {
        case CANBDR_1000: slow_baud = canBITRATE_1M; break;
        case CANBDR_800: return CANERR_BAUDRATE;
        case CANBDR_500: slow_baud = canBITRATE_500K; break;
        case CANBDR_250: slow_baud = canBITRATE_250K; break;
        case CANBDR_125: slow_baud = canBITRATE_125K; break;
        case CANBDR_100: slow_baud = canBITRATE_100K; break;
        case CANBDR_50: slow_baud = canBITRATE_50K; break;
        case CANBDR_20: slow_baud = 20000; slow_tseg1 = 11; slow_tseg2 = 4; slow_sjw = 1; slow_sam = 1; break;
        case CANBDR_10: slow_baud = canBITRATE_10K; break;
        default: return CANERR_BAUDRATE;
        }
        if((rc = canSetBusParams(can[handle].handle, slow_baud, slow_tseg1, slow_tseg2, slow_sjw, slow_sam, slow_sync)) != canOK)
            return kvaser_error(rc);
    }
    else {                              // bit-rate from parameter
        if(bitrate->btr.nominal.brp == 0)
            return CANERR_BAUDRATE;     //   divide-by-zero!

        /* nominal bit-rate = frequency / (brp * (1 + tseg1 + tseg2)) */
        slow_baud = bitrate->btr.frequency / (((long)bitrate->btr.nominal.brp) * (1l + (long)bitrate->btr.nominal.tseg1 + (long)bitrate->btr.nominal.tseg2));
        slow_tseg1 = bitrate->btr.nominal.tseg1;
        slow_tseg2 = bitrate->btr.nominal.tseg2;
        slow_sjw = bitrate->btr.nominal.sjw;
        slow_sam = bitrate->btr.nominal.sam;
        
        if((rc = canSetBusParams(can[handle].handle, slow_baud, slow_tseg1, slow_tseg2, slow_sjw, slow_sam, slow_sync)) != canOK)
            return kvaser_error(rc);
        /* bit-rate for CAN FD data */
        if(can[handle].mode.b.fdoe && can[handle].mode.b.brse) {
            if(bitrate->btr.nominal.brp == 0)
                return CANERR_BAUDRATE; //   divide-by-zero!

            /* data bit-rate = frequency / (brp * (1 + tseg1 + tseg2)) */
            fast_baud = bitrate->btr.frequency / (((long)bitrate->btr.data.brp) * (1l + (long)bitrate->btr.data.tseg1 + (long)bitrate->btr.data.tseg2));
            fast_tseg1 = bitrate->btr.data.tseg1;
            fast_tseg2 = bitrate->btr.data.tseg2;
            fast_sjw = bitrate->btr.data.sjw;
            
            if((rc = canSetBusParamsFd(can[handle].handle, fast_baud, fast_tseg1, fast_tseg2, fast_sjw)) != canOK)
                return kvaser_error(rc);
        }
    }
    if((rc = canBusOn(can[handle].handle)) != canOK)
        return kvaser_error(rc);
    
    memcpy(&can[handle].bitrate, bitrate, sizeof(can_bitrate_t));
    can[handle].status.byte = 0x00;     // clear old status bits
    can[handle].status.b.can_stopped = 0;// CAN controller started!

    return CANERR_NOERROR;
}

int can_kill(int handle)
{
    HANDLE hEvent = NULL;               // event object
    int i;

    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(handle != CANKILL_ALL) {
        if(!IS_HANDLE_VALID(handle))    // must be a valid handle
            return CANERR_HANDLE;
#ifdef _BLOCKING_READ
        if(can[handle].handle != canINVALID_HANDLE) {
            if((canIoCtl(can[handle].handle, canIOCTL_GET_EVENTHANDLE, 
                         (void*)&hEvent, sizeof(hEvent)) == canOK) &&
               (hEvent != NULL)) {
                (void)SetEvent(hEvent);     // signal the event object
            }
        }
    }
    else {
        for(i = 0; i < KVASER_MAX_HANDLES; i++) {
            if(can[i].handle != canINVALID_HANDLE) {
                if((canIoCtl(can[i].handle, canIOCTL_GET_EVENTHANDLE,
                    (void*)&hEvent, sizeof(hEvent)) == canOK) &&
                   (hEvent != NULL)) {
                    (void)SetEvent(hEvent); // signal all event objects
                }
            }
        }
#endif
    }
    return CANERR_NOERROR;
}

int can_reset(int handle)
{
    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if(can[handle].status.b.can_stopped) {  // CAN started, then reset
        (void)canBusOff(can[handle].handle);
    }
    can[handle].status.b.can_stopped = 1;   // CAN controller stopped!

    return CANERR_NOERROR;
}

int can_write(int handle, const can_msg_t *msg)
{
    long id;                            // the message:
    unsigned int dlc, flags;
    unsigned char data[CANFD_MAX_LEN];
    canStatus rc;                       // return value

    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;
    if(msg == NULL)                     // check for null-pointer
        return CANERR_NULLPTR;
    if(can[handle].status.b.can_stopped)// must be running!
        return CANERR_OFFLINE;

    if(!can[handle].mode.b.fdoe) {
        if(msg->dlc > CAN_MAX_LEN)      //   data length 0 .. 8!
            return CANERR_ILLPARA;
        if(msg->ext)                    //   29-bit identifier
            flags = canMSG_EXT;
        else                            //   11-bit identifier
            flags = canMSG_STD;
        if(msg->rtr)                    //   request a message
            flags |= canMSG_RTR;
        id = (long)(msg->id);
        dlc = (unsigned int)(msg->dlc);
        memcpy(data, msg->data, msg->dlc);
    }
    else {
        if(msg->dlc > CANFD_MAX_LEN)    //   data length 0 .. 64!
            return CANERR_ILLPARA;
        if(msg->ext)                    //   29-bit identifier
            flags = canMSG_EXT;
        else                            //   11-bit identifier
            flags = canMSG_STD;
        if(msg->rtr)                    //   request a message
            flags |= canMSG_RTR;
        if(msg->fdf)                    //   CAN FD format
            flags |= canFDMSG_FDF;
        if(msg->brs && can[handle].mode.b.brse) //   bit-rate switching
            flags |= canFDMSG_BRS;
        id = (long)(msg->id);
        dlc = (unsigned int)(DLC2LEN(msg->dlc));
        memcpy(data, msg->data, DLC2LEN(msg->dlc));
    }
    if((rc = canWriteWait(can[handle].handle, id, &data, dlc, flags, KVASER_TRM_TIEMOUT)) != canOK)
    {
        if((rc & canERR_TXBUFOFL)) {    //   transmit queue full?
            can[handle].status.b.transmitter_busy = 1;
            return CANERR_TX_BUSY;      //     transmitter busy
        }
        else if((rc & canERR_TIMEOUT)) {//   timed out?
            can[handle].status.b.transmitter_busy = 1;
            return CANERR_TX_BUSY;      //     transmitter busy
        }
        return kvaser_error(rc);        //   Kvaser specific error?
    }
    can[handle].status.b.transmitter_busy = 0;  // message transmitted

    return CANERR_NOERROR;
}

int can_read(int handle, can_msg_t *msg, unsigned short timeout)
{
    long id;                            // the message:
    unsigned int len, flags;
    unsigned char data[CANFD_MAX_LEN];
    unsigned long timestamp;            // time stamp (in [ms])
    canStatus rc;                       // return value

    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;
    if(msg == NULL)                     // check for null-pointer
        return CANERR_NULLPTR;
    if(can[handle].status.b.can_stopped)// must be running!
        return CANERR_OFFLINE;

    if((rc = canRead(can[handle].handle, &id, data, &len, &flags, &timestamp)) == canERR_NOMSG) {
#ifdef _BLOCKING_READ
        if(timeout > 0) {
            switch(canWaitForEvent(can[handle].handle, (timeout != CANREAD_INFINITE) ? timeout : INFINITE)) {
            case canOK:
                break;                  //   one or more messages received
            case canERR_TIMEOUT:
                break;                  //   time-out, but look for old messages
            default:
                return CANERR_FATAL;    //   function failed!
            }
            if((rc = canRead(can[handle].handle, &id, data, &len, &flags, &timestamp)) == canERR_NOMSG) {
                can[handle].status.b.receiver_empty = 1;
                return CANERR_RX_EMPTY; //   receiver empty
            }
        }
        else {
            can[handle].status.b.receiver_empty = 1;
            return CANERR_RX_EMPTY;     //   receiver empty
        }
#else
        can[handle].status.b.receiver_empty = 1;
        return CANERR_RX_EMPTY;         //   receiver empty
#endif
    }
    if(rc < canOK)                      // receive error?
    {
        return kvaser_error(rc);        //   something´s wrong
    }
    if((flags & canMSG_ERROR_FRAME)) {  // error frame?  
        can[handle].status.b.receiver_empty = 1;
        return CANERR_RX_EMPTY;         //   receiver empty
    }
    msg->id = id;
    msg->ext = (flags & canMSG_EXT)? 1 : 0;
    msg->rtr = (flags & canMSG_RTR)? 1 : 0;
    msg->fdf = (flags & canFDMSG_FDF)? 1 : 0;
    msg->brs = (flags & canFDMSG_BRS)? 1 : 0;
    msg->esi = (flags & canFDMSG_ESI)? 1 : 0;
    msg->dlc = LEN2DLC(len);
#ifndef CAN_20_ONLY
    memcpy(msg->data, data, CANFD_MAX_LEN);
#else
    memcpy(msg->data, data, CAN_MAX_LEN);
#endif
    msg->timestamp.sec = (long)(timestamp / 1000ul);
    msg->timestamp.usec = (long)(timestamp % 1000ul) * 1000l;
    can[handle].status.b.receiver_empty = 0;// message read

    return CANERR_NOERROR;
}

int can_status(int handle, unsigned char *status)
{
    unsigned long flags;                // status flags
    canStatus rc;                       // return value

    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if((rc = canRequestChipStatus(can[handle].handle)) != canOK)
        return kvaser_error(rc);
    if((rc = canReadStatus(can[handle].handle, &flags)) != canOK)
        return kvaser_error(rc);

    can[handle].status.b.bus_off = (flags & canSTAT_BUS_OFF)? 1 : 0;
    can[handle].status.b.bus_error = (flags & canSTAT_ERROR_PASSIVE)? 1 : 0;
    can[handle].status.b.warning_level = (flags & canSTAT_ERROR_WARNING)? 1 : 0;
    can[handle].status.b.message_lost |= (flags & canSTAT_RXERR)? 1 : 0;
    can[handle].status.b.transmitter_busy |= (flags & canSTAT_TX_PENDING)? 1 : 0;
    if(status)                          // status-register
      *status = can[handle].status.byte;
      
    return CANERR_NOERROR;
}

int can_busload(int handle, unsigned char *load, unsigned char *status)
{
    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if(!can[handle].status.b.can_stopped) { // must be running:
        if (load)
            *load = 0;                  //   TODO: bus-load [percent]
    }
    return can_status(handle, status);  // status-register
}

int can_bitrate(int handle, can_bitrate_t *bitrate, can_speed_t *speed)
{
    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if(!can[handle].status.b.can_stopped) { // must be running:
        if(bitrate)
            memcpy(bitrate, &can[handle].bitrate, sizeof(can_bitrate_t));
        if(speed)
            memset(speed, 0, sizeof(can_speed_t)); // TODO: calculate this!
    }
    return can_status(handle, NULL);    // current status
}

int can_interface(int handle, int *board, unsigned char *mode, void *param)
{

    if(!init)                           // must be initialized
        return CANERR_NOTINIT;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return CANERR_HANDLE;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return CANERR_HANDLE;

    if(board)                           // handle of the CAN channel
        *board = can[handle].channel;
    if(mode)                            // current opperation mode
        *mode = can[handle].mode.byte;
    (void)param;                        // no parameters available

    return CANERR_NOERROR;
}

char *can_hardware(int handle)
{
    if(!init)                           // must be initialized
        return NULL;
    if(!IS_HANDLE_VALID(handle))        // must be a valid handle
        return NULL;
    if(can[handle].handle == canINVALID_HANDLE) // must be an opened handle
        return NULL;

    if(canGetChannelData(can[handle].channel, canCHANNELDATA_CHANNEL_NAME, hardware, 255) != canOK)
        return NULL;

    return (char*)hardware;             // hardware version
}

char *can_software(int handle)
{
    unsigned short version;             // version number

    if(!init)                           // must be initialized
        return NULL;
    (void)handle;                       // handle not needed here

    version = canGetVersion();
    _snprintf_s(software, 256, 256, "Kvaser CANLIB API V%u.%u (canlib32.dll)", (version >> 8), (version & 0xFF));

    return (char*)software;             // software version
}

int can_library(unsigned short *version, unsigned char *revision, unsigned long *build)
{
    if(version)
        *version = ((unsigned short)VERSION_MAJOR << 8) | ((unsigned short)VERSION_MINOR & 0x00FFu);
    if(revision)
        *revision = (unsigned char)VERSION_REVISION;
    if(build)
        *build = (unsigned long)BUILD_NO;

    return KVASER_LIB_ID;               // library ID
}

/*  -----------  local functions  ----------------------------------------
 */

static int kvaser_error(canStatus status)
{
    if((canOK > status) && (status > canERR__RESERVED))
        return KVASER_ERR_OFFSET + (int)status;

    return KVASER_ERR_UNKNOWN;
}

/*  -----------  revision control  ---------------------------------------
 */

char* can_version()
{
    return (char*)_id;
}
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
