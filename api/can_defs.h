/*  -- $HeadURL$ --
 *
 *  project   :  CAN - Controller Area Network
 *
 *  purpose   :  CAN Interface API, Version 3 (Kvaser canLib32)
 *
 *  copyright :  (C) 2017-2018, UV Software, Berlin
 *
 *  compiler  :  Microsoft Visual C/C++ Compiler (Version 19.16)
 *
 *  export    :  (see below)
 *
 *  includes  :  (none)
 *
 *  author    :  Uwe Vogt, UV Software
 *
 *  e-mail    :  uwe.vogt@uv-software.de
 *
 *
 *  -----------  description  --------------------------------------------
 */
/** @file        can_api.h
 *
 *  @brief       CAN API V3 for Kvaser CAN Interfaces - Defines
 *
 *               For Kvaser CANLIB API Interfaces (canlib32.dll V5.28).
 *
 *  @author      $Author$
 *
 *  @version     $Rev$
 *
 *  @defgroup    can_defs Options and Definitions
 *  @{
 */
#ifndef __CAN_DEFS_H
#define __CAN_DEFS_H

/*  -----------  options  ------------------------------------------------
 */

#define _ASYNCHRONOUS_WRITE             // w/o transmit acknowledge
#define _BLOCKING_READ                  // blocking read via wait event
#define _SHARED_ACCESS                  // permit non-exclusive access
#define _VIRTUAL_CHANNELS               // support of virtual channels
/*#define _SIMULATED_CHANNELS           // support of simulated channels */
#define _CiA_BIT_TIMING                 // CiA bit-timing (from CANopen spec.)


/*  -----------  defines  ------------------------------------------------
 */

#ifndef _CAN_DEFS                       // Kvaser CAN interfaces:
 #define KVASER_CAN_CHANNEL0        0   //   Kvaser CAN Interface, Channel 0
 #define KVASER_CAN_CHANNEL1        1   //   Kvaser CAN Interface, Channel 1
 #define KVASER_CAN_CHANNEL2        2   //   Kvaser CAN Interface, Channel 2
 #define KVASER_CAN_CHANNEL3        3   //   Kvaser CAN Interface, Channel 3
 #define KVASER_CAN_CHANNEL4        4   //   Kvaser CAN Interface, Channel 4
 #define KVASER_CAN_CHANNEL5        5   //   Kvaser CAN Interface, Channel 5
 #define KVASER_CAN_CHANNEL6        6   //   Kvaser CAN Interface, Channel 6
 #define KVASER_CAN_CHANNEL7        7   //   Kvaser CAN Interface, Channel 7
 #define KVASER_BOARDS             (8)  //   number of Kvaser Interface boards

 #define KVASER_MAX_HANDLES        (8)  //   maximum number of interface handles

 #define KVASER_ERR_OFFSET         -600 //   offset for Kvaser-specific errors
 #define KVASER_ERR_UNKNOWN        -699 //   unknown error

 #define KVASER_TRM_TIMEOUT         55  //   time-out for canWriteWait (in [ms])

 #define KVASER_LIB_ID              600 //   library ID (CAN/COP API V1 compatible)
 #define KVASER_LIB_CANLIB                  "canLib32.DLL"
 #define KVASER_LIB_VENDOR                  "Kvaser AB"
#endif
#endif /* __CAN_DEFS_H */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
