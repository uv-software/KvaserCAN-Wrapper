/*  -- $HeadURL$ --
 *
 *  project   :  CAN - Controller Area Network
 *
 *  purpose   :  CAN Interface API, Version 3 (Kvaser canLib32)
 *
 *  copyright :  (C) 2017-2020, UV Software, Berlin
 *
 *  compiler  :  Microsoft Visual C/C++ Compiler (Version 19.16)
 *
 *  export    :  (see below)
 *
 *  includes  :  stdint.h, stdbool.h
 *
 *  author    :  Uwe Vogt, UV Software
 *
 *  e-mail    :  uwe.vogt@uv-software.de
 *
 *
 *  -----------  description  --------------------------------------------
 */
/** @file        can_defs.h
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
#ifndef CAN_DEFS_H_INCLUDED
#define CAN_DEFS_H_INCLUDED

/*  -----------  includes  ------------------------------------------------
 */

#ifdef _MSC_VER
//no Microsoft extensions please!
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif
#endif
#include <stdint.h>                     // C99 header for sized integer types
#include <stdbool.h>                    // C99 header for boolean type


/*  -----------  options  ------------------------------------------------
 */

#define CANAPI_CiA_BIT_TIMING           // CiA bit-timing (from CANopen spec.)
#define KVASER_ASYNCHRONOUS_WRITE       // w/o transmit acknowledge
#define KVASER_SHARED_ACCESS            // permit non-exclusive access
#define KVASER_VIRTUAL_CHANNELS         // support of virtual channels
/*#define KVASER_SIMULATED_CHANNELS     // support of simulated channels */


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
 #define KVASER_LIB_WRAPPER                 "u3cankvl.dll"
 #define KVASER_LIB_VENDOR                  "Kvaser AB, Sweden"
 #define CAN_API_VENDOR                     "UV Software, Berlin"
 #define CAN_API_AUTHOR                     "Uwe Vogt, UV Software"
 #define CAN_API_WEBSITE                    "www.uv-software.com"
 #define CAN_API_CONTACT                    "info@uv-software.com"
 #define CAN_API_LICENSE                    "UVS Freeware License (without any warranty or support)"
 #define CAN_API_COPYRIGHT                  "Copyright (C) 2005-20%02u, UV Software, Berlin"
 #define CAN_API_HAZARD_NOTE                "Do not connect your CAN device to a real CAN network when using this program.\n" \
                                            "This can damage your application."
#endif
#endif /* CAN_DEFS_H_INCLUDED */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
