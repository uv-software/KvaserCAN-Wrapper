/*
 *  CAN Interface API, Version 3 (for Kvaser CAN Interfaces)
 *
 *  Copyright (C) 2017-2021  Uwe Vogt, UV Software, Berlin (info@uv-software.de)
 *
 *  This file is part of KvaserCAN-Wrapper.
 *
 *  KvaserCAN-Wrapper is free software : you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  KvaserCAN-Wrapper is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with KvaserCAN-Wrapper.  If not, see "http://www.gnu.org/licenses/".
 */
/** @file        CANAPI_KvaserCAN.h
 *
 *  @brief       CAN API V3 for generic CAN Interfaces - Kvaser CAN Interfaces
 *
 *  @remarks     For Kvaser CANlib API Interfaces (canlib32.dll).
 *
 *  @author      $Author: haumea $
 *
 *  @version     $Rev: 963 $
 *
 *  @addtogroup  can_api
 *  @{
 */
#ifndef CANAPI_KVASERCAN_H_INCLUDED
#define CANAPI_KVASERCAN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/*  -----------  includes  ------------------------------------------------
 */

#include <stdint.h>                     /* C99 header for sized integer types */
#include <stdbool.h>                    /* C99 header for boolean type */


/*  -----------  options  ------------------------------------------------
 */

#define OPTION_KVASER_CiA_BIT_TIMING    /* CiA bit-timing (from CANopen spec.) */


/*  -----------  defines  ------------------------------------------------
 */

/** @name  CAN API Interfaces
 *  @brief Kvaser CAN channel no.
 *  @{ */
#define KVASER_CAN_CHANNEL0        0    /**< Kvaser CAN Interface, Channel 0 */
#define KVASER_CAN_CHANNEL1        1    /**< Kvaser CAN Interface, Channel 1 */
#define KVASER_CAN_CHANNEL2        2    /**< Kvaser CAN Interface, Channel 2 */
#define KVASER_CAN_CHANNEL3        3    /**< Kvaser CAN Interface, Channel 3 */
#define KVASER_CAN_CHANNEL4        4    /**< Kvaser CAN Interface, Channel 4 */
#define KVASER_CAN_CHANNEL5        5    /**< Kvaser CAN Interface, Channel 5 */
#define KVASER_CAN_CHANNEL6        6    /**< Kvaser CAN Interface, Channel 6 */
#define KVASER_CAN_CHANNEL7        7    /**< Kvaser CAN Interface, Channel 7 */
#define KVASER_BOARDS              8    /**< number of Kvaser Interface boards */
/** @} */

/** @name  CAN API Error Codes
 *  @brief CANlib-specific error code
 *  @{ */
// TODO: insert coin here
#define KVASER_ERR_OFFSET      (-600)   /**< offset for CANlib-specific errors */
#define KVASER_ERR_UNKNOWN     (-699)   /**< unknown error */
/** @} */

/** @name  CAN API Property Value
 *  @brief CANlib parameter to be read or written
 *  @{ */
// TODO: insert coin here
/** @} */

/** @name  CAN API Library ID
 *  @brief Library ID and dynamic library names
 *  @{ */
#define KVASER_LIB_ID            600    /**< library ID (CAN/COP API V1 compatible) */
#if defined(_WIN32) || defined (_WIN64)
 #define KVASER_LIB_CANLIB              "canLib32.DLL"
 #define KVASER_LIB_WRAPPER             "u3cankvl.dll"
#elif defined(__APPLE__)
 #define KVASER_LIB_CANLIB              "(n/a)"
 #define KVASER_LIB_WRAPPER             "libUVCANKVL.dylib"
#else
 #error Platform not supported
#endif
/** @} */

/** @name  Miscellaneous
 *  @brief More or less useful stuff
 *  @{ */                               // TODO: to be moved or removed
#define KVASER_LIB_VENDOR               "Kvaser AB, Sweden"
#define CAN_API_VENDOR                  "UV Software, Berlin"
#define CAN_API_AUTHOR                  "Uwe Vogt, UV Software"
#define CAN_API_WEBSITE                 "www.uv-software.com"
#define CAN_API_CONTACT                 "info@uv-software.com"
#define CAN_API_LICENSE                 "GNU Lesser General Public License, Version 3"
#define CAN_API_COPYRIGHT               "Copyright (C) 2005-20%02u, UV Software, Berlin"
#define CAN_API_HAZARD_NOTE             "Do not connect your CAN device to a real CAN network when using this program.\n" \
                                        "This can damage your application."
/** @} */

#ifdef __cplusplus
}
#endif
#endif /* CANAPI_KVASERCAN_H_INCLUDED */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
