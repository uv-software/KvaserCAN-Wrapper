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
/** @addtogroup  can_api
 *  @{
 */
#ifndef CAN_DEFS_H_INCLUDED
#define CAN_DEFS_H_INCLUDED

/*  -----------  includes  ------------------------------------------------
 */

#include "CANAPI_KvaserCAN.h"


/*  -----------  options  ------------------------------------------------
 */
#define KVASER_TRM_TIMEOUT         55   // time-out value for canWriteWait
#define KVASER_ASYNCHRONOUS_WRITE       // w/o transmit acknowledge
#define KVASER_SHARED_ACCESS            // permit non-exclusive access
#define KVASER_VIRTUAL_CHANNELS         // support of virtual channels
/*#define KVASER_SIMULATED_CHANNELS     // support of simulated channels */
/* note: all other options move into header CANAPI_KvaserCAN.h */


/*  -----------  defines  ------------------------------------------------
 */
/* note: all defines move into header CANAPI_KvaserCAN.h */


/*  -----------  types  --------------------------------------------------
 */
/* note: all type definitions move into header CANAPI_KvaserCAN.h */


#endif /* CAN_DEFS_H_INCLUDED */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
