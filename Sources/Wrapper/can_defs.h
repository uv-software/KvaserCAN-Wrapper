/*  SPDX-License-Identifier: BSD-2-Clause OR GPL-3.0-or-later */
/*
 *  CAN Interface API, Version 3 (for Kvaser CAN Interfaces)
 *
 *  Copyright (c) 2017-2021 Uwe Vogt, UV Software, Berlin (info@uv-software.com)
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
#ifndef CAN_DEFS_H_INCLUDED
#define CAN_DEFS_H_INCLUDED

/*  -----------  includes  ------------------------------------------------
 */

#include "CANAPI_Defines.h"
#include "KvaserCAN_Defines.h"


/*  -----------  options  ------------------------------------------------
 */
#define KVASER_TRM_TIMEOUT         55   // time-out value for canWriteWait
#define KVASER_ASYNCHRONOUS_WRITE       // w/o transmit acknowledge
#define KVASER_SHARED_ACCESS            // permit non-exclusive access
#define KVASER_VIRTUAL_CHANNELS         // support of virtual channels
/*#define KVASER_SIMULATED_CHANNELS     // support of simulated channels */
/* note: all other options moved into header CANAPI_KvaserCAN.h */


/*  -----------  defines  ------------------------------------------------
 */
/* note: all defines moved into header CANAPI_KvaserCAN.h */


/*  -----------  types  --------------------------------------------------
 */
/* note: all type definitions moved into header CANAPI_KvaserCAN.h */


#endif /* CAN_DEFS_H_INCLUDED */
/** @}
 */
/*  ----------------------------------------------------------------------
 *  Uwe Vogt,  UV Software,  Chausseestrasse 33 A,  10115 Berlin,  Germany
 *  Tel.: +49-30-46799872,  Fax: +49-30-46799873,  Mobile: +49-170-3801903
 *  E-Mail: uwe.vogt@uv-software.de,  Homepage: http://www.uv-software.de/
 */
