/*
 *                   Copyright 1994-2017 by KVASER AB, SWEDEN
 *
 *                        WWW: http://www.kvaser.com
 *
 * This software is furnished under a license and may be used and copied
 * only in accordance with the terms of such license.
 *
 * \note MATLAB users on Windows: if you define WIN32_LEAN_AND_MEAN before
 * including this file, you will see a lot less warnings.
 *
 */

#ifndef _BUS_PARAMS_TQ_H
#define _BUS_PARAMS_TQ_H

/**
 * \file bus_params_tq.h
 * \brief Definitions for the CANLIB API.
 * \details
 * \ingroup CAN
 *
*/

/**
 *\b Constraints
 \verbatim
 Constraints that must be fulfilled when opening channel in classic CAN Mode:

   tq         = 1 + prop + phase1 + phase2
   tq        >= 3
   sjw       <= min(phase1, phase2)
   prescaler >= 1
 \endverbatim
*
 \verbatim
   Constraints that must be fulfilled when opening channel in CAN FD Mode:

   arbitration.tq         = 1 + arbitration.prop + arbitration.phase1 + arbitration.phase2
   arbitration.tq        >= 3
   arbitration.sjw       <= min(arbitration.phase1, arbitration.phase2)
   arbitration.prescaler >= 1
   arbitration.prescaler <= 2

   data.tq         = 1 + data.phase1 + data.phase2
   data.tq        >= 3
   data.sjw       <= min(data.phase1, data.phase2)
   data.prop       = 0
   data.prescaler  = arbitration.prescaler
 \endverbatim
*
* Used in \ref canSetBusParamsTq, \ref canSetBusParamsFdTq, \ref canGetBusParamsTq and \ref canGetBusParamsTq
*/
typedef struct kvBusParamsTq {
  int tq;                /**< Total bit time, in number of time quanta. */
  int phase1;            /**< Phase segment 1, in number of time quanta */
  int phase2;            /**< Phase segment 2, in number of time quanta */
  int sjw;               /**< Sync jump width, in number of time quanta */
  int prop;              /**< Propagation segment, in number of time quanta */
  int prescaler;         /**< Prescaler */
} kvBusParamsTq;

#endif
