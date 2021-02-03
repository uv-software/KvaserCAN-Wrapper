//
//  CAN Interface API, Version 3 (for Kvaser CAN Interfaces)
//
//  Copyright (C) 2017-2021  Uwe Vogt, UV Software, Berlin (info@uv-software.com)
//
//  This file is part of KvaserCAN-Wrapper.
//
//  KvaserCAN-Wrapper is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  KvaserCAN-Wrapper is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with KvaserCAN-Wrapper.  If not, see <https://www.gnu.org/licenses/>.
//
#ifndef KVASERCAN_H_INCLUDED
#define KVASERCAN_H_INCLUDED

#include "CANAPI.h"

/// \name   KvaserCAN
/// \brief  KvaserCAN dynamic library
/// \{
#define KVASERCAN_LIBRARY_ID  CANLIB_KVASER_32
#define KVASERCAN_LIBRARY_NAME  CANDLL_KVASER_32
#define KVASERCAN_LIBRARY_VENDOR  "UV Software, Berlin"
#define KVASERCAN_LIBRARY_LICENSE  "GNU Lesser General Public License, Version 3"
#define KVASERCAN_LIBRARY_COPYRIGHT  "Copyright (C) 2017-2021  Uwe Vogt, UV Software, Berlin"
#define KVASERCAN_LIBRARY_HAZARD_NOTE  "If you connect your CAN device to a real CAN network when using this library,\n" \
                                       "you might damage your application."
/// \}


/// \name   KvaserCAN API
/// \brief  CAN API V3 driver for Kvaser CAN interfaces
/// \note   See CCANAPI for a description of the overridden methods
/// \{
class CANCPP CKvaserCAN : public CCANAPI {
private:
    CANAPI_OpMode_t m_OpMode;  ///< CAN operation mode
    CANAPI_Bitrate_t m_Bitrate;  ///< CAN bitrate settings
    struct {
        uint64_t u64TxMessages;  ///< number of transmitted CAN messages
        uint64_t u64RxMessages;  ///< number of received CAN messages
        uint64_t u64ErrorFrames;  ///< number of received status messages
    } m_Counter;
    // opaque data type
    struct SCAN;  ///< C++ forward declaration
    SCAN *m_pCAN;  ///< Kvaser CANlib interface
public:
    // constructor / destructor
    CKvaserCAN();
    ~CKvaserCAN();
    // CKvaserCAN-specific error codes (CAN API V3 extension)
    enum EErrorCodes {
        // note: range 0...-99 is reserved by CAN API V3
        GeneralError = VendorSpecific, ///< mapped Kvaser CANlib error codes
    };
    // CCANAPI overrides
    static CANAPI_Return_t ProbeChannel(int32_t channel, CANAPI_OpMode_t opMode, const void *param, EChannelState &state);
    static CANAPI_Return_t ProbeChannel(int32_t channel, CANAPI_OpMode_t opMode, EChannelState &state);

    CANAPI_Return_t InitializeChannel(int32_t channel, can_mode_t opMode, const void *param = NULL);
    CANAPI_Return_t TeardownChannel();
    CANAPI_Return_t SignalChannel();

    CANAPI_Return_t StartController(CANAPI_Bitrate_t bitrate);
    CANAPI_Return_t ResetController();

    CANAPI_Return_t WriteMessage(CANAPI_Message_t message, uint16_t timeout = 0U);
    CANAPI_Return_t ReadMessage(CANAPI_Message_t &message, uint16_t timeout = CANREAD_INFINITE);

    CANAPI_Return_t GetStatus(CANAPI_Status_t &status);
    CANAPI_Return_t GetBusLoad(uint8_t &load);

    CANAPI_Return_t GetBitrate(CANAPI_Bitrate_t &bitrate);
    CANAPI_Return_t GetBusSpeed(CANAPI_BusSpeed_t &speed);

    CANAPI_Return_t GetProperty(uint16_t param, void *value, uint32_t nbyte);
    CANAPI_Return_t SetProperty(uint16_t param, const void *value, uint32_t nbyte);

    char *GetHardwareVersion();  // (for compatibility reasons)
    char *GetFirmwareVersion();  // (for compatibility reasons)
    static char *GetVersion();  // (for compatibility reasons)

    static CANAPI_Return_t MapIndex2Bitrate(int32_t index, CANAPI_Bitrate_t &bitrate);
    static CANAPI_Return_t MapString2Bitrate(const char *string, CANAPI_Bitrate_t &bitrate);
    static CANAPI_Return_t MapBitrate2String(CANAPI_Bitrate_t bitrate, char *string, size_t length);
    static CANAPI_Return_t MapBitrate2Speed(CANAPI_Bitrate_t bitrate, CANAPI_BusSpeed_t &speed);
private:
    CANAPI_Return_t MapBitrate2Sja1000(CANAPI_Bitrate_t bitrate, uint16_t &btr0btr1);
    CANAPI_Return_t MapSja10002Bitrate(uint16_t btr0btr1, CANAPI_Bitrate_t &bitrate);
public:
    static uint8_t DLc2Len(uint8_t dlc);
    static uint8_t Len2Dlc(uint8_t len);
};
/// \}

/// \name   KvaserCAN Property IDs
/// \brief  Properties that can be read (or written)
/// \{
#define KVASERCAN_PROPERTY_CANAPI          (CANPROP_GET_SPEC)
#define KVASERCAN_PROPERTY_VERSION         (CANPROP_GET_VERSION)
#define KVASERCAN_PROPERTY_PATCH_NO        (CANPROP_GET_PATCH_NO)
#define KVASERCAN_PROPERTY_BUILD_NO        (CANPROP_GET_BUILD_NO)
#define KVASERCAN_PROPERTY_LIBRARY_ID      (CANPROP_GET_LIBRARY_ID)
#define KVASERCAN_PROPERTY_LIBRARY_NAME    (CANPROP_GET_LIBRARY_DLLNAME)
#define KVASERCAN_PROPERTY_LIBRARY_VENDOR  (CANPROP_GET_LIBRARY_VENDOR)
#define KVASERCAN_PROPERTY_DEVICE_TYPE     (CANPROP_GET_DEVICE_TYPE)
#define KVASERCAN_PROPERTY_DEVICE_NAME     (CANPROP_GET_DEVICE_NAME)
#define KVASERCAN_PROPERTY_DEVICE_VENDOR   (CANPROP_GET_DEVICE_VENDOR)
#define KVASERCAN_PROPERTY_DEVICE_DLLNAME  (CANPROP_GET_DEVICE_DLLNAME)
#define KVASERCAN_PROPERTY_OP_CAPABILITY   (CANPROP_GET_OP_CAPABILITY)
#define KVASERCAN_PROPERTY_OP_MODE         (CANPROP_GET_OP_MODE)
#define KVASERCAN_PROPERTY_BITRATE         (CANPROP_GET_BITRATE)
#define KVASERCAN_PROPERTY_SPEED           (CANPROP_GET_SPEED)
#define KVASERCAN_PROPERTY_STATUS          (CANPROP_GET_STATUS)
#define KVASERCAN_PROPERTY_BUSLOAD         (CANPROP_GET_BUSLOAD)
#define KVASERCAN_PROPERTY_TX_COUNTER      (CANPROP_GET_TX_COUNTER)
#define KVASERCAN_PROPERTY_RX_COUNTER      (CANPROP_GET_RX_COUNTER)
#define KVASERCAN_PROPERTY_ERR_COUNTER     (CANPROP_GET_ERR_COUNTER)
// TODO: insert coin here
/// \}
#endif // KVASERCAN_H_INCLUDED
