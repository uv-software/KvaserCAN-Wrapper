### Wrapper Library for Kvaser CAN Interfaces (Windows&reg;)

_Copyright &copy; 2017-2021   Uwe Vogt, UV Software, Berlin (info@uv-software.de)_

# CAN API V3 for Kvaser CAN Interfaces

CAN API V3 is a wrapper specification to have a uniform CAN Interface API for various CAN interfaces from different vendors running under multiple operating systems.

## KvaserCAN-Wrapper

This repo contains the source code for a CAN API V3 compatible wrapper library under Windows for CAN Interfaces from Kvaser AB, Sweden.
The wrapper library is build upon the Kvaser CANlib SDK.

### CAN Interface API, Version 3

```C
#if (OPTION_CANAPI_LIBRARY != 0)
extern int can_test(int32_t library, int32_t channel, uint8_t mode, const void *param, int *result);
extern int can_init(int32_t library, int32_t channel, uint8_t mode, const void *param);
#else
extern int can_test(int32_t channel, uint8_t mode, const void *param, int *result);
extern int can_init(int32_t channel, uint8_t mode, const void *param);
#endif
extern int can_exit(int handle);
extern int can_kill(int handle);

extern int can_start(int handle, const can_bitrate_t *bitrate);
extern int can_reset(int handle);

extern int can_write(int handle, const can_message_t *message, uint16_t timeout);
extern int can_read(int handle, can_message_t *message, uint16_t timeout);

extern int can_status(int handle, uint8_t *status);
extern int can_busload(int handle, uint8_t *load, uint8_t *status);

extern int can_bitrate(int handle, can_bitrate_t *bitrate, can_speed_t *speed);
extern int can_property(int handle, uint16_t param, void *value, uint32_t nbyte);

extern char *can_hardware(int handle);
extern char *can_software(int handle);

#if (OPTION_CANAPI_LIBRARY != 0)
extern char *can_library(int handle);
#endif
extern char* can_version(void);
```
See header file `can_api.h` for a description of the provided functions.


## Build Targets

Important note: _To build any of the following build targets run the script_ `build_no.bat` _to generate a pseudo build number._
```
C:\Users\haumea>cd C:\Projects\CAN\DRV\Drivers\PCANBbasic
C:\Projects\CAN\DRV\Drivers\PCANBbasic>build_no.bat
```
Repeat this step after each `git commit`, `git pull`, `git clone`, etc.

_Tbd._

#### kvasercan (DLL)

___kvasercan___ is a dynamic link library with a CAN API V3 compatible application programming interface for use in __C++__ applications.
See header file `KvaserCAN.h` for a description of all class members.

#### u3cankvl (DLL)

___u3cankvl___ is a dynamic link library with a CAN API V3 compatible application programming interface for use in __C__ applications.
See header file `can_api.h` for a description of all API functions.

#### can_moni (CLI)

`can_moni` is a command line tool to view incoming CAN messages.
I hate this messing around with binary masks for identifier filtering.
So I wrote this little program to have an exclude list for single identifiers or identifier ranges (see program option `/EXCLUDE` or just `/X`). Precede the list with a `~` and you get an include list.

Type `can_moni /?` to display all program options.

#### can_test (CLI)

`can_test` is a command line tool to test CAN communication.
Originally developed for electronic environmental tests on an embedded Linux system with SocketCAN, I´m using it for many years as a traffic generator for CAN stress-tests.

Type `can_test /?` to display all program options.

### Target Platform

- Windows 10 (x64 operating systems)

### Development Environments

- Microsoft Visual Studio Community 2017 (Version 15.9)

### Supported CAN Hardware

- Kvaser Leaf Light V2 - single channel, CAN 2.0 (EAN: 73-30130-00685-0)
- Kvaser Leaf Pro HS V2 - single channel, CAN FD (EAN: 73-30130-00843-4)

### Required Kvaser CANlib SDK

- Version 5.34 or later _(Latest is Greatest!)_

## Known Bugs and Caveats

- ...

## This and That

The Kvaser CANlib SDK can be downloaded form [Kvaser](https://www.kvaser.com/) website.
Please note the copyright and license agreements.

### Licenses

#### CAN API V3 License

CAN API V3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CAN API V3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with CAN API V3.  If not, see <http://www.gnu.org/licenses/>.

#### KvaserCAN-Wrapper License

KvaserCAN-Wrapper is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

KvaserCAN-Wrapper is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with KvaserCAN-Wrapper.  If not, see <http://www.gnu.org/licenses/>.

### Trademarks

Windows is a registered trademark of Microsoft Corporation in the United States and/or other countries.

### Hazard Note

_If you connect your CAN device to a real CAN network when using this library, you might damage your application._

### Contact

Uwe Vogt \
UV Software \
Chausseestrasse 33a \
10115 Berlin \
Germany

E-Mail: mailto://info@uv-software.de \
Internet: https://www.uv-software.de

##### *Enjoy!*
