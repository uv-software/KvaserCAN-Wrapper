### CAN API V3 Wrapper Library for Kvaser CAN Interfaces (Windows&reg;)

_Copyright &copy; 2017-2023 Uwe Vogt, UV Software, Berlin (info@uv-software.de)_
_All rights reserved._

# Deployment

## Create the Release Candidate

### Precondition

- **_All changes are made exclusively on a feature branch!_**

### Preparation

1. Update the CANlib SDK in `$(PROJROOT)\Sources\CANlib` from Kvaser´s website
   when required and commit them with commit comment:
  - `Update Kvaser´s CANlib SDK (version `_n_`.`_nn_`)` \
    `- `_list of major changes (optional)_
2. Update the CAN API V3 sources in `$(PROJROOT)\Sources\CANAPI` from SVN repo
   when required and commit them with commit comment:
  - `Update CAN API V3 sources to rev. `_nnn_ \
    `- `_list of major changes (optional)_
3. Update the CAN API V3 testing sources in `$(PROJROOT)\Tests` from SVN repo
   when required and commit them with commit comment:
  - `Update CAN API V3 testing sources to rev. `_nnn_ \
    `- `_list of major changes (optional)_
4. Check and update the version and date information in the following files:
  - `$(PROJROOT)\Sources\KvaserCAN.h`
  - `$(PROJROOT)\Sources\KvaserCAN.cpp`
  - `$(PROJROOT)\Sources\Wrapper\can_api.cpp`
  - `$(PROJROOT)\Libraries\CANAPI\Resource.rc`
  - `$(PROJROOT)\Libraries\KvaserCAN\Resource.rc`
  - `$(PROJROOT)\Utilities\can_moni\Driver.h`
  - `$(PROJROOT)\Utilities\can_test\Driver.h`

### Procedure

1. Check the working directory for uncommitted changes.
  - _**There should not be any uncommitted change.**_
  - _If there are uncommitted changes then commit them or revert them._
2. Open the trial program with Visual Studio and run a code analysis.
  - _**There should not be any serious finding.**_
  - _If there are findings then fix them or create an issue in the repo._
3. Run `x86_build.bat` and `x64_build.bat` in the project root directory.
  - _**There should be absolute no compiler or linker error!**_
  - _If there are compiler or linker warnings then think twice._
4. Try out the trial program with different options.
  - _**There should be no crash, hangup, or any other error.**_
  - _If there is an error then fix it or create an issue in the repo._
5. Try out the utilities with different options.
  - _**There should be no crash, hangup, or any other error.**_
  - _If there is an error then fix it or create an issue in the repo._
6. Build and try out the examples (repair them if necessary);
  - `$(PROJROOT)\Examples\C++`
  - `$(PROJROOT)\Examples\Python`

### Pull Request

1. Update the `README.md` (e.g. development environment, supported devices, etc.).
2. Push the feature branch onto the remote repo.
3. Create a pull request and name it somehow like '**Release Candidate _n_ for** ...'.
4. Review the changes and merge the feature branch into the default branch.

## Create the Release Tag

### Preparation

1. Pull or clone the default branch on all development systems.
2. Double check all version numbers again (see above).
3. Run the batch file in the project root directory:
  - `C:\Users\haumea>cd C:\Projects\CAN\Drivers\KvaserCAN`
  - `C:\Projects\CAN\Drivers\KvaserCAN>x86_build.bat`
  - `C:\Projects\CAN\Drivers\KvaserCAN>x64_build.bat`
  - `C:\Projects\CAN\Drivers\KvaserCAN>x86_install.bat`
  - `C:\Projects\CAN\Drivers\KvaserCAN>x64_install.bat`
4. Build the CAN API V3 GoogleTest program:
  - `C:\Users\haumea>cd C:\Projects\CAN\Drivers\KvaserCAN\Tests`
  - `C:\Projects\CAN\Drivers\KvaserCAN\Tests>x64_build.bat`
5. Run the CAN API V3 GoogleTest with two Kvaser CAN devices in CAN 2.0 mode:
  - `C:\Projects\CAN\Drivers\KvaserCAN\Tests>x64\Debug\kvl_testing --can_dut1="Kvaser CAN Channel 0" --can_dut2="Kvaser CAN Channel 1" --gtest_output=xml:TestReport_KvaserCAN.xml --run_all=YES --smoketest_frames=100000` [...]
  - _If there is any error then **stop** here or create an issue for each error in the repo._
  - Copy the test report into the binaries directory `$(PROJROOT)\Binaries`.
6. Run the CAN API V3 GoogleTest with two Kvaser CAN devices in CAN FD mode:
  - `C:\Projects\CAN\Drivers\KvaserCAN\Tests>x64\Debug\kvl_testing --can_dut1="Kvaser CAN Channel 1" --can_dut2="Kvaser CAN Channel 2" --can_bitrate=DEFAULT --can_mode=CANFD+BRS  --gtest_output=xml:TestReport_KvaserCAN_FD.xml --run_all=YES --smoketest_frames=100000` [...]
  - _If there is any error then **stop** here or create an issue for each error in the repo._
  - Copy the test report into the binaries directory `$(PROJROOT)\Binaries`.
7. Pack the artifacts into a .zip-archive, e.g. `artifacts.zip`:
  - `$(PROJROOT)\Binaries\*.*`
  - `$(PROJROOT)\Includes\*.*`
  - `$(PROJROOT)\README.md`
  - `$(PROJROOT)\LICENSE`
8. Double check and update the [`README.md`](https://github.com/uv-software/KvaserCAN-Wrapper/blob/main/README.md) on GitHub (or insert just a blank).

### Procedure

1. Click on `Draft a new release` in the [GitHub](https://github.com/uv-software/KvaserCAN-Wrapper) repo.
2. Fill out all required fields:
  - Tag version: e.g `v0.2.4` (cf. semantic versioning)
  - Target: `main` (default branch)
  - Release title: e.g. `Release of August 29, 2023`
  - Change-log: list all major changes, e.g. from commit comments
  - Assets: drag and drop the artifacts archive (see above)
3. Click on `Publish release`.
4. That´s all folks!

### Announcement

1. Create a new post with the change-log in the `mac-can.github.io` repo.
2. Update the KvaserCAN driver page in the `mac-can.github.io` repo.
3. Post the new release on
[Twitter](https://twitter.com/uv_software),
[LinkedIn](https://linkedin.com/in/uwe-vogt-software),
[Facebook](https://facebook.com/uvsoftware.berlin),
etc.
