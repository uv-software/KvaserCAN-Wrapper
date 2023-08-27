@echo off

pushd
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars32.bat" x86
popd

pushd
call build_no.bat
rem type .\Sources\build_no.h
rem pause

call msbuild.exe .\Trial\kvl_test.vcxproj /t:Clean;Build /p:"Configuration=Debug";"Platform=Win32"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\CANAPI\uvcankvl.vcxproj /t:Clean;Build /p:"Configuration=Release_dll";"Platform=Win32"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\CANAPI\uvcankvl.vcxproj /t:Clean;Build /p:"Configuration=Debug_lib";"Platform=Win32"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\KvaserCAN\KvaserCAN.vcxproj /t:Clean;Build /p:"Configuration=Release_dll";"Platform=Win32"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\KvaserCAN\KvaserCAN.vcxproj /t:Clean;Build /p:"Configuration=Debug_lib";"Platform=Win32"
if errorlevel 1 goto end

echo Copying artifacts...
set BIN=".\Binaries"
if not exist %BIN% mkdir %BIN%
set BIN="%BIN%\x86"
if not exist %BIN% mkdir %BIN%
copy /Y .\Libraries\CANAPI\Release_dll\u3cankvl.dll %BIN%
copy /Y .\Libraries\CANAPI\Release_dll\u3cankvl.lib %BIN%
copy /Y .\Libraries\KvaserCAN\Release_dll\uvKvaserCAN.dll %BIN%
copy /Y .\Libraries\KvaserCAN\Release_dll\uvKvaserCAN.lib %BIN%
set BIN="%BIN%\lib"
if not exist %BIN% mkdir %BIN%
copy /Y .\Libraries\CANAPI\Debug_lib\u3cankvl.lib %BIN%
copy /Y .\Libraries\CANAPI\Debug_lib\u3cankvl.pdb %BIN%
copy /Y .\Libraries\KvaserCAN\Debug_lib\uvKvaserCAN.lib %BIN%
copy /Y .\Libraries\KvaserCAN\Debug_lib\uvKvaserCAN.pdb %BIN%
copy /Y .\Sources\CANlib\x86\canlib32.lib %BIN%
echo Static libraries (x86) > %BIN%\readme.txt

echo Copying header files...
set INC=".\Includes"
if not exist %INC% mkdir %INC%
copy /Y .\Sources\KvaserCAN*.h %INC%
copy /Y .\Sources\CANAPI\CANAPI.h %INC%
copy /Y .\Sources\CANAPI\CANAPI_Types.h %INC%
copy /Y .\Sources\CANAPI\CANAPI_Defines.h %INC%
copy /Y .\Sources\CANAPI\CANBTR_Defaults.h %INC%
copy /Y .\Sources\CANAPI\can_api.h %INC%
copy /Y .\Sources\CANAPI\can_btr.h %INC%

call msbuild.exe .\Utilities\can_moni\can_moni.vcxproj /t:Clean;Build /p:"Configuration=Release";"Platform=Win32"
if errorlevel 1 goto end

call msbuild.exe .\Utilities\can_test\can_test.vcxproj /t:Clean;Build /p:"Configuration=Release";"Platform=Win32"
if errorlevel 1 goto end

echo Copying utilities...
set BIN=".\Binaries"
if not exist %BIN% mkdir %BIN%
set BIN="%BIN%\x86"
if not exist %BIN% mkdir %BIN%
copy /Y .\Utilities\can_moni\Release\can_moni.exe %BIN%
copy /Y .\Utilities\can_test\Release\can_test.exe %BIN%

:end
popd
pause
