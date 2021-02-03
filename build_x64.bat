@echo off

pushd
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" x64
popd

pushd
call build_no.bat
rem type .\Sources\build_no.h
rem pause

call msbuild.exe .\Trial\kvl_test.vcxproj /t:Clean;Build /p:"Configuration=Debug";"Platform=x64"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\CANAPI\uvcankvl.vcxproj /t:Clean;Build /p:"Configuration=Release_dll";"Platform=x64"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\CANAPI\uvcankvl.vcxproj /t:Clean;Build /p:"Configuration=Debug_lib";"Platform=x64"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\KvaserCAN\KvaserCAN.vcxproj /t:Clean;Build /p:"Configuration=Release_dll";"Platform=x64"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\KvaserCAN\KvaserCAN.vcxproj /t:Clean;Build /p:"Configuration=Debug_lib";"Platform=x64"
if errorlevel 1 goto end

echo Copying artifacts...
set BIN=".\Binaries"
if not exist %BIN% mkdir %BIN%
set BIN="%BIN%\x64"
if not exist %BIN% mkdir %BIN%
copy /Y .\Libraries\CANAPI\x64\Release_dll\u3cankvl.dll %BIN%
copy /Y .\Libraries\CANAPI\x64\Release_dll\u3cankvl.lib %BIN%
copy /Y .\Libraries\KvaserCAN\x64\Release_dll\uvKvaserCAN.dll %BIN%
copy /Y .\Libraries\KvaserCAN\x64\Release_dll\uvKvaserCAN.lib %BIN%
set BIN="%BIN%\lib"
if not exist %BIN% mkdir %BIN%
copy /Y .\Libraries\CANAPI\x64\Debug_lib\u3cankvl.lib %BIN%
copy /Y .\Libraries\CANAPI\x64\Debug_lib\uvcankvl.pdb %BIN%\u3cankvl.pdb
copy /Y .\Libraries\KvaserCAN\x64\Debug_lib\uvKvaserCAN.lib %BIN%
copy /Y .\Libraries\KvaserCAN\x64\Debug_lib\KvaserCAN.pdb %BIN%\uvKvaserCAN.pdb
copy /Y .\Sources\CANlib\x64\canlib32.lib %BIN%
echo Static libraries (x64) > %BIN%\readme.txt

echo Copying header files...
set INC=".\Includes"
if not exist %INC% mkdir %INC%
copy /Y .\Sources\KvaserCAN*.h %INC%
copy /Y .\Sources\CANAPI\CANAPI.h %INC%
copy /Y .\Sources\CANAPI\CANAPI_Types.h %INC%
copy /Y .\Sources\CANAPI\CANAPI_Defines.h %INC%
copy /Y .\Sources\CANAPI\can_api.h %INC%

:end
popd
pause
