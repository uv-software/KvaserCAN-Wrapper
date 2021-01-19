@echo off

pushd
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars32.bat" x86
popd

pushd
call build_no.bat
rem type .\Sources\build_no.h
rem pause

call msbuild.exe .\Trial\kvl_test.vcxproj /t:Clean;Build /p:"Configuration=Debug";"Platform=Win32"
if errorlevel 1 goto end

call msbuild.exe .\Libraries\CANAPI\uvcankvl.vcxproj /t:Clean;Build /p:"Configuration=Release";"Platform=Win32"
if errorlevel 1 goto end

echo Copying artifacts...
set BIN=".\Binaries"
if not exist %BIN% mkdir %BIN%
set BIN=".\Binaries\x86"
if not exist %BIN% mkdir %BIN%
copy .\Libraries\CANAPI\Release\u3cankvl.dll %BIN%
copy .\Libraries\CANAPI\Release\u3cankvl.lib %BIN%

set INC=".\Includes"
if not exist %INC% mkdir %INC%
copy .\Sources\PCAN.h %INC%
copy .\Sources\CANAPI\can_api.h %INC%
copy .\Sources\CANAPI\CANAPI*.h %INC%

:end
popd
pause
