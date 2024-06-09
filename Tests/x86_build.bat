@echo off

set VCVARS="True"
set NORUN="False"

rem parse arguments: [NOVARS] [NORUN]
:LOOP
if "%1" == "NOVARS" set VCVARS="False"
if "%1" == "NORUN" set NORUN="True"
SHIFT
if not "%1" == "" goto LOOP

rem set MSBuild environment variables
if %VCVARS% == "True" (
   pushd
   call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars32.bat" x86
   popd
)
pushd

rem build the test suites
call msbuild.exe .\kvl_testing.vcxproj /t:Clean;Build /p:"Configuration=Debug";"Platform=Win32"
if errorlevel 1 goto end

rem execute the smoke-test
if %NORUN% == "False" (
   call .\Debug\kvl_testing.exe --gtest_filter=SmokeTest.*
)

rem end of the job
:end
popd
if %VCVARS% == "True" (
   pause
)
