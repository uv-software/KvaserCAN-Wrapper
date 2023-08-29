@echo off

pushd
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" x64
popd

pushd
call msbuild.exe .\kvl_testing.vcxproj /t:Clean;Build /p:"Configuration=Debug";"Platform=x64"
if errorlevel 1 goto end

call .\x64\Debug\kvl_testing.exe --gtest_filter=SmokeTest.*

:end
popd
pause
