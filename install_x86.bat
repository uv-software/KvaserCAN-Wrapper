@echo off
set PWD="%~dp0"
pushd
cd /D %PWD%
copy /Y .\Libraries\CANAPI\Release_dll\u3cankvl.dll C:\Windows\SysWOW64
copy /Y .\Libraries\UVPCAN\Release_dll\uvKvaserCAN.dll C:\Windows\SysWOW64
popd
dir C:\Windows\SysWOW64\u*can*.dll
pause
