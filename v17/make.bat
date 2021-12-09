@echo off
SET PKG_CONFIG_PATH=c:\msys64\mingw64\lib\pkgconfig
set p=;c:/msys64/mingw64/bin/;c:/msys64/usr/bin/
if not defined alreadysetp  SET PATH=%PATH%%p%
SET alreadysetp=1
c:/msys64/mingw64/bin/mingw32-make.exe %1 %2 %3 %4 %5 %6 %7 %8 %9
