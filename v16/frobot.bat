@echo off
set p=;c:/msys64/mingw64/bin/
if not defined alreadysetp  SET PATH=%PATH%%p%
SET alreadysetp=1
frobot.exe %1 %2 %3 %4 %5 %6 %7 %8 %9
