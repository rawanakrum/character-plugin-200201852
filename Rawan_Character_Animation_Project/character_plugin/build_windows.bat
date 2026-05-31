@echo off
setlocal

REM Build from an x64 Native Tools Command Prompt for Visual Studio.
REM Output: character_plugin_200201852.dll

if not exist build mkdir build

cl /nologo /std:c++17 /O2 /EHs- /GR- /GS- /DARKHEON_NO_CRT /I include /c src\StudentController.cpp /Fo:build\StudentController.obj
if errorlevel 1 exit /b 1

link /nologo /DLL /NOENTRY /MACHINE:X64 /NODEFAULTLIB /SUBSYSTEM:CONSOLE /OUT:character_plugin_200201852.dll /IMPLIB:build\character_plugin_200201852.lib build\StudentController.obj
if errorlevel 1 exit /b 1

dumpbin /exports character_plugin_200201852.dll
