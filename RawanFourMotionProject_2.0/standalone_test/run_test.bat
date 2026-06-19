@echo off
setlocal
cd /d %~dp0
cl /EHsc /std:c++17 RawanStandaloneMotionTest.cpp /Fe:RawanStandaloneMotionTest.exe
RawanStandaloneMotionTest.exe
endlocal
