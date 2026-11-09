@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
echo CHECKING CL:
where cl
cd /d c:\dev\opt\longest-path-optil
cl /EHsc /O2 /std:c++17 /Fe:gen.exe gen.cpp
cl /EHsc /O2 /std:c++17 /Fe:validate.exe validate.cpp
cl /EHsc /O2 /std:c++17 /Fe:solution.exe solution.cpp
