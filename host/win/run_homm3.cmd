@echo off
rem run_on_fpga.cmd - Heroes Chronicles: Warlords of the Wasteland on the FPGA GPU, from a copy: the install stays as it is.
rem This is the HoMM3 engine, and the only HoMM3 on this PC. 800 x 600 x 16; the picture is on board B's HDMI monitor.
rem   run_on_fpga.cmd         play; the log is in the copy, fpga_gpu.log
rem   run_on_fpga.cmd check   everything but starting the game: copies it the first time, about 490 MB
rem Warlords_og.exe is the one that imports DDRAW.dll: GOG's Warlords.exe uses its own xdd.dll and would never reach ours.
rem The game reads ..\data and ..\mp3, so the whole install is copied.
rem This file sits beside the game. host\win\run_homm3.cmd in the repository is the same file. The library is the build in the
rem host\win folder this file is in, else GPU2_REPO's: beside the game, GPU2_REPO must be the repository.
setlocal
set "GAME=C:\GOG Games\HC1 - Warlords of the Wasteland"
set "COPY=%LOCALAPPDATA%\gpu2\homm3"
set "RUNDIR=%COPY%\Warlords of the Wasteland"
set "WIN=%GPU2_REPO%\host\win"
if exist "%~dp0build.ps1" set "WIN=%~dp0."
set "DLL=%WIN%\build\x86\ddraw\ddraw.dll"

if not exist "%DLL%" (echo no ddraw.dll at %DLL%: build it with host\win\build.ps1 -Arch x86 -Target ddraw, and set GPU2_REPO to the repository & goto fail)
for %%g in (Warlords_og.exe heroes4.exe NFSHP2.exe openarena.exe) do tasklist /fi "imagename eq %%g" | find /i "%%g" >nul && (echo %%g is running and only one program can use the boards: close it first & goto fail)
ping -n 1 -w 1000 192.168.1.10 | find "TTL=" >nul || (echo board A does not answer at 192.168.1.10: are the boards on? & goto fail)

if not exist "%COPY%\copied.ok" (
    echo copying the game, once, about 490 MB
    robocopy "%GAME%" "%COPY%" /E /NFL /NDL /NJH /NJS /NP >nul
    if errorlevel 8 (echo the copy failed & goto fail)
    echo copied>"%COPY%\copied.ok"
)
copy /y "%DLL%" "%RUNDIR%\ddraw.dll" >nul || (echo cannot put ddraw.dll in the copy of the game & goto fail)
if /i "%~1"=="check" (
    del "%RUNDIR%\ddraw.dll"
    echo ready: the boards answer, and the copy of the game and ddraw.dll are in place
    exit /b 0
)

set "GPU2_LOG=%RUNDIR%\fpga_gpu.log"
del /q "%GPU2_LOG%" 2>nul
echo Heroes Chronicles on the FPGA GPU: look at the HDMI monitor. Keep this window open, it cleans up when the game exits.
pushd "%RUNDIR%"
start "" /wait Warlords_og.exe
popd
del "%RUNDIR%\ddraw.dll"
findstr /i /c:"failed" /c:"unusable" /c:"exception" "%GPU2_LOG%" >nul 2>nul && (
    echo The GPU library reported a problem, see %GPU2_LOG%
    goto fail
)
exit /b 0

:fail
pause
exit /b 1
