@echo off
rem run_on_fpga.cmd - Heroes of Might and Magic IV on the FPGA GPU, from a copy: the install stays as it is.
rem 1024 x 768 x 16, board B's own 1024 x 768 mode (shown 1:1 in the middle of 1280 x 800 on a board B without that mode); the picture is on board B's HDMI monitor.
rem   run_on_fpga.cmd         play; the log is in the copy, fpga_gpu.log
rem   run_on_fpga.cmd check   everything but starting the game: copies it the first time, about 970 MB
rem GOG ships its own ddraw.dll, a wrapper over Direct3D 9, beside the game: it is left out of the copy, or the game would use it and not ours.
rem This file sits beside the game. host\win\run_homm4.cmd in the repository is the same file. The library is the build in the
rem host\win folder this file is in, else GPU2_REPO's: beside the game, GPU2_REPO must be the repository.
setlocal
set "GAME=C:\Program Files\GOG Galaxy\Games\HoMM 4 Complete"
set "COPY=%LOCALAPPDATA%\gpu2\homm4"
set "WIN=%GPU2_REPO%\host\win"
if exist "%~dp0build.ps1" set "WIN=%~dp0."
set "DLL=%WIN%\build\x86\ddraw\ddraw.dll"

if not exist "%DLL%" (echo no ddraw.dll at %DLL%: build it with host\win\build.ps1 -Arch x86 -Target ddraw, and set GPU2_REPO to the repository & goto fail)
for %%g in (heroes4.exe Warlords_og.exe NFSHP2.exe openarena.exe) do tasklist /fi "imagename eq %%g" | find /i "%%g" >nul && (echo %%g is running and only one program can use the boards: close it first & goto fail)
ping -n 1 -w 1000 192.168.1.10 | find "TTL=" >nul || (echo board A does not answer at 192.168.1.10: are the boards on? & goto fail)

if not exist "%COPY%\copied.ok" (
    echo copying the game, once, about 970 MB
    robocopy "%GAME%" "%COPY%" /E /XF ddraw.dll /NFL /NDL /NJH /NJS /NP >nul
    if errorlevel 8 (echo the copy failed & goto fail)
    echo copied>"%COPY%\copied.ok"
)
copy /y "%DLL%" "%COPY%\ddraw.dll" >nul || (echo cannot put ddraw.dll in the copy of the game & goto fail)
if /i "%~1"=="check" (
    del "%COPY%\ddraw.dll"
    echo ready: the boards answer, and the copy of the game and ddraw.dll are in place
    exit /b 0
)

set "GPU2_LOG=%COPY%\fpga_gpu.log"
del /q "%GPU2_LOG%" 2>nul
echo Heroes IV on the FPGA GPU: look at the HDMI monitor. Keep this window open, it cleans up when the game exits.
pushd "%COPY%"
start "" /wait heroes4.exe
popd
del "%COPY%\ddraw.dll"
findstr /i /c:"failed" /c:"unusable" /c:"exception" "%GPU2_LOG%" >nul 2>nul && (
    echo The GPU library reported a problem, see %GPU2_LOG%
    goto fail
)
exit /b 0

:fail
pause
exit /b 1
