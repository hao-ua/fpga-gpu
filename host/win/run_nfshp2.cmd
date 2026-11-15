@echo off
rem run_on_fpga.cmd - Need for Speed Hot Pursuit 2 on the FPGA GPU, from a copy: the install stays as it is.
rem   run_on_fpga.cmd        play; the log is in the copy, fpga_gpu.log
rem   run_on_fpga.cmd dump   also saves every 4th frame as a .bmp in the copy's frames folder (about 2 MB/s): for catching a short artifact
rem   run_on_fpga.cmd record keeps the last 48 frames of draws in memory: press F12 right after an artifact and they go to the copy's rec folder (a few hundred MB,
rem                          the game pauses a moment while it writes). build\x86\d3d9\d3_replay.exe plays such a file on the boards with no game
rem   run_on_fpga.cmd nokeep every vertex is streamed, no buffers kept on board A: tells a fault of keeping from any other
rem   run_on_fpga.cmd check  everything but starting the game: copies it the first time, about 580 MB
rem The game keeps its own settings and saves in Documents\EA Games\Need For Speed Hot Pursuit 2.
rem This file sits beside the game. host\win\run_nfshp2.cmd in the repository is the same file. The game is the one beside this
rem file, else NFS_DIR, else its default install folder. The library is the build in the host\win folder this file is in, else
rem GPU2_REPO's: beside the game, GPU2_REPO must be the repository.
setlocal
set "GAME=%NFS_DIR%"
if exist "%~dp0NFSHP2.exe" set "GAME=%~dp0."
if not defined GAME set "GAME=C:\Program Files (x86)\EA GAMES\Need For Speed Hot Pursuit 2"
set "COPY=%LOCALAPPDATA%\gpu2\nfshp2"
set "WIN=%GPU2_REPO%\host\win"
if exist "%~dp0build.ps1" set "WIN=%~dp0."
set "DLL=%WIN%\build\x86\d3d8\d3d8.dll"

if not exist "%DLL%" (echo no d3d8.dll at %DLL%: build it with host\win\build.ps1 -Arch x86 -Target d3d8, and set GPU2_REPO to the repository & goto fail)
for %%g in (NFSHP2.exe Warlords_og.exe heroes4.exe openarena.exe) do tasklist /fi "imagename eq %%g" | find /i "%%g" >nul && (echo %%g is running and only one program can use the boards: close it first & goto fail)
rem board A: GPU2_IP (an address or a host name), else the address it falls back to without a DHCP lease
set "BOARD=%GPU2_IP%"
if not defined BOARD set "BOARD=192.168.1.10"
ping -n 1 -w 1000 %BOARD% | find "TTL=" >nul || (echo board A does not answer at %BOARD%: are the boards on? & goto fail)

if not exist "%COPY%\NFSHP2.exe" (
    echo copying the game, once, about 580 MB
    robocopy "%GAME%" "%COPY%" /E /NFL /NDL /NJH /NJS /NP >nul
    if not exist "%COPY%\NFSHP2.exe" (echo the copy failed & goto fail)
)
copy /y "%DLL%" "%COPY%\d3d8.dll" >nul || (echo cannot put d3d8.dll in the copy of the game & goto fail)
if /i "%~1"=="check" (
    del "%COPY%\d3d8.dll"
    echo ready: the boards answer, and the copy of the game and d3d8.dll are in place
    exit /b 0
)

set GPU2_LOG=%COPY%\fpga_gpu.log
set GPU2_STATS=3
set GPU2_TRACE_D3D=1
if /i "%~1"=="nokeep" set GPU2_KEEP_MB=0
if /i "%~1"=="dump" (
    if not exist "%COPY%\frames" mkdir "%COPY%\frames"
    del /q "%COPY%\frames\*.bmp" 2>nul
    set "GPU2_DUMP=%COPY%\frames"
    set GPU2_DUMP_EVERY=4
)
if /i "%~1"=="record" (
    if not exist "%COPY%\rec" mkdir "%COPY%\rec"
    set "GPU2_RECORD=%COPY%\rec"
)
echo Need for Speed on the FPGA GPU: look at the HDMI monitor. Keep this window open, it cleans up when the game exits.
pushd "%COPY%"
start "" /wait NFSHP2.exe
popd
del "%COPY%\d3d8.dll"
exit /b 0

:fail
pause
exit /b 1
