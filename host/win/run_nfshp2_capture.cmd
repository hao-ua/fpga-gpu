@echo off
rem run_nfshp2_capture.cmd - Need for Speed Hot Pursuit 2 on the FPGA GPU with BOTH captures on (run_nfshp2.cmd's dump + record):
rem   * every 4th presented frame is read back from board B and saved as frame_NNNNN.bmp in %LOCALAPPDATA%\gpu2\nfshp2\frames
rem     (the folder is emptied at start; about 2 MB/s, costs a few fps)
rem   * the last 48 frames of draws stay in memory: press F12 right after a phantom and they go to %LOCALAPPDATA%\gpu2\nfshp2\rec
rem     (a few hundred MB, the game pauses a moment while it writes); build\x86\d3d9\d3_replay.exe replays such a file with no game
rem   run_nfshp2_capture.cmd          play with both captures
rem   run_nfshp2_capture.cmd nokeep   also stream every vertex (no buffers kept on board A)
rem The game keeps its own settings and saves in Documents\EA Games\Need For Speed Hot Pursuit 2. The install is not touched.
rem The game is the one beside this file, else NFS_DIR, else its default install folder. The library is the build in the
rem host\win folder this file is in, else GPU2_REPO's: beside the game, GPU2_REPO must be the repository.
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
ping -n 1 -w 1000 192.168.1.10 | find "TTL=" >nul || (echo board A does not answer at 192.168.1.10: are the boards on? & goto fail)

if not exist "%COPY%\NFSHP2.exe" (
    echo copying the game, once, about 580 MB
    robocopy "%GAME%" "%COPY%" /E /NFL /NDL /NJH /NJS /NP >nul
    if not exist "%COPY%\NFSHP2.exe" (echo the copy failed & goto fail)
)
copy /y "%DLL%" "%COPY%\d3d8.dll" >nul || (echo cannot put d3d8.dll in the copy of the game & goto fail)

set GPU2_LOG=%COPY%\fpga_gpu.log
set GPU2_STATS=3
set GPU2_TRACE_D3D=1
if /i "%~1"=="nokeep" set GPU2_KEEP_MB=0
if not exist "%COPY%\frames" mkdir "%COPY%\frames"
del /q "%COPY%\frames\*.bmp" 2>nul
set "GPU2_DUMP=%COPY%\frames"
set GPU2_DUMP_EVERY=4
if not exist "%COPY%\rec" mkdir "%COPY%\rec"
set "GPU2_RECORD=%COPY%\rec"
echo Need for Speed on the FPGA GPU with captures: look at the HDMI monitor.
echo   frames  -^> %COPY%\frames  (every 4th frame)
echo   F12     -^> %COPY%\rec     (the last 48 frames of draws; press it right after a phantom)
echo Keep this window open, it cleans up when the game exits.
pushd "%COPY%"
start "" /wait NFSHP2.exe
popd
del "%COPY%\d3d8.dll"
echo done: %COPY%\frames and %COPY%\rec hold the captures.
exit /b 0

:fail
pause
exit /b 1
