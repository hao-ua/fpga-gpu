@echo off
rem run_halflife.cmd - Half-Life (Steam) on the FPGA GPU, from a copy: the install stays as it is. run_halflife.sh starts this from WSL.
rem   run_halflife.cmd           play at 640 x 480 in a window on the PC; the picture is on board B's HDMI monitor
rem   run_halflife.cmd 800       800 x 600; also 1024 (1024 x 768) and 1280 (1280 x 720): board B's video modes, 640 is the default
rem   run_halflife.cmd full      full screen on the PC instead of a window (the PC's monitor changes mode)
rem   run_halflife.cmd info      only say what the game renders with: starts nothing, copies nothing, needs no boards
rem   run_halflife.cmd check     everything but starting the game: copies it the first time, about 0.55 GB
rem   run_halflife.cmd dump      also saves every 4th frame as a .bmp in the copy's frames folder
rem   run_halflife.cmd nokeep    every vertex is streamed: tells a fault of buffers kept on board A from any other
rem   run_halflife.cmd recopy    copy the game again, after Steam has updated it
rem Words go together: run_halflife.cmd 800 dump. Steam must be running: the copy gets steam_appid.txt (70) so Steam does not start the install.
rem hl.exe imports no graphics library, it loads hw.dll at run time and hw.dll imports opengl32.dll: run_game.ps1 reads hl.exe's own imports
rem only, so it is told -Api opengl. hl.exe deletes an opengl32.dll in its folder at start: run_game.ps1 places ours read-only for that.
rem The copy is %LOCALAPPDATA%\gpu2\halflife, the log is its fpga_gpu.log, saves stay in the copy; the registry settings are put back after the run.
rem The game is the one beside this file if there is one, else HL_DIR, else the Steam library on D:.
rem run_game.ps1 is the one beside this file (host\win), else GPU2_REPO's: a copy of this file beside the game needs GPU2_REPO = the repository.
setlocal
set "RUN=%~dp0run_game.ps1"
if not exist "%RUN%" set "RUN=%GPU2_REPO%\host\win\run_game.ps1"
if exist "%~dp0hl.exe" set "HL_DIR=%~dp0."
if not defined HL_DIR set "HL_DIR=D:\SteamLibrary\steamapps\common\Half-Life"
set "W=640"
set "H=480"
set "SCREEN=-windowed"
set "OPT="
set "PLAY=1"
rem the game stores -w / -h / -windowed (and its video-crash flag) in the registry, for the Steam install too: saved here, put back after the run
set "REGKEY=HKCU\Software\Valve\Half-Life\Settings"
set "SAVED=%LOCALAPPDATA%\gpu2\halflife_settings.reg"

:args
if "%~1"=="" goto args_done
if "%~1"=="640" (set "W=640" & set "H=480" & shift & goto args)
if "%~1"=="800" (set "W=800" & set "H=600" & shift & goto args)
if "%~1"=="1024" (set "W=1024" & set "H=768" & shift & goto args)
if "%~1"=="1280" (set "W=1280" & set "H=720" & shift & goto args)
if /i "%~1"=="full" (set "SCREEN=-full" & shift & goto args)
if /i "%~1"=="window" (set "SCREEN=-windowed" & shift & goto args)
if /i "%~1"=="info" (set "OPT=%OPT% -Info" & set "PLAY=" & shift & goto args)
if /i "%~1"=="check" (set "OPT=%OPT% -Check" & set "PLAY=" & shift & goto args)
if /i "%~1"=="dump" (set "OPT=%OPT% -Dump" & shift & goto args)
if /i "%~1"=="nokeep" (set "OPT=%OPT% -NoKeep" & shift & goto args)
if /i "%~1"=="recopy" (set "OPT=%OPT% -Recopy" & shift & goto args)
echo unknown option %~1: run_halflife.cmd [640^|800^|1024^|1280] [full] [info^|check] [dump] [nokeep] [recopy]
goto fail
:args_done

if not exist "%RUN%" (echo no run_game.ps1 at %RUN%: set GPU2_REPO to the repository & goto fail)
if not exist "%HL_DIR%\hl.exe" (echo no hl.exe in "%HL_DIR%": set HL_DIR to the Half-Life folder & goto fail)
if not exist "%HL_DIR%\hw.dll" (echo no hw.dll in "%HL_DIR%": that is the OpenGL engine hl.exe loads & goto fail)
echo Half-Life at %W% x %H%, %SCREEN%
set "HAD="
if defined PLAY (
    if not exist "%LOCALAPPDATA%\gpu2" mkdir "%LOCALAPPDATA%\gpu2"
    reg export "%REGKEY%" "%SAVED%" /y >nul 2>&1 && set "HAD=1"
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%RUN%" "%HL_DIR%\hl.exe" -Api opengl -Name halflife -GameArgs "%SCREEN% -w %W% -h %H%"%OPT%
set "RC=%errorlevel%"
if defined PLAY (
    if defined HAD (reg import "%SAVED%" >nul 2>&1) else (reg delete "%REGKEY%" /f >nul 2>&1)
    echo Half-Life's own settings in the registry are back as they were before this run
)
if not "%RC%"=="0" goto fail
exit /b 0

:fail
pause
exit /b 1
