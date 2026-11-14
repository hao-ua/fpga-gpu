@echo off
rem run_game.cmd - any game on the FPGA GPU: run_game.cmd "C:\path\game.exe" [-Info] [-Check] [-Root folder] [-Api name] ...  run_game.ps1 says what each option does.
rem The .exe's import table picks the library. -Info only reports that, and starts nothing.
if "%~1"=="" (
    echo usage: run_game.cmd "C:\path\game.exe" [-Info] [-Check] [-Root folder] [-Api ddraw^|opengl^|gles^|d3d8^|d3d9] [-GameArgs "..."] [-Name folder] [-Recopy] [-Dump] [-Record] [-NoKeep]
    pause
    exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_game.ps1" %*
if errorlevel 1 pause
exit /b %errorlevel%
