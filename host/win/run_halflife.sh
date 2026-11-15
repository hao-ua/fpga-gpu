#!/bin/bash
# run_halflife.sh - Half-Life on the FPGA GPU from WSL: starts run_halflife.cmd beside this file on Windows, with the same words
#   host/win/run_halflife.sh              640 x 480 in a window; the picture is on board B's HDMI monitor
#   host/win/run_halflife.sh 800 dump     800 x 600, every 4th frame saved      (words: 640 800 1024 1280 full info check dump nokeep recopy)
#   HL_DIR=/mnt/d/Games/Half-Life host/win/run_halflife.sh     another install: a WSL or a Windows path
# The game runs on Windows, Steam must be running there. run_halflife.cmd says what each word does.
here=$(cd "$(dirname "$0")" && pwd)
if [ -n "$HL_DIR" ]; then
    case $HL_DIR in
        /*) export WSLENV="${WSLENV:+$WSLENV:}HL_DIR/p" ;;     # /p: Windows gets it as a Windows path (X:\...)
        *)  export WSLENV="${WSLENV:+$WSLENV:}HL_DIR" ;;
    esac
fi
case $here in
    /mnt/[a-z]/*) ;;
    *) echo "run_halflife.sh must stay in the Windows checkout (/mnt/c/...): cmd.exe cannot start in $here" >&2; exit 1 ;;
esac
cd "$here" || exit 1
exec cmd.exe /c '.\run_halflife.cmd' "$@"     # .\ : with NoDefaultCurrentDirectoryInExePath set cmd does not look in the current folder
