#!/bin/bash
# start-x-fpgagpu.sh: an X server on fpgagpu (display :1, board B's HDMI) with a few clients, in the background.
#   start-x-fpgagpu.sh [start|stop|status]
# Needs the fpgagpu module loaded and card1 free of other DRM masters (see 90-no-autoadd-gpu.conf).
set -u
here=$(cd "$(dirname "$0")" && pwd)
run=/run/fpgagpu-x
disp=:1

stop() {
	for p in $(cat "$run"/*.pid 2>/dev/null); do kill "$p" 2>/dev/null; done
	rm -f "$run"/*.pid
}

case "${1:-start}" in
start)
	mkdir -p "$run/empty"
	stop
	Xorg $disp -config "$here/xorg-fpgagpu.conf" -configdir "$run/empty" -nolisten tcp -noreset -novtswitch -sharevts \
		-logfile "$run/Xorg.log" > "$run/xorg.out" 2>&1 &
	echo $! > "$run/xorg.pid"
	for i in $(seq 1 60); do xset -display $disp q > /dev/null 2>&1 && break; sleep 1; done
	xset -display $disp q > /dev/null 2>&1 || { echo "X $disp did not come up; see $run/Xorg.log"; exit 1; }
	DISPLAY=$disp xsetroot -solid '#203050'
	DISPLAY=$disp xclock -geometry 200x200+20+20 -update 1 > /dev/null 2>&1 &
	echo $! > "$run/xclock.pid"
	DISPLAY=$disp xterm -geometry 80x24+240+20 -e 'echo "X on fpgagpu: board B, $(uname -r)"; exec bash' > /dev/null 2>&1 &
	echo $! > "$run/xterm.pid"
	echo "X $disp is up on fpgagpu"
	;;
stop)
	stop
	echo "stopped"
	;;
status)
	for f in "$run"/*.pid; do [ -f "$f" ] && echo "$(basename "$f" .pid): $(kill -0 "$(cat "$f")" 2>/dev/null && echo running || echo gone)"; done
	;;
esac
