#!/bin/bash
# start-x-fpgagpu.sh: an X server on fpgagpu (display :1, board B's HDMI) with a few clients, in the background.
#   start-x-fpgagpu.sh [start [glamor]|stop|status]
# glamor: X renders through the fpgagpu Mesa driver on the boards (FPGAGPU_MESA, default /opt/fpgagpu-mesa); without
# it X renders in software and the module copies its changes to board B.
# Needs the fpgagpu module loaded and card1 free of other DRM masters (see 90-no-autoadd-gpu.conf).
# X_WM=<window manager> starts one (openbox, say). DESKFLOW_SERVER=<host[:port]> starts a Deskflow client that takes
# keyboard and mouse from that server (X itself reads no input devices); DESKFLOW_NAME is this screen's name in the
# server's layout (default: the host name); DESKFLOW_TLS=0 for a server without TLS.
set -u
here=$(cd "$(dirname "$0")" && pwd)
run=/run/fpgagpu-x
disp=:1
conf="$here/xorg-fpgagpu.conf"
xenv=""
if [ "${2:-}" = glamor ]; then
	conf="$here/xorg-fpgagpu-glamor.conf"
	xenv="LD_LIBRARY_PATH=${FPGAGPU_MESA:-/opt/fpgagpu-mesa}/lib/x86_64-linux-gnu GLAMOR_DEBUG=${GLAMOR_DEBUG:-0}"
fi

stop() {
	pids=$(cat "$run"/*.pid 2>/dev/null)
	for p in $pids; do kill "$p" 2>/dev/null; done
	for i in $(seq 1 10); do			# X acts on SIGTERM only between requests: then SIGKILL
		alive=""
		for p in $pids; do kill -0 "$p" 2>/dev/null && alive="$alive $p"; done
		[ -z "$alive" ] && break
		sleep 1
	done
	for p in $alive; do kill -9 "$p" 2>/dev/null; done
	rm -f "$run"/*.pid
}

case "${1:-start}" in
start)
	mkdir -p "$run/empty"
	stop
	env $xenv Xorg $disp -config "$conf" -configdir "$run/empty" -nolisten tcp -noreset -novtswitch -sharevts \
		-logfile "$run/Xorg.log" > "$run/xorg.out" 2>&1 &
	echo $! > "$run/xorg.pid"
	for i in $(seq 1 60); do xset -display $disp q > /dev/null 2>&1 && break; sleep 1; done
	xset -display $disp q > /dev/null 2>&1 || { echo "X $disp did not come up; see $run/Xorg.log"; exit 1; }
	DISPLAY=$disp xsetroot -solid '#203050'
	if [ -n "${X_WM:-}" ]; then
		DISPLAY=$disp $X_WM > "$run/wm.out" 2>&1 &
		echo $! > "$run/wm.pid"
	fi
	if [ -n "${DESKFLOW_SERVER:-}" ]; then
		tls="--enable-crypto"
		[ "${DESKFLOW_TLS:-1}" = 0 ] && tls=""
		deskflow-client --no-daemon --name "${DESKFLOW_NAME:-$(hostname)}" $tls --display $disp \
			--log "$run/deskflow.log" "$DESKFLOW_SERVER" > "$run/deskflow.out" 2>&1 &
		echo $! > "$run/deskflow.pid"
	fi
	if [ "${X_CLIENTS:-1}" = 0 ]; then
		echo "X $disp is up on fpgagpu (no clients)"
		exit 0
	fi
	if [ -z "$xenv" ]; then
		DISPLAY=$disp xclock -geometry 200x200+20+20 -update 1 > /dev/null 2>&1 &
		echo $! > "$run/xclock.pid"
		DISPLAY=$disp xterm -geometry 80x24+240+20 -e 'echo "X on fpgagpu: board B, $(uname -r)"; exec bash' \
			> /dev/null 2>&1 &
	else
		# glamor draws core-font text in software (its own text needs GLSL 1.30), each run read back from the board:
		# an Xft font goes through Render's glyphs, on the board
		DISPLAY=$disp xterm -fa Monospace -fs 11 -geometry 70x20+40+40 \
			-e 'echo "X with glamor on fpgagpu: board B, $(uname -r)"; exec bash' > /dev/null 2>&1 &
	fi
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
