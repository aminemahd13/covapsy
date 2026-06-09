"""TCP client that sends START/STOP/STATUS to race.py.

  python3 covapsy_ctl.py start                  # on the Pi (localhost)
  python3 covapsy_ctl.py stop
  python3 covapsy_ctl.py status
  python3 covapsy_ctl.py start --host 192.168.1.50   # from the trackside laptop

SSH-free fallback on the Pi: `touch /tmp/covapsy/start` or `.../stop`.
"""

import argparse
import socket
import sys

DEFAULT_PORT = 5005


def main() -> int:
    ap = argparse.ArgumentParser(description="COVAPSY remote start/stop")
    ap.add_argument("command", choices=["start", "stop", "status", "quit"])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    a = ap.parse_args()
    try:
        with socket.create_connection((a.host, a.port), timeout=3.0) as s:
            s.sendall((a.command.upper() + "\n").encode("ascii"))
            print(s.recv(128).decode("ascii", "replace").strip())
    except OSError as e:
        print(f"error: {e}  (is race.py running on {a.host}:{a.port}?)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
