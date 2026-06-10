#!/usr/bin/env python3
"""find_drones.py — passive HULA discovery without the `dola` tool.

The HULA drones broadcast plane-status packets over UDP (pyhulax config:
udp_status_port=8668, udp_command_port=8085). `dola` finds drones by
listening for those broadcasts and reading the sender IPs. This does the
same thing: it binds the candidate UDP ports and prints every source IP it
hears from. It SENDS NOTHING to the drones — purely passive.

Run:  python find_drones.py [seconds]
"""
import socket
import sys
import time

LISTEN_PORTS = [8668, 8085, 8688, 8400]   # status, command, optitrack, control
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

socks = []
for port in LISTEN_PORTS:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except OSError:
        pass
    try:
        s.bind(("0.0.0.0", port))
        s.setblocking(False)
        socks.append((port, s))
        print(f"[listen] bound UDP 0.0.0.0:{port}")
    except OSError as e:
        print(f"[listen] could NOT bind UDP {port}: {e}")

print(f"[listen] listening {DURATION:.0f}s for HULA broadcasts... "
      f"(drones must be powered on & on this machine's network)")

seen = {}   # ip -> {"ports": set, "count": int, "last_bytes": int}
end = time.time() + DURATION
while time.time() < end:
    got_any = False
    for port, s in socks:
        try:
            data, addr = s.recvfrom(65535)
        except (BlockingIOError, OSError):
            continue
        got_any = True
        ip = addr[0]
        rec = seen.setdefault(ip, {"ports": set(), "count": 0, "last_bytes": 0})
        rec["ports"].add(port)
        rec["count"] += 1
        rec["last_bytes"] = len(data)
    if not got_any:
        time.sleep(0.05)

print("\n=== discovered senders ===")
if not seen:
    print("  (nothing heard — see notes below)")
for ip in sorted(seen):
    r = seen[ip]
    print(f"  {ip:<16} packets={r['count']:<4} ports={sorted(r['ports'])} "
          f"last_len={r['last_bytes']}")
print(f"\n{len(seen)} unique sender IP(s).")
if seen:
    print("If these look like the drones, fly with:")
    print("  --ips " + ",".join(sorted(seen)))
