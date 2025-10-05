import socket, json, time, math, sys, termios, tty, select

# ==== CONFIG ====
PICO_IP = "192.168.1.123"   # ← pon la IP del Pico
PORT = 9000
SEND_RATE_HZ = 50           # 20–50 Hz típico
# =================

# Estado (llena z_est, vz_est, yaw_est con tu sistema de visión)
armed = False
z_est, vz_est, yaw_est = 0.0, 0.0, 0.0
z_ref, yaw_ref = 0.3, 0.0

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def getch(timeout=0.0):  # no bloquea; envía SIEMPRE
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r,_,_ = select.select([sys.stdin], [], [], timeout)
        ch = sys.stdin.read(3) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

print("Controles: a=armar, s=desarmar, ↑/↓ z_ref ±0.05 m, ←/→ yaw_ref ±5°, q=salir")
print(f"TX continuo → {PICO_IP}:{PORT}")

try:
    while True:
        # --- aquí integra tus lecturas reales ---
        # z_est, vz_est, yaw_est = get_from_your_vision()

        ch = getch()
        if ch:
            if ch == 'q': break
            elif ch == 'a': armed = True;  print("✅ ARMADO")
            elif ch == 's': armed = False; print("🛑 DESARMADO")
            elif ch == '\x1b[A': z_ref += 0.05; print(f"↑ z_ref = {z_ref:.2f} m")
            elif ch == '\x1b[B': z_ref = max(0.1, z_ref - 0.05); print(f"↓ z_ref = {z_ref:.2f} m")
            elif ch == '\x1b[C': yaw_ref += math.radians(5); print(f"→ yaw_ref = {math.degrees(yaw_ref):.1f}°")
            elif ch == '\x1b[D': yaw_ref -= math.radians(5); print(f"← yaw_ref = {math.degrees(yaw_ref):.1f}°")

        pkt = {
            "armed": armed,
            "z": z_est,
            "vz": vz_est,
            "yaw": yaw_est,
            "z_ref": z_ref,
            "yaw_ref": yaw_ref
        }
        s.sendto(json.dumps(pkt).encode(), (PICO_IP, PORT))

        # print suave (cada ~0.2 s)
        if int(time.time()*5) % 5 == 0:
            print(f"[SEND] armed={armed}  z={z_est:.2f}  vz={vz_est:+.2f}  yaw={math.degrees(yaw_est):+.1f}°  "
                  f"z_ref={z_ref:.2f}  yaw_ref={math.degrees(yaw_ref):+.1f}°")
        time.sleep(1.0 / SEND_RATE_HZ)

except KeyboardInterrupt:
    pass
finally:
    s.close()
