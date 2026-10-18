"""Fake Arduino Mega on a pty, speaking the firmware's serial protocol (UI_PLAN §8.4), so the *real*
arduino_reader.py bridge is exercised unchanged: STATE4 telemetry, READY4 + BOOT at boot, ID, ZERO, STOP.
Touch /tmp/fake_arduino.reset to simulate a firmware reboot (new boot id, odometry zeroed)."""
import json, os, pty, random, time, tty

master, slave = pty.openpty()
tty.setraw(master)
os.symlink(os.ttyname(slave), "/tmp/ttyFAKE") if not os.path.exists("/tmp/ttyFAKE") else None
state = {"x": 42.0, "y": -1.0, "th": 1.5, "grip": 40.0, "hand": 0.0, "wrist": 0.0, "elbow": 60.0, "shoulder": 30.0}
stats = {"stops": 0, "zeros": 0, "ids": 0, "boot": None}
rx = b""

def send(line):
    os.write(master, (line + "\n").encode())

def boot():
    stats["boot"] = str(random.randint(1, 0xFFFFFF))
    state.update(x=0.0, y=0.0, th=0.0)
    send("READY4")
    send(f"BOOT {stats['boot']} ZERO")

boot()
state.update(x=42.0, y=-1.0, th=1.5)  # the robot was driven before the console started
last = 0.0
os.set_blocking(master, False)
while True:
    now = time.monotonic()
    if os.path.exists("/tmp/fake_arduino.reset"):
        os.remove("/tmp/fake_arduino.reset")
        state.update(x=17.0)  # odometry somewhere, then the reboot zeroes it
        boot()
    try:
        rx += os.read(master, 4096)
    except (BlockingIOError, OSError):
        pass
    while b"\n" in rx:
        line, rx = rx.split(b"\n", 1)
        cmd = line.strip().decode(errors="replace")
        if cmd == "STOP":
            stats["stops"] += 1
            send("ERROR 0 STOPPED")
        elif cmd == "ID":
            stats["ids"] += 1
            send(f"BOOT {stats['boot']} ZERO")
        elif cmd == "ZERO":
            stats["zeros"] += 1
            state.update(x=0.0, y=0.0, th=0.0)
            send(f"ZEROED {stats['boot']}")
    if now - last >= 0.05:
        last = now
        s = state
        send(f"STATE4 0 {s['x']:.3f} {s['y']:.3f} {s['th']:.3f} {s['grip']:.3f} {s['hand']:.3f} {s['wrist']:.3f} {s['elbow']:.3f} {s['shoulder']:.3f}")
        with open("/tmp/fake_arduino.json.tmp", "w") as f:
            json.dump(dict(stats, **state), f)
        os.replace("/tmp/fake_arduino.json.tmp", "/tmp/fake_arduino.json")
    time.sleep(0.005)
