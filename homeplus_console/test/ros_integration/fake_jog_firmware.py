#!/usr/bin/env python3
"""A pty that speaks the STATE4 streaming protocol closely enough to exercise arduino_reader.py's
live-jog path without hardware: STR packets with CRC, sequence and final flag, RX4 credits every
32 bytes, BUF3 credits with STATE4 every 50 ms, linear interpolation by the firmware clock,
BUFFER_UNDERRUN, settle-then-DONE, the 3 deg / 2 deg / 6 cm tracking trip, HOST_LOST and STOP.

Joint dynamics are first order with the firmware's speed caps, so a stream that moves faster than
the "motors" trips TRACKING_ERROR the way the real board does.

    python3 fake_jog_firmware.py            # prints the pty path, then serves until killed
    python3 fake_jog_firmware.py --log /tmp/fw.log   # also log every received line
"""
import argparse
import binascii
import os
import pty
import select
import sys
import time
import tty

# hardware units: x_cm y_cm theta_deg grip hand wrist elbow shoulder
RATE = [12.0, 12.0, 25.0, 180.0, 170.0, 45.0, 40.0, 30.0]   # what the "motors" can do per second
TRACK = {0: 6.0, 1: 6.0, 2: 2.0, 6: 3.0, 7: 3.0}              # firmware tracking limits
SETTLE = [0.2, 0.2, 0.2, 0.5, 0.5, 0.5, 0.5, 0.5]


class Fw:
    def __init__(self, write, log):
        self.write, self.log = write, log
        self.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target = list(self.pos)
        self.queue = []          # [(values10, t_ms)]
        self.idx = 0
        self.active = 0
        self.stream = False
        self.final = True
        self.seq = 0
        self.t_start = 0.0
        self.settled_since = None
        self.last_host = time.monotonic()
        self.buf = b''
        self.last_state = 0.0

    def now_ms(self):
        return (time.monotonic() - self.t_start) * 1000.0

    def err(self, reason):
        self.write(f'ERROR {self.active} {reason}\n')

    def stop_here(self):
        self.target = list(self.pos)
        self.queue, self.idx, self.stream, self.settled_since = [], 0, False, None

    def feed(self, data):
        for b in data:
            c = bytes([b])
            if c == b'\n':
                line = self.buf.decode('ascii', 'replace')
                self.buf = b''
                if line:
                    self.last_host = time.monotonic()
                    self.log(line)
                    self.handle(line)
            elif c != b'\r':
                self.buf += c
                if len(self.buf) % 32 == 0 and self.buf.startswith(b'STR'):
                    self.write(f'RX4 {len(self.buf)}\n')

    def handle(self, line):
        if line == 'PING' or line == 'ID':
            if line == 'ID':
                self.write('BOOT 4242 ZERO\n')
            return
        if line == 'STOP':
            self.stop_here()
            self.err('STOPPED')
            return
        if not line.startswith('STR '):
            if self.queue:
                self.write('BUSY2\n')
            return
        _, cid, crc, body = line.split(' ', 3)
        cid = int(cid)
        if self.queue and (not self.stream or cid != self.active):
            self.write('BUSY2\n')
            return
        first = not self.queue
        if first:
            self.active = cid
        if binascii.crc_hqx(body.encode('ascii'), 0xffff) != int(crc, 16):
            self.stop_here(); self.err('CRC_MISMATCH'); return
        seq, flag, pts = body.split(' ', 2)
        seq, flag = int(seq), int(flag)
        if seq != (0 if first else self.seq) or (not first and self.final):
            self.stop_here(); self.err('STREAM_SEQUENCE'); return
        final = bool(flag & 1)
        if not first:
            drop = max(0, self.idx - 1)
            self.queue, self.idx = self.queue[drop:], self.idx - drop
        else:
            self.queue, self.idx = [], 0
        for p in pts.split(','):
            v = [float(x) for x in p.split()]
            if len(v) != 11:
                self.stop_here(); self.err('INCORRECT_INPUT_COUNT'); return
            if v[6] < -35 or v[6] > 140 or v[7] < -30 or v[7] > 150 or not 0 <= v[3] <= 90:
                self.stop_here(); self.err('JOINT_LIMIT'); return
            if v[10] > 120000 or (self.queue and v[10] <= self.queue[-1][1]) or (not self.queue and v[10] != 0):
                self.stop_here(); self.err('BAD_TIMING'); return
            self.queue.append((v[:8], v[10]))
        if len(self.queue) > 45:
            self.stop_here(); self.err('BUFFER_FULL'); return
        if first and not final and len(self.queue) < 2:
            self.stop_here(); self.err('EMPTY_STREAM'); return
        self.stream, self.final, self.seq = True, final, seq + 1
        if first:
            self.target = list(self.queue[0][0]); self.idx = 1
            self.t_start = time.monotonic(); self.settled_since = None
        self.write(f'ACK {self.active} BUFFERED\n')

    def step(self, dt):
        now = time.monotonic()
        for i in range(8):
            d = self.target[i] - self.pos[i]
            m = RATE[i] * dt
            self.pos[i] += max(-m, min(m, d))
        if self.queue and now - self.last_host > 1.5:
            self.stop_here(); self.err('HOST_LOST')
        if self.queue:
            for i, lim in TRACK.items():
                if abs(self.target[i] - self.pos[i]) > lim:
                    e = [self.target[k] - self.pos[k] for k in range(8)]
                    self.stop_here()
                    self.write(f'ERROR {self.active} TRACKING_ERROR dx_cm={e[0]:.3f} dy_cm={e[1]:.3f} '
                               f'yaw_deg={e[2]:.3f} elbow_deg={e[6]:.3f} shoulder_deg={e[7]:.3f}\n')
                    break
        if self.queue:
            el = self.now_ms()
            while self.idx < len(self.queue) and el >= self.queue[self.idx][1]:
                self.idx += 1
            if self.idx < len(self.queue):
                (a, ta), (b, tb) = self.queue[self.idx - 1], self.queue[self.idx]
                u = (el - ta) / (tb - ta)
                self.target = [x + u * (y - x) for x, y in zip(a, b)]
                self.settled_since = None
            elif self.stream and not self.final:
                self.stop_here(); self.err('BUFFER_UNDERRUN')
            else:
                self.target = list(self.queue[-1][0])
                ok = all(abs(self.target[i] - self.pos[i]) <= SETTLE[i] for i in range(8))
                if ok and self.settled_since is None:
                    self.settled_since = now
                if not ok:
                    self.settled_since = None
                if self.settled_since and now - self.settled_since >= 0.25:
                    self.state()
                    self.write(f'DONE {self.active}\n')
                    self.queue, self.idx, self.stream = [], 0, False
        if now - self.last_state >= 0.05:
            self.last_state = now
            self.state()

    def state(self):
        p = self.pos
        self.write(f'STATE4 {self.active} ' + ' '.join(f'{v:.3f}' for v in p) + '\n')
        if self.stream:
            retained = len(self.queue) - max(0, self.idx - 1)
            self.write(f'BUF3 {self.active} {self.seq} {45 - retained}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default=None)
    ap.add_argument('--link', default=None, help='symlink the pty here')
    args = ap.parse_args()
    master, slave = pty.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    if args.link:
        try:
            os.unlink(args.link)
        except FileNotFoundError:
            pass
        os.symlink(path, args.link)
    print(path, flush=True)
    logf = open(args.log, 'a') if args.log else None

    def log(line):
        if logf:
            logf.write(f'{time.monotonic():.3f} {line}\n'); logf.flush()

    fw = Fw(lambda s: os.write(master, s.encode()), log)
    fw.write('READY4\n')
    last = time.monotonic()
    while True:
        r, _, _ = select.select([master], [], [], 0.002)
        if r:
            try:
                fw.feed(os.read(master, 4096))
            except OSError:
                time.sleep(0.01)
        now = time.monotonic()
        fw.step(now - last)
        last = now


if __name__ == '__main__':
    sys.exit(main())
