#!/usr/bin/env python3
"""make_diagrams.py -- generates the per-stage bring-up wiring diagrams in
docs/bringup/img/ (plain SVG, no dependencies beyond Python 3).

    python3 docs/bringup/make_diagrams.py

Each stage diagram shows ONLY the wires that stage needs. Wire IDs (I1, S1,
E3, P5, ...) match the connection tables in docs/icd/*.md and the full
harness in docs/wiring/tvc_harness.yml. If you move a wire, change the ICD,
docs/pinout.md, config.h and this file together, then re-run it.
"""
from pathlib import Path
from xml.sax.saxutils import escape

OUT = Path(__file__).resolve().parent / "img"

# Wire colors by signal type (also used in the legend)
WIRE = {
    "gnd":  ("#222222", "GND"),
    "3v3":  ("#f08c00", "3.3 V"),
    "5v":   ("#d6336c", "5 V"),
    "12v":  ("#e03131", "+12 V"),
    "sda":  ("#2f9e44", "I2C SDA"),
    "scl":  ("#c9a227", "I2C SCL"),
    "tx":   ("#1c7ed6", "UART (Teensy TX)"),
    "rx":   ("#7048e8", "UART (Teensy RX)"),
    "pwm":  ("#0c8599", "PWM throttle"),
    "usb":  ("#868e96", "USB"),
    "acl":  ("#8b5a2b", "AC line (L)"),
    "acn":  ("#1864ab", "AC neutral (N)"),
    "pe":   ("#5c940d", "Earth (PE)"),
    "mot":  ("#495057", "motor phase"),
    "bus":  ("#f59f00", "servo bus cable"),
    "jump": ("#e8590c", "jumper"),
}

PIN_DY = 26     # vertical spacing between pins
HEAD_H = 48     # component title area


class Diagram:
    def __init__(self, name, w, h, title, subtitle):
        self.name, self.w, self.h = name, w, h
        self.parts = []
        self.pins = {}
        self.used_wires = []
        self.parts.append(f'<rect width="{w}" height="{h}" fill="#ffffff"/>')
        self.text(24, 36, title, 22, weight="700")
        self.text(24, 60, subtitle, 13, fill="#495057")

    # -------------------------------------------------------------- basics
    def text(self, x, y, s, size=12, weight="400", fill="#212529", anchor="start", italic=False):
        style = ' font-style="italic"' if italic else ""
        self.parts.append(
            f'<text x="{x}" y="{y}" font-family="Helvetica, Arial, sans-serif" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}"{style}>{escape(s)}</text>')

    def comp(self, key, x, y, w, title, sub="", left=(), right=(), tint="#f1f3f5", nc=()):
        """A part. left/right = pin names on each edge, top to bottom.
        nc = pin names to draw grey (not connected in this stage)."""
        n = max(len(left), len(right), 1)
        h = HEAD_H + n * PIN_DY + 8
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{tint}" '
                          f'stroke="#343a40" stroke-width="1.5"/>')
        self.text(x + w / 2, y + 20, title, 14, "700", anchor="middle")
        if sub:
            self.text(x + w / 2, y + 37, sub, 11, fill="#495057", anchor="middle")
        for side, names in (("L", left), ("R", right)):
            for i, name in enumerate(names):
                py = y + HEAD_H + i * PIN_DY + PIN_DY / 2
                px = x if side == "L" else x + w
                grey = name in nc
                col = "#adb5bd" if grey else "#343a40"
                self.parts.append(f'<circle cx="{px}" cy="{py}" r="4" fill="#ffffff" stroke="{col}" stroke-width="1.5"/>')
                tx = x + 10 if side == "L" else x + w - 10
                self.text(tx, py + 4, name, 12, "600" if not grey else "400",
                          fill="#adb5bd" if grey else "#212529", anchor="start" if side == "L" else "end")
                self.pins[(key, name)] = (px, py, side)
        return h

    def wire(self, a, b, kind, label="", via=(), stub=24, stub_b=None, width=None, label_at=0, dash=False):
        """Orthogonal wire from pin a to pin b. via = intermediate (x, y)
        corner points; with no via, it routes out, across at the midpoint x,
        and in. label goes on segment number label_at (0 = first)."""
        color = WIRE[kind][0]
        if kind not in self.used_wires:
            self.used_wires.append(kind)
        ax, ay, aside = self.pins[a]
        bx, by, bside = self.pins[b]
        stub_b = stub if stub_b is None else stub_b
        a2 = (ax - stub if aside == "L" else ax + stub, ay)
        b2 = (bx - stub_b if bside == "L" else bx + stub_b, by)
        if via:
            mids = list(via)
        else:
            mx = (a2[0] + b2[0]) / 2
            mids = [(mx, ay), (mx, by)]
        pts = [(ax, ay), a2] + mids + [b2, (bx, by)]
        # make every segment horizontal or vertical
        clean = [pts[0]]
        for p in pts[1:]:
            q = clean[-1]
            if p[0] != q[0] and p[1] != q[1]:
                clean.append((p[0], q[1]))
            clean.append(p)
        d = " ".join(f"{p[0]:.1f},{p[1]:.1f}" for p in clean)
        wdt = width or (5 if kind in ("12v", "usb", "mot") else 3)
        dash_attr = ' stroke-dasharray="7,5"' if dash else ""
        self.parts.append(f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{wdt}" '
                          f'stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>')
        if label:
            # put the label on the longest horizontal segment at/after label_at
            segs = [(clean[i], clean[i + 1]) for i in range(len(clean) - 1)]
            horiz = [s for s in segs[label_at:] if s[0][1] == s[1][1]] or segs
            p, q = max(horiz, key=lambda s: abs(s[1][0] - s[0][0]))
            lx, ly = (p[0] + q[0]) / 2, p[1]
            tw = len(label) * 6.4 + 10
            self.parts.append(f'<rect x="{lx - tw / 2:.1f}" y="{ly - 9:.1f}" width="{tw:.1f}" height="17" rx="4" '
                              f'fill="#ffffff" stroke="{color}" stroke-width="1"/>')
            self.text(lx, ly + 4, label, 11, "600", fill="#212529", anchor="middle")

    def cross(self, pin, text):
        """Mark a pin that must stay DISCONNECTED (e.g. the ESC's BEC wire)."""
        px, py, side = self.pins[pin]
        x2 = px + 30 if side == "R" else px - 30
        self.parts.append(f'<line x1="{px}" y1="{py}" x2="{x2}" y2="{py}" stroke="#e03131" stroke-width="3"/>')
        for dx in (-6, 6):
            self.parts.append(f'<line x1="{x2 - 6}" y1="{py - 6}" x2="{x2 + 6}" y2="{py + 6}" stroke="#e03131" stroke-width="3"/>')
            self.parts.append(f'<line x1="{x2 - 6}" y1="{py + 6}" x2="{x2 + 6}" y2="{py - 6}" stroke="#e03131" stroke-width="3"/>')
            break
        tx = x2 + 12 if side == "R" else x2 - 12
        self.text(tx, py + 4, text, 11, "700", fill="#e03131", anchor="start" if side == "R" else "end")

    def probe(self, x, y, text):
        """A multimeter reading to take at this point."""
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="13" fill="#fff3bf" stroke="#e67700" stroke-width="2"/>')
        self.text(x, y + 5, "V", 13, "700", fill="#e67700", anchor="middle")
        self.text(x + 20, y + 5, text, 11, "700", fill="#e67700")

    def note(self, x, y, w, lines, kind="info"):
        fill, stroke, head = {"info": ("#e7f5ff", "#1c7ed6", "NOTE"),
                              "warn": ("#fff5f5", "#e03131", "WARNING"),
                              "pass": ("#ebfbee", "#2f9e44", "PASS WHEN")}[kind]
        h = 28 + 17 * len(lines)
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6" fill="{fill}" '
                          f'stroke="{stroke}" stroke-width="1.5"/>')
        self.text(x + 12, y + 19, head, 12, "700", fill=stroke)
        for i, s in enumerate(lines):
            self.text(x + 12, y + 37 + 17 * i, s, 12)
        return h

    def legend(self, x, y):
        self.text(x, y, "WIRE COLORS", 11, "700", fill="#495057")
        for i, kind in enumerate(self.used_wires):
            col, name = WIRE[kind]
            yy = y + 16 + i * 18
            self.parts.append(f'<line x1="{x}" y1="{yy}" x2="{x + 28}" y2="{yy}" stroke="{col}" stroke-width="4"/>')
            self.text(x + 36, yy + 4, name, 11)

    def save(self):
        OUT.mkdir(exist_ok=True)
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
               f'viewBox="0 0 {self.w} {self.h}">\n' + "\n".join(self.parts) + "\n</svg>\n")
        (OUT / f"{self.name}.svg").write_text(svg)
        print("wrote", OUT / f"{self.name}.svg")


# =====================================================================
# Reusable parts (same positions/pins in every diagram they appear in)
# =====================================================================
def pi_and_usb(d, x, y, teensy_usb_pin, label="USB"):
    d.comp("PI", x, y, 170, "Raspberry Pi 5", "own 5 V 5 A USB-C supply", right=["USB-A"], tint="#fff0f6")
    d.wire(("PI", "USB-A"), teensy_usb_pin, "usb", label, stub=30)


TEENSY_NOTE = ["Teensy 4.0 (USB end up): right row from the top is",
               "VIN, GND, 3.3V, 23, 22, 21, 20, 19, 18, ...; left row",
               "is GND, 0, 1, 2, ... (check the PJRC pinout card)."]


# =====================================================================
# STAGE 1 -- Pi + Teensy over USB only
# =====================================================================
def stage1():
    d = Diagram("stage1_usb", 900, 360, "Stage 1 -- Pi <-> Teensy over USB",
                "Nothing else connected. Proves: Pi setup, compile + flash from the Pi, USB serial.")
    d.comp("T", 520, 110, 200, "Teensy 4.0", "flash: usb_hello", left=["USB (micro)"], right=["13 LED (onboard)"],
           tint="#e6fcf5")
    pi_and_usb(d, 80, 110, ("T", "USB (micro)"), "USB data cable")
    d.note(80, 220, 520, ["The LED blinks once per second.",
                          "Pi shows the Teensy as /dev/ttyACM0.",
                          "Typing 'ping' in tvc_logger.py answers '# pong'."], "pass")
    d.note(620, 220, 260, ["Cheap micro-USB cables are often", "power-only: if no ttyACM0 appears,", "try another cable first."], "info")
    d.legend(760, 110)
    d.save()


# =====================================================================
# STAGE 2 -- IMU
# =====================================================================
def stage2():
    d = Diagram("stage2_imu", 1100, 490, "Stage 2 -- IMU (GY-521) on the Teensy",
                "4 wires. Wire IDs = docs/icd/imu_i2c.md. Flash: firmware/tests/imu_test")
    d.comp("T", 400, 100, 210, "Teensy 4.0", "3.3 V logic, NOT 5 V tolerant",
           left=["USB (micro)"], right=["3.3V", "GND", "19 (SCL)", "18 (SDA)", "3 (INT)"], tint="#e6fcf5", nc=["3 (INT)"])
    d.comp("IMU", 780, 100, 190, "GY-521 (MPU-6050)", "I2C address 0x68",
           left=["VCC", "GND", "SCL", "SDA", "INT"], right=["AD0  (leave open)", "XDA  (open)", "XCL  (open)"],
           tint="#fff9db", nc=["INT", "AD0  (leave open)", "XDA  (open)", "XCL  (open)"])
    pi_and_usb(d, 60, 100, ("T", "USB (micro)"))
    d.wire(("T", "3.3V"), ("IMU", "VCC"), "3v3", "I1")
    d.wire(("T", "GND"), ("IMU", "GND"), "gnd", "I2")
    d.wire(("T", "19 (SCL)"), ("IMU", "SCL"), "scl", "I3")
    d.wire(("T", "18 (SDA)"), ("IMU", "SDA"), "sda", "I4")
    d.note(60, 310, 520, TEENSY_NOTE + ["Power the IMU from 3.3V, NOT VIN/5V: that keeps its", "I2C pull-ups at a Teensy-safe 3.3 V."], "warn")
    d.note(600, 310, 470, ["Boot shows 'device at 0x68' and WHO_AM_I.",
                           "Flat and still: |accel| ~ 1.00 g, gyro ~ 0 deg/s.",
                           "Tilt the way the rocket tips: angle_x (or angle_y)",
                           "follows, POSITIVE when the nose tips toward +x.",
                           "'n' noise check: angle std < ~0.1 deg."], "pass")
    d.legend(60, 200)
    d.save()


# =====================================================================
# STAGE 3 -- level shifter loopback (no BusLinker yet)
# =====================================================================
def stage3():
    d = Diagram("stage3_shifter", 1100, 520, "Stage 3 -- Level shifter loopback (no BusLinker, no servo)",
                "Wire IDs = docs/icd/servo_bus.md. The orange jumper stands in for the BusLinker. Flash: firmware/tests/uart_loopback_test")
    d.comp("T", 330, 100, 210, "Teensy 4.0", "3.3 V logic",
           left=["USB (micro)"], right=["1 (TX1)", "0 (RX1)", "3.3V", "GND", "VIN (5 V from USB)"], tint="#e6fcf5")
    d.comp("LS", 690, 100, 210, "BSS138 level shifter", "4-channel, 3.3 V <-> 5 V",
           left=["LV2", "LV1", "LV", "GND"], right=["HV2", "HV1", "HV", "GND "], tint="#f3f0ff", nc=["GND "])
    pi_and_usb(d, 40, 100, ("T", "USB (micro)"))
    d.wire(("T", "1 (TX1)"), ("LS", "LV2"), "tx", "S1")
    d.wire(("T", "0 (RX1)"), ("LS", "LV1"), "rx", "S4")
    d.wire(("T", "3.3V"), ("LS", "LV"), "3v3", "S5")
    d.wire(("T", "GND"), ("LS", "GND"), "gnd", "S7")
    # VIN (5 V) goes around the bottom of the shifter to the HV pin on its right side
    vin = d.pins[("T", "VIN (5 V from USB)")]
    hv = d.pins[("LS", "HV")]
    d.wire(("T", "VIN (5 V from USB)"), ("LS", "HV"), "5v", "S6",
           via=[(600, vin[1]), (600, 330), (960, 330), (960, hv[1])], stub_b=60)
    # loopback jumper HV2 -> HV1
    hv2 = d.pins[("LS", "HV2")]
    d.wire(("LS", "HV2"), ("LS", "HV1"), "jump", "loopback jumper", via=[(990, hv2[1])], stub=90)
    d.note(40, 360, 500, ["Before connecting the Teensy, check the shifter's labels:",
                          "LV side = 3.3 V side, HV side = 5 V side. Swapping them",
                          "puts 5 V on a Teensy pin. The two GND pins are",
                          "connected inside the module, so one GND wire is enough."], "warn")
    d.probe(720, 385, "meter: LV pin = 3.3 V,  HV pin = 5.0 V")
    d.note(580, 430, 490, ["'# loopback OK' every second, errors = 0.",
                           "Pull the jumper: '# FAIL: nothing came back' (proves the test works)."], "pass")
    d.legend(40, 210)
    d.save()


# =====================================================================
# STAGE 4 -- power supply only, no loads
# =====================================================================
def stage4():
    d = Diagram("stage4_power", 1300, 560, "Stage 4 -- Power supply and fused branches, NO loads",
                "Wire IDs = docs/icd/power.md. Both branch ends stay OPEN (nothing plugged in). Measure with a multimeter.")
    d.comp("IN", 40, 110, 190, "AC inlet (IEC C14)", "switch + T5A fuse", right=["L", "N", "PE"], tint="#fff4e6")
    # DC terminals drawn grouped by branch (EDF pair on top, servo pair below);
    # the numbers are the real terminal numbers on the PSU. 6 and 9 are spares.
    d.comp("PSU", 330, 100, 230, "Mean Well LRS-600-12", "INPUT SWITCH = 115 V",
           left=["1  AC/L", "2  AC/N", "3  FG (earth)"],
           right=["4  -V", "7  +V", " ", "  ", "5  -V", "8  +V"], tint="#f8f9fa", nc=[" ", "  "])
    d.comp("F1", 660, 126, 130, "F1  15 A", "inline blade fuse", left=["IN"], right=["OUT"], tint="#fff5f5")
    d.comp("F2", 660, 230, 130, "F2  5 A", "inline blade fuse", left=["IN"], right=["OUT"], tint="#fff5f5")
    d.comp("XT", 900, 100, 200, "XT60 FEMALE", "EDF branch end (open)", left=["-", "+"], tint="#f8f9fa")
    d.comp("SB", 900, 217, 200, "Servo branch leads", "to BusLinker (open)", left=["GND", "+12 V"], tint="#f8f9fa")
    d.wire(("IN", "L"), ("PSU", "1  AC/L"), "acl", "P2", width=4)
    d.wire(("IN", "N"), ("PSU", "2  AC/N"), "acn", "P3", width=4)
    d.wire(("IN", "PE"), ("PSU", "3  FG (earth)"), "pe", "P4", width=4)
    # EDF branch (12 AWG)
    d.wire(("PSU", "7  +V"), ("F1", "IN"), "12v", "P5")
    d.wire(("F1", "OUT"), ("XT", "+"), "12v", "P6")
    d.wire(("PSU", "4  -V"), ("XT", "-"), "gnd", "P7  12 AWG", width=5,
           via=[(600, d.pins[("PSU", "4  -V")][1]), (600, 112), (870, 112), (870, d.pins[("XT", "-")][1])])
    # Servo branch (18 AWG)
    d.wire(("PSU", "8  +V"), ("F2", "IN"), "12v", "P8", width=4)
    d.wire(("F2", "OUT"), ("SB", "+12 V"), "12v", "P9", width=4)
    d.wire(("PSU", "5  -V"), ("SB", "GND"), "gnd", "P10  18 AWG",
           via=[(600, d.pins[("PSU", "5  -V")][1]), (600, 220), (870, 220), (870, d.pins[("SB", "GND")][1])])
    d.probe(1130, 175, "XT60: 12.0 V +/- 0.2")
    d.probe(1130, 192 + 100, "leads: 12.0 V")
    d.text(1150, 210, "red probe on +", 11, fill="#e67700")
    d.text(1150, 327, "check polarity!", 11, fill="#e67700")
    d.note(40, 360, 420, ["Mains (120 VAC). Have a TA / lab staff check the",
                          "AC wiring BEFORE the first plug-in. Cover the PSU's",
                          "AC terminals. Set the input switch to 115 V."], "warn")
    d.note(480, 360, 400, ["Fuses on the + side, close to the PSU.",
                           "Fork/ring terminals under every screw.",
                           "Thick (12 AWG) = EDF branch, thin = servo."], "info")
    d.legend(900, 380)
    d.save()


# =====================================================================
# STAGE 5 -- servo bus
# =====================================================================
def stage5():
    d = Diagram("stage5_servo", 1340, 640, "Stage 5 -- Servo bus: Teensy -> level shifter -> BusLinker -> HTD-30H",
                "Stage 3 wiring + the BusLinker in place of the jumper. Wire IDs = docs/icd/servo_bus.md. Flash: firmware/tests/servo_bus_test")
    d.comp("T", 300, 110, 200, "Teensy 4.0", "", left=["USB (micro)"],
           right=["1 (TX1)", "0 (RX1)", "3.3V", "GND", "VIN (5 V)"], tint="#e6fcf5")
    d.comp("LS", 600, 110, 170, "Level shifter", "LV = 3.3 V | HV = 5 V",
           left=["LV2", "LV1", "LV", "GND"], right=["HV2", "HV1", "HV", "GND "], tint="#f3f0ff")
    d.comp("BL", 900, 110, 210, "BusLinker V3.0", "jumpers: Servo + TTL",
           left=["RX", "TX", "GND", " ", "PWR +", "PWR -"], right=["servo port"], tint="#fff4e6", nc=[" "])
    d.comp("SV", 940, 400, 190, "HTD-30H servo", "ID 1, horn OFF the gimbal at first",
           left=["supplied PH2.0 cable"], tint="#fff9db")
    d.comp("SB", 600, 420, 190, "Servo branch", "from Stage 4 (F2 5 A)", right=["+12 V", "GND"], tint="#f8f9fa")
    pi_and_usb(d, 40, 110, ("T", "USB (micro)"))
    d.wire(("T", "1 (TX1)"), ("LS", "LV2"), "tx", "S1")
    d.wire(("T", "0 (RX1)"), ("LS", "LV1"), "rx", "S4")
    d.wire(("T", "3.3V"), ("LS", "LV"), "3v3", "S5")
    d.wire(("T", "GND"), ("LS", "GND"), "gnd", "S7")
    vin = d.pins[("T", "VIN (5 V)")]
    hv = d.pins[("LS", "HV")]
    d.wire(("T", "VIN (5 V)"), ("LS", "HV"), "5v", "S6", via=[(550, vin[1]), (550, 345), (810, 345), (810, hv[1])], stub_b=40)
    d.wire(("LS", "HV2"), ("BL", "RX"), "tx", "S2")
    d.wire(("LS", "HV1"), ("BL", "TX"), "rx", "S3")
    d.wire(("LS", "GND "), ("BL", "GND"), "gnd", "S7", via=[(850, d.pins[("LS", "GND ")][1]), (850, d.pins[("BL", "GND")][1])])
    d.wire(("SB", "+12 V"), ("BL", "PWR +"), "12v", "P9", width=4, via=[(860, d.pins[("SB", "+12 V")][1]), (860, d.pins[("BL", "PWR +")][1])])
    d.wire(("SB", "GND"), ("BL", "PWR -"), "gnd", "P10", via=[(880, d.pins[("SB", "GND")][1]), (880, d.pins[("BL", "PWR -")][1])])
    sp = d.pins[("BL", "servo port")]
    d.wire(("BL", "servo port"), ("SV", "supplied PH2.0 cable"), "bus", "S8",
           via=[(1170, sp[1]), (1170, 375), (910, 375), (910, d.pins[("SV", "supplied PH2.0 cable")][1])], stub_b=30, width=5)
    d.note(40, 300, 450, ["BEFORE wiring TX/RX: power the BusLinker alone",
                          "and measure its idle TX pin to GND.",
                          "  <= 3.3 V: the shifter is optional.",
                          "  ~5 V: the shifter is REQUIRED (as drawn).",
                          "Header labels aren't published [VERIFY on the board].",
                          "Record both in docs/icd/servo_bus.md."], "warn")
    d.note(40, 470, 520, ["'i' reads ID = 1; 'r' reads voltage ~12 V and a position.",
                          "'c' centers; '+' / '-' move 10 deg; 'w' sweeps +/-15 deg.",
                          "Then attach the horn at center and record the trim."], "pass")
    d.note(600, 540, 600, ["Cross-check: TX on one side goes to RX on the other (S1->S2, S3->S4).",
                           "If 'r' shows only your own bytes echoed: swap TX/RX or check the jumpers."], "info")
    d.legend(1200, 110)
    d.save()


# =====================================================================
# STAGE 6 -- closed-loop bench: IMU + servo, EDF unpowered
# =====================================================================
def stage6():
    d = Diagram("stage6_bench", 1250, 600, "Stage 6 -- Closed-loop bench test: IMU + servo, EDF UNPLUGGED",
                "Stage 2 + Stage 5 wiring together. Flash: firmware/flight_computer. Tilt the IMU by hand; the gimbal answers.")
    d.comp("IMU", 40, 110, 170, "GY-521", "", right=["VCC", "GND", "SCL", "SDA"], tint="#fff9db")
    d.comp("T", 360, 110, 220, "Teensy 4.0", "flight_computer",
           left=["3.3V", "GND", "19 (SCL)", "18 (SDA)", " ", "USB (micro)"],
           right=["1 (TX1)", "0 (RX1)", "3.3V ", "GND ", "VIN (5 V)"], tint="#e6fcf5", nc=[" "])
    d.comp("LS", 700, 110, 160, "Level shifter", "", left=["LV2", "LV1", "LV", "GND"], right=["HV2", "HV1", "HV", "GND "],
           tint="#f3f0ff")
    d.comp("BL", 980, 110, 200, "BusLinker", "Servo + TTL", left=["RX", "TX", "GND"], right=["servo port"], tint="#fff4e6")
    d.comp("SV", 1000, 330, 190, "HTD-30H", "on the gimbal", left=["cable"], tint="#fff9db")
    d.comp("PI", 40, 330, 170, "Raspberry Pi 5", "tvc_logger.py", right=["USB-A"], tint="#fff0f6")
    d.wire(("IMU", "VCC"), ("T", "3.3V"), "3v3", "I1")
    d.wire(("IMU", "GND"), ("T", "GND"), "gnd", "I2")
    d.wire(("IMU", "SCL"), ("T", "19 (SCL)"), "scl", "I3")
    d.wire(("IMU", "SDA"), ("T", "18 (SDA)"), "sda", "I4")
    d.wire(("PI", "USB-A"), ("T", "USB (micro)"), "usb", "USB", via=[(290, d.pins[("PI", "USB-A")][1]),
                                                                   (290, d.pins[("T", "USB (micro)")][1])])
    d.wire(("T", "1 (TX1)"), ("LS", "LV2"), "tx", "S1")
    d.wire(("T", "0 (RX1)"), ("LS", "LV1"), "rx", "S4")
    d.wire(("T", "3.3V "), ("LS", "LV"), "3v3", "S5")
    d.wire(("T", "GND "), ("LS", "GND"), "gnd", "S7")
    vin = d.pins[("T", "VIN (5 V)")]
    hv = d.pins[("LS", "HV")]
    d.wire(("T", "VIN (5 V)"), ("LS", "HV"), "5v", "S6", via=[(640, vin[1]), (640, 335), (900, 335), (900, hv[1])], stub_b=40)
    d.wire(("LS", "HV2"), ("BL", "RX"), "tx", "S2")
    d.wire(("LS", "HV1"), ("BL", "TX"), "rx", "S3")
    d.wire(("LS", "GND "), ("BL", "GND"), "gnd", "S7", via=[(930, d.pins[("LS", "GND ")][1]), (930, d.pins[("BL", "GND")][1])])
    sp = d.pins[("BL", "servo port")]
    d.wire(("BL", "servo port"), ("SV", "cable"), "bus", "S8",
           via=[(1220, sp[1]), (1220, 305), (970, 305), (970, d.pins[("SV", "cable")][1])], stub_b=30, width=5)
    d.note(240, 450, 460, ["Teensy 3.3V and GND each feed TWO things (IMU and",
                           "shifter). Same physical pin: use a short splice or a",
                           "breadboard rail. BusLinker power = Stage 4 servo branch.",
                           "EDF branch XT60: UNPLUGGED for this whole stage."], "info")
    d.note(720, 450, 500, ["'arm', then nose tips toward +x (theta > 0):",
                           "the fan tilts so its thrust pushes the nose BACK.",
                           "Wrong way -> flip SERVO_DIR in config.h. 'stop' re-centers."], "pass")
    d.save()


# =====================================================================
# STAGE 7 -- ESC + EDF (restrained)
# =====================================================================
def stage7():
    d = Diagram("stage7_esc", 1250, 640, "Stage 7 -- ESC + EDF, fan RESTRAINED, guard on",
                "Wire IDs = docs/icd/edf_esc.md and power.md. Flash: firmware/tests/esc_test. Servo branch may stay unplugged.")
    d.comp("F1", 40, 120, 170, "EDF branch", "from Stage 4 (F1 15 A)", right=["+12 V", "GND"], tint="#f8f9fa")
    d.comp("WM", 300, 120, 170, "Inline watt meter", "XT60 in / out", left=["in +", "in -"], right=["out +", "out -"],
           tint="#fff4e6")
    d.comp("ESC", 580, 110, 240, "Skywalker 50A V2 ESC", "brake OFF",
           left=["BAT+ (XT60)", "BAT- (XT60)", " ", "SIG (white/yellow)", "GND (black/brown)", "BEC +5 V (red)"],
           right=["MOT A", "MOT B", "MOT C"], tint="#e7f5ff", nc=[" ", "BEC +5 V (red)"])
    d.comp("EDF", 960, 110, 210, "JFtech 64 mm EDF", "3.5 mm bullets", left=["A", "B", "C"], tint="#f8f9fa")
    d.comp("T", 240, 370, 200, "Teensy 4.0", "esc_test", left=["USB (micro)"], right=["2 (PWM)", "GND"], tint="#e6fcf5")
    d.comp("PI", 20, 470, 170, "Raspberry Pi 5", "", right=["USB-A"], tint="#fff0f6")
    d.wire(("F1", "+12 V"), ("WM", "in +"), "12v", "P6")
    d.wire(("F1", "GND"), ("WM", "in -"), "gnd", "P7", width=5)
    d.wire(("WM", "out +"), ("ESC", "BAT+ (XT60)"), "12v", "E1")
    d.wire(("WM", "out -"), ("ESC", "BAT- (XT60)"), "gnd", "E1", width=5)
    for ph in ("A", "B", "C"):
        d.wire(("ESC", "MOT " + ph), ("EDF", ph), "mot", "E2" if ph == "A" else "")
    d.wire(("T", "2 (PWM)"), ("ESC", "SIG (white/yellow)"), "pwm", "E3",
           via=[(500, d.pins[("T", "2 (PWM)")][1]), (500, d.pins[("ESC", "SIG (white/yellow)")][1])])
    d.wire(("T", "GND"), ("ESC", "GND (black/brown)"), "gnd", "E4",
           via=[(515, d.pins[("T", "GND")][1]), (515, d.pins[("ESC", "GND (black/brown)")][1])])
    d.cross(("ESC", "BEC +5 V (red)"), "")
    d.text(600, 345, "E5: the ESC's red BEC wire is NOT connected -- pull it from the plug or insulate it", 11, "700",
           fill="#e03131")
    d.wire(("PI", "USB-A"), ("T", "USB (micro)"), "usb", "USB", via=[(215, d.pins[("PI", "USB-A")][1]),
                                                                   (215, d.pins[("T", "USB (micro)")][1])])
    d.note(560, 370, 640, ["EDF clamped to the bench, guard on, NOBODY in line with intake or exhaust.",
                           "Hand on the AC switch / E-stop. Eye protection. Start at 't 0.05'.",
                           "Air must blow OUT the exhaust: if not, swap any two motor wires (E2).",
                           "Raise THROTTLE_MAX only in small steps, watching the meter (< ~12 A)."], "warn")
    d.note(560, 500, 640, ["Calibration beeps as the manual describes; 's' always stops the fan;",
                           "UNPLUGGING the Teensy's USB also stops it (ESC signal-loss protection).",
                           "Each throttle step + current logged in docs/icd/edf_esc.md."], "pass")
    d.save()


# =====================================================================
# OVERVIEW -- which stage tests which link
# =====================================================================
def overview():
    d = Diagram("overview", 1200, 520, "Bring-up order -- each link is tested before it's trusted",
                "Blocks = parts. Numbers on the links = the stage that tests them. Full wire-level harness: docs/wiring/tvc_harness.png")
    d.comp("PI", 40, 110, 170, "Raspberry Pi 5", "ground computer", right=["USB"], tint="#fff0f6")
    d.comp("T", 330, 110, 200, "Teensy 4.0", "flight computer", left=["USB"], right=["I2C", "UART", "PWM"], tint="#e6fcf5")
    d.comp("IMU", 680, 100, 170, "GY-521 IMU", "", left=["I2C"], tint="#fff9db")
    d.comp("LS", 640, 220, 150, "Level shifter", "", left=["3.3 V"], right=["5 V"], tint="#f3f0ff")
    d.comp("BL", 870, 220, 150, "BusLinker", "", left=["TTL"], right=["bus", "power"], tint="#fff4e6")
    d.comp("SV", 1060, 110, 120, "HTD-30H", "gimbal servo", left=["bus"], tint="#fff9db")
    d.comp("ESC", 640, 380, 150, "ESC", "", left=["PWM"], right=["motor", "power"], tint="#e7f5ff")
    d.comp("EDF", 1060, 380, 120, "EDF", "", left=["motor"], tint="#f8f9fa")
    d.comp("PSU", 40, 400, 200, "12 V PSU + fuses", "", right=["F2 5 A", "F1 15 A"], tint="#f8f9fa")
    d.wire(("PI", "USB"), ("T", "USB"), "usb", "1")
    d.wire(("T", "I2C"), ("IMU", "I2C"), "sda", "2", via=[(600, d.pins[("T", "I2C")][1]), (600, d.pins[("IMU", "I2C")][1])])
    d.wire(("T", "UART"), ("LS", "3.3 V"), "tx", "3")
    d.wire(("LS", "5 V"), ("BL", "TTL"), "tx", "5")
    d.wire(("BL", "bus"), ("SV", "bus"), "bus", "5", via=[(1040, d.pins[("BL", "bus")][1]), (1040, d.pins[("SV", "bus")][1])])
    d.wire(("T", "PWM"), ("ESC", "PWM"), "pwm", "7", via=[(570, d.pins[("T", "PWM")][1]), (570, d.pins[("ESC", "PWM")][1])])
    d.wire(("ESC", "motor"), ("EDF", "motor"), "mot", "7")
    d.wire(("PSU", "F2 5 A"), ("BL", "power"), "12v", "4",
           via=[(270, d.pins[("PSU", "F2 5 A")][1]), (270, 355), (1045, 355), (1045, d.pins[("BL", "power")][1])])
    d.wire(("PSU", "F1 15 A"), ("ESC", "power"), "12v", "4",
           via=[(300, d.pins[("PSU", "F1 15 A")][1]), (300, 505), (830, 505), (830, d.pins[("ESC", "power")][1])])
    d.note(40, 230, 260, ["Stage 6 = IMU + servo together", "(EDF off). Stage 8 = everything,", "rocket on the pivot."], "info")
    d.save()


if __name__ == "__main__":
    for f in (overview, stage1, stage2, stage3, stage4, stage5, stage6, stage7):
        f()
