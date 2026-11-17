"""Run CarSim headless: stubbed Ethos lcd/system, scripted or autopilot inputs.

Counts Lua instructions per wakeup/paint (Ethos aborts a paint that runs
about 20000) and saves chosen frames as PNG.

    python tools/headless.py [--scenario auto|straight|spin] [--seconds S] [--frames 40,120] [--sticks]

The "auto" scenario drives laps with a pure-pursuit autopilot. --sticks uses
the stick controls (aileron, throttle, elevator) instead of wheel + trigger.

Needs: pip install lupa pillow
"""
import argparse
import os
import sys

from lupa import lua54
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "carsim")
W, H = 800, 480
WAKE_DT = 0.05

# Scripts: (start time s, steer, trigger) in -1..1, held until the next row.
SCENARIOS = {
    "straight": [(0.0, 0, 0), (3.2, 0, 1), (9.0, 0, -1), (12.0, 0, 0)],
    "spin": [(0.0, 0, 0), (3.2, 0, 1), (6.0, -1, 1), (9.0, 0, 0)],
}

STUBS = r"""
local W, H = %d, %d
local T = 0
local analog = {[0] = 0, 0, 0, 0}
draw = {}
local colour = 0
local function rec(op) draw[#draw + 1] = op end
lcd = {
  RGB = function(r, g, b, a)
    a = a or 1
    return math.floor(r) * 16777216 + math.floor(g) * 65536 + math.floor(b) * 256 + math.floor(a * 255)
  end,
  color = function(c) colour = c end,
  getWindowSize = function() return W, H end,
  drawFilledRectangle = function(x, y, w, h) rec({"rect", colour, x, y, w, h}) end,
  drawFilledTriangle = function(a, b, c, d, e, f) rec({"tri", colour, a, b, c, d, e, f}) end,
  drawLine = function(a, b, c, d) rec({"line", colour, a, b, c, d}) end,
  drawFilledCircle = function(x, y, r) rec({"circle", colour, x, y, r}) end,
  drawText = function(x, y, s, flags) rec({"text", colour, x, y, s, flags or 0}) end,
  font = function() end,
  invalidate = function() end,
  loadMask = function() return nil end,
}
BLACK = lcd.RGB(0, 0, 0); WHITE = lcd.RGB(255, 255, 255)
FONT_S, FONT_STD, FONT_L, FONT_XL = 1, 2, 3, 4
CENTERED, RIGHT = 1, 2
CATEGORY_ANALOG = 1
EVT_KEY, EVT_CLOSE, EVT_TOUCH = 0, 1, 2
TOUCH_START, TOUCH_END = 1, 2
KEY_ENTER_BREAK, KEY_ENTER_LONG, KEY_ENTER_FIRST = 10, 11, 12
KEY_EXIT_BREAK, KEY_EXIT_FIRST = 20, 21
KEY_ROTARY_LEFT, KEY_ROTARY_RIGHT = 30, 31
system = {
  getSource = function(t)
    return {value = function() return math.floor(analog[t.member] * 1024) end}
  end,
  getVersion = function() return {board = BOARD} end,
  playTone = function() end,
}
os.clock = function() return T end
function setTime(t) T = t end
function setAnalog(i, v) analog[i] = v end

-- Autopilot: aim at the centre line some way ahead, ease off for corners.
apTrack = assert(loadfile("lib/track.lua"))()
apTrack.build()
while not apTrack.buildStep() do end
apSeg = nil
function autopilot(h)
  local tr = apTrack
  apSeg = apSeg and tr.nearest(h.px, h.py, apSeg) or tr.nearestAll(h.px, h.py)
  local n = tr.count
  local look = 2 + math.floor(h.speed / 6)
  local j = ((apSeg - 1 + look) %% n) + 1
  local dx, dy = tr.CX[j] - h.px, tr.CY[j] - h.py
  local want = math.atan(dy, dx)
  local err = (want - h.yaw + math.pi) %% (2 * math.pi) - math.pi
  local steer = math.max(-1, math.min(1, -err * 2.5))
  -- Corner ahead: the heading change over the next few samples.
  local k = ((apSeg - 1 + look + 6) %% n) + 1
  local a1 = math.atan(tr.TY[j], tr.TX[j])
  local a2 = math.atan(tr.TY[k], tr.TX[k])
  local bend = math.abs((a2 - a1 + math.pi) %% (2 * math.pi) - math.pi)
  local target = math.max(14, 52 - bend * 45)
  local trig = (h.speed < target) and 1 or -0.6
  return steer, trig
end

function countCall(fn, ...)
  local n = 0
  debug.sethook(function() n = n + 1 end, "", 1)
  fn(...)
  debug.sethook()
  return n
end
""" % (W, H)


def unpack_rgba(c):
    c = int(c)
    return ((c >> 24) & 255, (c >> 16) & 255, (c >> 8) & 255, c & 255)


def render_png(ops, path):
    img = Image.new("RGBA", (W, H), (0, 0, 0, 255))
    font = ImageFont.load_default()
    for op in ops:
        kind = op[0]
        col = unpack_rgba(op[1])
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0)) if col[3] < 255 else None
        d = ImageDraw.Draw(layer if layer else img)
        if kind == "rect":
            x, y, w, h = op[2], op[3], op[4], op[5]
            d.rectangle([x, y, x + w - 1, y + h - 1], fill=col)
        elif kind == "tri":
            d.polygon([(op[2], op[3]), (op[4], op[5]), (op[6], op[7])], fill=col)
        elif kind == "line":
            d.line([(op[2], op[3]), (op[4], op[5])], fill=col)
        elif kind == "circle":
            x, y, r = op[2], op[3], op[4]
            d.ellipse([x - r, y - r, x + r, y + r], fill=col)
        elif kind == "text":
            x, y, s, flags = op[2], op[3], str(op[4]), op[5]
            tw = d.textlength(s, font=font)
            if flags == 1:
                x -= tw / 2
            elif flags == 2:
                x -= tw
            d.text((x, y), s, fill=col, font=font)
        if layer:
            img = Image.alpha_composite(img, layer)
    img.convert("RGB").save(path)


def save_map(tr, path_pts, out):
    """Top-down: asphalt grey, kerbs red, trees green, the path blue (red where sliding)."""
    n = int(tr.count)
    pts = [(tr.KLX[i], tr.KLY[i]) for i in range(1, n + 1)] + [(tr.KRX[i], tr.KRY[i]) for i in range(1, n + 1)]
    x0, x1 = min(p[0] for p in pts) - 70, max(p[0] for p in pts) + 70
    y0, y1 = min(p[1] for p in pts) - 70, max(p[1] for p in pts) + 70
    k = 900 / max(x1 - x0, y1 - y0)
    img = Image.new("RGB", (int((x1 - x0) * k), int((y1 - y0) * k)), (86, 136, 64))
    d = ImageDraw.Draw(img)

    def sp(x, y):
        return ((x - x0) * k, (y1 - y) * k)

    for i in range(1, n + 1):
        j = i % n + 1
        d.polygon([sp(tr.KLX[i], tr.KLY[i]), sp(tr.KLX[j], tr.KLY[j]), sp(tr.KRX[j], tr.KRY[j]), sp(tr.KRX[i], tr.KRY[i])], fill=(200, 60, 50))
        d.polygon([sp(tr.LX[i], tr.LY[i]), sp(tr.LX[j], tr.LY[j]), sp(tr.RX[j], tr.RY[j]), sp(tr.RX[i], tr.RY[i])], fill=(90, 92, 98))
    d.line([sp(tr.LX[1], tr.LY[1]), sp(tr.RX[1], tr.RY[1])], fill=(255, 255, 255), width=3)
    for t in tr.TREES.values():
        x, y = sp(t[1], t[2])
        d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(30, 70, 40))
    for a, b in zip(path_pts, path_pts[1:]):
        d.line([sp(a[0], a[1]), sp(b[0], b[1])], fill=(230, 40, 40) if b[2] else (60, 140, 255), width=2)
    img.save(out)
    print("  saved", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "tools", "out"))
    ap.add_argument("--frames", default="")
    ap.add_argument("--scenario", default="auto")
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--sticks", action="store_true", help="stick controls instead of wheel + trigger")
    ap.add_argument("--detail", type=int, default=1)
    ap.add_argument("--map", action="store_true", help="also save a top-down map of the circuit and the path driven")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    want = {int(x) for x in args.frames.split(",") if x}

    os.chdir(SRC)  # Ethos runs a tool with its own folder as the current directory
    L = lua54.LuaRuntime()
    L.globals().BOARD = "X20S" if args.sticks else "VX10"
    L.execute(STUBS)
    sim = L.eval('assert(loadfile("sim.lua"))()')
    g = L.globals()
    state = sim.create(None)
    state.cfg.detail = args.detail
    state.cfg.best = 0

    path_pts = []
    worst_w = worst_p = 0
    total_w = total_p = 0
    frames = int(args.seconds / WAKE_DT)
    for f in range(frames):
        t = f * WAKE_DT
        h = state.h
        if args.scenario == "auto" and state.phase != "boot":
            steer, trig = g.autopilot(h)
        elif args.scenario == "auto":
            steer, trig = 0, 0
        else:
            row = [r for r in SCENARIOS[args.scenario] if r[0] <= t][-1]
            steer, trig = row[1], row[2]
        if args.sticks:
            g.setAnalog(3, steer)
            g.setAnalog(2, max(0.0, trig) * 2 - 1)
            g.setAnalog(1, min(0.0, trig))
        else:
            g.setAnalog(0, steer)
            g.setAnalog(1, trig)
        g.setTime(t)
        nw = g.countCall(sim.wakeup, state)
        g.draw = L.table()
        np_ = g.countCall(sim.paint, state)
        worst_w, worst_p = max(worst_w, nw), max(worst_p, np_)
        total_w += nw
        total_p += np_
        h = state.h
        if h.px is not None:
            path_pts.append((h.px, h.py, h.sliding))
        if (f % 40 == 0 or f in want) and h.px is not None:
            print(f"f{f:4d} t={t:5.2f} {state.phase:5s} wake={nw:6d} paint={np_:5d} "
                  f"pos=({h.px:7.1f},{h.py:6.1f}) v={h.speed:5.1f} seg={state.seg:3d} "
                  f"lap={state.lap} lapT={state.lapTime:5.1f} surf={h.surface} slide={h.sliding}"
                  f"{' MSG ' + state.message if state.message else ''}")
        if f in want:
            ops = [tuple(op.values()) for op in g.draw.values()]
            path = os.path.join(args.out, f"{args.scenario}_{f:04d}.png")
            render_png(ops, path)
            print("  saved", path)
    if args.map:
        save_map(g.apTrack, path_pts, os.path.join(args.out, f"{args.scenario}_map.png"))
    print(f"best lap {state.cfg.best} ms; worst wakeup {worst_w}, worst paint {worst_p}; "
          f"mean wakeup {total_w // frames}, mean paint {total_p // frames}")


if __name__ == "__main__":
    sys.exit(main())
