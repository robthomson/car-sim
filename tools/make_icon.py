"""Draw src/carsim/gfx/icon.png: the car seen from behind and above (the
game's own view, turned a little) as an Ethos mask (greyscale: black is
drawn, white is clear, as lcd.loadMask expects). The cabin glass and the
tail lights are cut out so it reads as a car, not a block.

    python tools/make_icon.py      (needs lupa and pillow)
"""
import math
import os

from lupa import lua54
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIZE, SS = 75, 4   # final size, supersampling

os.chdir(os.path.join(ROOT, "src", "carsim"))
L = lua54.LuaRuntime()
c = L.eval('assert(loadfile("lib/car.lua"))()')
c.build()
while not c.buildStep():
    pass
c.animate(0.25)    # front wheels turned in a little

# Camera: behind the car and to its right, looking down.
YAW, PITCH = math.radians(148), math.radians(18)
cy_, sy_ = math.cos(YAW), math.sin(YAW)
cp, sp = math.cos(PITCH), math.sin(PITCH)


def view(x, y, z):
    # Turn about Z by YAW, then tilt about the view's right axis by PITCH.
    x1, y1 = x * cy_ - y * sy_, x * sy_ + y * cy_
    # screen right = y1, depth = x1 (towards the viewer is +), up = z
    depth = -x1 * cp - z * sp
    up = z * cp - x1 * sp
    return y1, up, depth


n = int(c.vertexCount)
P = {i: view(c.MX[i], c.MY[i], c.MZ[i]) for i in range(1, n + 1)}
xs = [p[0] for p in P.values()]
ys = [p[1] for p in P.values()]
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
scale = (SIZE * SS * 0.94) / max(x1 - x0, y1 - y0)
mx, my = (x0 + x1) / 2, (y0 + y1) / 2


def pt(i):
    p = P[i]
    return (SIZE * SS / 2 + (p[0] - mx) * scale, SIZE * SS / 2 - (p[1] - my) * scale)


CUT = {int(c.MAT_GLASS), int(c.MAT_TAIL)}
OUTLINE = {int(c.MAT_TYRE), int(c.MAT_DARK)}
faces = []
for f in range(1, int(c.faceCount) + 1):
    ids = [c.FA[f], c.FB[f], c.FC[f]] + ([c.FD[f]] if c.FD[f] else [])
    depth = sum(P[i][2] for i in ids) / len(ids)
    faces.append((depth, f, ids))
faces.sort(reverse=True)   # far to near

img = Image.new("L", (SIZE * SS, SIZE * SS), 0)
d = ImageDraw.Draw(img)
for _, f, ids in faces:
    mat = int(c.FMAT[f])
    poly = [pt(i) for i in ids]
    if mat in CUT:
        # Glass and lights: a cut-out inside the shape, shrunk so an outline stays.
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        d.polygon(poly, fill=255)
        d.polygon([(cx + (p[0] - cx) * 0.78, cy + (p[1] - cy) * 0.72) for p in poly], fill=0)
    elif mat in OUTLINE:
        # Tyres and the wing: outlined, so they stand apart from the body.
        d.polygon(poly, fill=255, outline=0, width=SS)
    else:
        d.polygon(poly, fill=255)

img = img.resize((SIZE, SIZE), Image.LANCZOS)
img = Image.eval(img, lambda a: 255 - a)
out = os.path.join(ROOT, "src", "carsim", "gfx", "icon.png")
img.save(out)
print("wrote", out)
