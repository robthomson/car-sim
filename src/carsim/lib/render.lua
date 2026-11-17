-- The view: a chase camera behind and above the car, following its heading
-- with a little lag. The camera never rolls, so the horizon stays level.
--
-- render.frame() runs in wakeup and turns the scene into flat display lists
-- (sky and grass bands, hills, the circuit, trees and the start gantry, the
-- car's sorted faces); render.paint() only replays them. Ethos aborts a
-- paint that runs about 20000 Lua instructions, while wakeup is not held to
-- that budget, so all the maths lives in wakeup. No tables are allocated
-- per frame. This is plane-sim's renderer with a moving eye.
--
-- The circuit is flat, and no two of its pieces (asphalt, kerbs, the start
-- line, the car's shadow) overlap, so the ground needs no sorting: any
-- segment within reach and in front of the camera is drawn. Trees and the
-- gantry are sorted far to near and split around the car.
--
-- World axes: X east, Y north, Z up (metres). Camera yaw 0 looks north,
-- yaw > 0 turns towards east; pitch > 0 looks up.

local sin, cos, tan, atan = math.sin, math.cos, math.tan, math.atan
local sqrt, floor, min, max, abs = math.sqrt, math.floor, math.min, math.max, math.abs
local rad, pi = math.rad, math.pi
local t_sort = table.sort

local render = {}

local model                       -- the mesh module
local track                       -- the circuit

local HFOV_SLOW, HFOV_FAST = rad(62), rad(74)
local NEAR = 0.3
local CAM_BACK, CAM_H = 7.5, 2.6    -- metres behind and above the car
local LOOK_AHEAD, LOOK_Z = 6, 0.8   -- the point the camera looks at
local DRAW_DIST = 230               -- metres
local KERB_DIST = 130               -- kerbs beyond this are left out
local CAM_LAG = 5                   -- 1/s: how fast the camera swings round

---------------------------------------------------------------------------
-- Colours, built once.
---------------------------------------------------------------------------

local C = {}
local SHADES = {}
local SHADE_LEVELS = 8
local AMBIENT, DIFFUSE = 0.45, 0.65

-- In the mesh's material order.
local MODEL_COLORS = {
  {255, 95, 31},      -- body: Gripora orange
  {58, 60, 68},       -- chassis, wing
  {240, 240, 228},    -- headlights
  {60, 75, 95},       -- cabin glass
  {160, 160, 165},    -- hubs
  {40, 40, 46},       -- tyres
  {225, 40, 30},      -- tail lights
}

local SKY_RGB = {{200, 222, 240}, {160, 198, 232}, {120, 168, 224}, {86, 138, 210}}
local GRASS_RGB = {{110, 146, 92}, {92, 140, 70}, {80, 132, 58}}

local BAND_R, BAND_C = {}, {}

local function buildColors()
  for m = 1, #MODEL_COLORS do
    local base = MODEL_COLORS[m]
    local levels = {}
    for i = 1, SHADE_LEVELS do
      local s = AMBIENT + DIFFUSE * (i - 1) / (SHADE_LEVELS - 1)
      levels[i] = lcd.RGB(min(255, floor(base[1] * s)), min(255, floor(base[2] * s)), min(255, floor(base[3] * s)))
    end
    SHADES[m] = levels
  end
  C.hills = lcd.RGB(122, 150, 150)
  C.asphalt = {lcd.RGB(84, 86, 92), lcd.RGB(92, 94, 100)}
  C.kerb = {lcd.RGB(220, 40, 36), lcd.RGB(240, 240, 236)}
  C.line = lcd.RGB(240, 240, 236)
  C.lineDark = lcd.RGB(30, 30, 34)
  C.shadow = lcd.RGB(0, 0, 0, 0.4)
  C.trunk = lcd.RGB(92, 70, 50)
  C.pine = {lcd.RGB(34, 82, 44), lcd.RGB(44, 96, 50), lcd.RGB(28, 70, 40)}
  C.post = lcd.RGB(70, 72, 80)
  C.banner = lcd.RGB(255, 95, 31)
  local sky = {}
  for i = 1, #SKY_RGB do sky[i] = lcd.RGB(SKY_RGB[i][1], SKY_RGB[i][2], SKY_RGB[i][3]) end
  local grass = {}
  for i = 1, #GRASS_RGB do grass[i] = lcd.RGB(GRASS_RGB[i][1], GRASS_RGB[i][2], GRASS_RGB[i][3]) end
  local E = {92, 30, 10, 2.5, 0, -2, -7, -92}
  for i = 1, #E do BAND_R[i] = rad(E[i]) end
  local cols = {sky[4], sky[3], sky[2], sky[1], grass[1], grass[2], grass[3]}
  for i = 1, #cols do BAND_C[i] = cols[i] end
end

-- Far hills: elevation in degrees at evenly spaced azimuths (index 1 = north).
local HILLS = {2.0, 3.2, 2.4, 1.6, 2.6, 4.0, 3.0, 1.8, 1.8, 2.8, 3.4, 2.0, 1.6, 2.2, 1.8, 2.0}
local hillRing

local function ringTables(elev)
  local n = #elev
  local r = {n = n, step = 2 * pi / n, SA = {}, CA = {}, SE = {}, CE = {}}
  for i = 1, n do
    local a = (i - 1) * r.step
    r.SA[i], r.CA[i] = sin(a), cos(a)
    local e = rad(elev[i])
    r.SE[i], r.CE[i] = sin(e), cos(e)
  end
  return r
end

local detail = 1                     -- 1 low (default), 2 high

---------------------------------------------------------------------------
-- Camera and display lists.
---------------------------------------------------------------------------

local eyeX, eyeY, eyeZ = 0, 0, CAM_H
local camYaw, camPitch, camDist = 0, 0, 1
local hfov = HFOV_SLOW
local cRx, cRy                       -- right (z = 0)
local cUx, cUy, cUz                  -- up
local cFx, cFy, cFz                  -- forward
local scrW, scrH, cx, cy, focal = 800, 480, 400, 240, 571
local kx, ky = 1, 1                  -- frustum slopes (with margin)

local BY, BH, BCOL = {}, {}, {}
local bandCount = 0

-- Triangles: ground layer, then scenery behind the car, then in front of it.
local TX1, TY1, TX2, TY2, TX3, TY3, TCOL = {}, {}, {}, {}, {}, {}, {}
local triCount, triGround, triFar = 0, 0, 0

-- The car: screen vertices, sorted face keys, face colours.
local SX, SY = {}, {}
local KEY, FCOL = {}, {}
local keyCount = 0
local modelMode = 0                   -- 0 hidden, 2 mesh
local KEY_FACES = 512

-- Scenery sort keys (trees, and the gantry as index 0).
local SKEY = {}
local skeyCount = 0
local SCENE_IDX = 128

local function tri(x1, y1, x2, y2, x3, y3, col)
  local n = triCount + 1
  triCount = n
  TX1[n], TY1[n], TX2[n], TY2[n], TX3[n], TY3[n], TCOL[n] = x1, y1, x2, y2, x3, y3, col
end

local function band(y0, y1, col)
  if y1 <= 0 or y0 >= scrH or y1 <= y0 then return end
  if y0 < 0 then y0 = 0 end
  if y1 > scrH then y1 = scrH end
  local n = bandCount + 1
  bandCount = n
  BY[n], BH[n], BCOL[n] = floor(y0), floor(y1) - floor(y0) + 1, col
end

local function toCam(x, y, z)
  local dx, dy, dz = x - eyeX, y - eyeY, z - eyeZ
  return dx * cRx + dy * cRy, dx * cUx + dy * cUy + dz * cUz, dx * cFx + dy * cFy + dz * cFz
end

local PX, PY, PZ, QX, QY, QZ = {}, {}, {}, {}, {}, {}

local function clipPlane(n, a, b, c, d, sx, sy, sz, dx, dy, dz)
  local m = 0
  if n == 0 then return 0 end
  local px, py, pz = sx[n], sy[n], sz[n]
  local pd = a * px + b * py + c * pz + d
  for i = 1, n do
    local qx, qy, qz = sx[i], sy[i], sz[i]
    local qd = a * qx + b * qy + c * qz + d
    if (pd >= 0) ~= (qd >= 0) then
      local t = pd / (pd - qd)
      m = m + 1
      dx[m], dy[m], dz[m] = px + (qx - px) * t, py + (qy - py) * t, pz + (qz - pz) * t
    end
    if qd >= 0 then
      m = m + 1
      dx[m], dy[m], dz[m] = qx, qy, qz
    end
    px, py, pz, pd = qx, qy, qz, qd
  end
  return m
end

-- Clip a convex polygon in PX/PY/PZ (camera coordinates) and fan it into
-- triangles.
local function polygon(n, col)
  local any, all = 0, 31
  for i = 1, n do
    local x, y, z = PX[i], PY[i], PZ[i]
    local code = 0
    if z < NEAR then code = 1 end
    if x > kx * z then code = code | 2 elseif -x > kx * z then code = code | 4 end
    if y > ky * z then code = code | 8 elseif -y > ky * z then code = code | 16 end
    any, all = any | code, all & code
  end
  if all ~= 0 then return end
  local f = focal
  if any == 0 then
    local x0, y0 = cx + PX[1] * f / PZ[1], cy - PY[1] * f / PZ[1]
    local xp, yp = cx + PX[2] * f / PZ[2], cy - PY[2] * f / PZ[2]
    for i = 3, n do
      local xi, yi = cx + PX[i] * f / PZ[i], cy - PY[i] * f / PZ[i]
      tri(x0, y0, xp, yp, xi, yi, col)
      xp, yp = xi, yi
    end
    return
  end
  n = clipPlane(n, 0, 0, 1, -NEAR, PX, PY, PZ, QX, QY, QZ)
  n = clipPlane(n, -1, 0, kx, 0, QX, QY, QZ, PX, PY, PZ)
  n = clipPlane(n, 1, 0, kx, 0, PX, PY, PZ, QX, QY, QZ)
  n = clipPlane(n, 0, -1, ky, 0, QX, QY, QZ, PX, PY, PZ)
  n = clipPlane(n, 0, 1, ky, 0, PX, PY, PZ, QX, QY, QZ)
  if n < 3 then return end
  local x0, y0 = cx + QX[1] * f / QZ[1], cy - QY[1] * f / QZ[1]
  local xp, yp = cx + QX[2] * f / QZ[2], cy - QY[2] * f / QZ[2]
  for i = 3, n do
    local xi, yi = cx + QX[i] * f / QZ[i], cy - QY[i] * f / QZ[i]
    tri(x0, y0, xp, yp, xi, yi, col)
    xp, yp = xi, yi
  end
end

-- A flat quad on the ground (corners in order).
local function groundQuad(x1, y1, x2, y2, x3, y3, x4, y4, col)
  PX[1], PY[1], PZ[1] = toCam(x1, y1, 0)
  PX[2], PY[2], PZ[2] = toCam(x2, y2, 0)
  PX[3], PY[3], PZ[3] = toCam(x3, y3, 0)
  PX[4], PY[4], PZ[4] = toCam(x4, y4, 0)
  polygon(4, col)
end

-- An upright quad from ground points a to b, between heights z0 and z1.
local function wallQuad(ax, ay, bx, by, z0, z1, col)
  PX[1], PY[1], PZ[1] = toCam(ax, ay, z0)
  PX[2], PY[2], PZ[2] = toCam(bx, by, z0)
  PX[3], PY[3], PZ[3] = toCam(bx, by, z1)
  PX[4], PY[4], PZ[4] = toCam(ax, ay, z1)
  polygon(4, col)
end

local function elevY(e)
  e = e - camPitch
  if e >= pi * 0.49 then return -1 end
  if e <= -pi * 0.49 then return scrH + 1 end
  return cy - focal * tan(e)
end

-- A ring of silhouette points between the horizon and its elevations.
local function silhouette(r, col)
  local cp, sp = cos(camPitch), sin(camPitch)
  local sy, cyw = sin(camYaw), cos(camYaw)
  local half = atan(kx) + r.step
  local i0 = floor((camYaw - half) / r.step)
  local i1 = floor((camYaw + half) / r.step) + 1
  local SA, CA, SE, CE, n = r.SA, r.CA, r.SE, r.CE, r.n
  local pfx, pfy, ptx, pty, havePrev
  local f = focal
  for j = i0, i1 do
    local i = (j % n) + 1
    local sd = SA[i] * cyw - CA[i] * sy
    local cd = CA[i] * cyw + SA[i] * sy
    local zf = cd * cp
    local ce, se = CE[i], SE[i]
    local zt = ce * cd * cp + se * sp
    if zf > 0.2 and zt > 0.2 then
      local fx, fy = cx + sd * f / zf, cy + cd * sp * f / zf
      local tx, ty = cx + ce * sd * f / zt, cy - (se * cp - ce * cd * sp) * f / zt
      if havePrev then
        tri(pfx, pfy, ptx, pty, tx, ty, col)
        tri(pfx, pfy, tx, ty, fx, fy, col)
      end
      pfx, pfy, ptx, pty, havePrev = fx, fy, tx, ty, true
    else
      havePrev = false
    end
  end
end

---------------------------------------------------------------------------
-- The circuit.
---------------------------------------------------------------------------

-- Each track sample's four edge points (kerb left, asphalt left, asphalt
-- right, kerb right) in camera coordinates, with their outcodes and screen
-- positions, at index (sample - 1) * 4 + k. Computed once per frame for the
-- samples in use (STAMP), so neighbouring quads share them.
local GX, GY, GZ, GC, GSX, GSY = {}, {}, {}, {}, {}, {}
local STAMP = {}
local frameNo = 0
local NEAR_SEG = 70                  -- metres: beyond this, segments are drawn in pairs

local function projectSample(i)
  STAMP[i] = frameNo
  local base = (i - 1) * 4
  local xs1, xs2, xs3, xs4 = track.KLX[i], track.LX[i], track.RX[i], track.KRX[i]
  local ys1, ys2, ys3, ys4 = track.KLY[i], track.LY[i], track.RY[i], track.KRY[i]
  local rx, ry, ux, uy, fx, fy = cRx, cRy, cUx, cUy, cFx, cFy
  local uz0, fz0 = -eyeZ * cUz, -eyeZ * cFz
  local f, kxx, kyy = focal, kx, ky
  for k = 1, 4 do
    local wx, wy
    if k == 1 then wx, wy = xs1, ys1 elseif k == 2 then wx, wy = xs2, ys2
    elseif k == 3 then wx, wy = xs3, ys3 else wx, wy = xs4, ys4 end
    local dx, dy = wx - eyeX, wy - eyeY
    local x, y, z = dx * rx + dy * ry, dx * ux + dy * uy + uz0, dx * fx + dy * fy + fz0
    local code = 0
    if z < NEAR then code = 1 end
    if x > kxx * z then code = code | 2 elseif -x > kxx * z then code = code | 4 end
    if y > kyy * z then code = code | 8 elseif -y > kyy * z then code = code | 16 end
    local m = base + k
    GX[m], GY[m], GZ[m], GC[m] = x, y, z, code
    if z >= NEAR then GSX[m], GSY[m] = cx + x * f / z, cy - y * f / z end
  end
end

-- The ground quad between sample points a, b (at sample i) and c, d (at j).
local function sampleQuad(a, b, c, d, col)
  local ca, cb, cc, cd = GC[a], GC[b], GC[c], GC[d]
  if ca & cb & cc & cd ~= 0 then return end
  if ca | cb | cc | cd == 0 then
    local ax, ay, cx3, cy3 = GSX[a], GSY[a], GSX[c], GSY[c]
    tri(ax, ay, GSX[b], GSY[b], cx3, cy3, col)
    tri(ax, ay, cx3, cy3, GSX[d], GSY[d], col)
    return
  end
  PX[1], PY[1], PZ[1] = GX[a], GY[a], GZ[a]
  PX[2], PY[2], PZ[2] = GX[b], GY[b], GZ[b]
  PX[3], PY[3], PZ[3] = GX[c], GY[c], GZ[c]
  PX[4], PY[4], PZ[4] = GX[d], GY[d], GZ[d]
  polygon(4, col)
end

local function circuit()
  frameNo = frameNo + 1
  local n = track.count
  local CXs, CYs = track.CX, track.CY
  local LX, LY, RX, RY = track.LX, track.LY, track.RX, track.RY
  local asphalt, kerb = C.asphalt, C.kerb
  local fx, fy = cFx, cFy
  local fl = sqrt(fx * fx + fy * fy)
  fx, fy = fx / fl, fy / fl
  local draw2, kerb2, near2 = DRAW_DIST * DRAW_DIST, KERB_DIST * KERB_DIST, NEAR_SEG * NEAR_SEG
  local ex, ey = eyeX, eyeY
  local i = 1
  while i <= n do
    local dx, dy = CXs[i] - ex, CYs[i] - ey
    local d2 = dx * dx + dy * dy
    local step = (d2 > near2 and i < n) and 2 or 1
    -- Within reach, and not wholly behind the camera.
    if d2 < draw2 and dx * fx + dy * fy > -12 then
      local j = i + step
      if j > n then j = j - n end
      if STAMP[i] ~= frameNo then projectSample(i) end
      if STAMP[j] ~= frameNo then projectSample(j) end
      local bi, bj = (i - 1) * 4, (j - 1) * 4
      local stripe = ((step == 1) and (i % 2) or ((i // 2) % 2)) + 1
      sampleQuad(bi + 2, bj + 2, bj + 3, bi + 3, asphalt[stripe])
      if d2 < kerb2 then
        local kc = kerb[stripe]
        sampleQuad(bi + 1, bj + 1, bj + 2, bi + 2, kc)
        sampleQuad(bi + 3, bj + 3, bj + 4, bi + 4, kc)
      end
    end
    i = i + step
  end
  -- Start line at sample 1: a white band with a dark stripe behind it.
  local tx, ty = track.TX[1], track.TY[1]
  local dx, dy = CXs[1] - ex, CYs[1] - ey
  if dx * dx + dy * dy < draw2 then
    groundQuad(LX[1], LY[1], RX[1], RY[1], RX[1] + tx * 0.8, RY[1] + ty * 0.8, LX[1] + tx * 0.8, LY[1] + ty * 0.8, C.line)
    groundQuad(LX[1] - tx * 0.8, LY[1] - ty * 0.8, RX[1] - tx * 0.8, RY[1] - ty * 0.8, RX[1], RY[1], LX[1], LY[1], C.lineDark)
  end
end

local function tree(t)
  local x, y, h, w = t[1], t[2], t[3], t[4]
  local bx, by, bz = toCam(x, y, 0)
  if bz < 1 then return end
  if abs(bx) > kx * bz + w then return end
  local tx, ty, tz = toCam(x, y, h)
  local f = focal
  local sbx, sby = cx + bx * f / bz, cy - by * f / bz
  local stx, sty = cx + tx * f / tz, cy - ty * f / tz
  local hw = w * 0.5 * f / bz
  if detail == 1 then
    tri(sbx - hw, sby, sbx + hw, sby, stx, sty, C.pine[t[5]])
    return
  end
  local dx, dy = stx - sbx, sty - sby
  local tw = max(1, hw * 0.14)
  local t1x, t1y = sbx + dx * 0.3, sby + dy * 0.3
  tri(sbx - tw, sby, sbx + tw, sby, t1x + tw, t1y, C.trunk)
  tri(sbx - tw, sby, t1x + tw, t1y, t1x - tw, t1y, C.trunk)
  local col = C.pine[t[5]]
  local ax, ay = sbx + dx * 0.18, sby + dy * 0.18
  local mx, my = sbx + dx * 0.78, sby + dy * 0.78
  tri(ax - hw, ay, ax + hw, ay, mx, my, col)
  local bx2, by2 = sbx + dx * 0.48, sby + dy * 0.48
  local hw2 = hw * 0.72
  tri(bx2 - hw2, by2, bx2 + hw2, by2, stx, sty, col)
end

-- The start gantry over the line: two posts and a banner.
local function gantry()
  local LX, LY, RX, RY = track.KLX[1], track.KLY[1], track.KRX[1], track.KRY[1]
  local nx, ny = track.NX[1], track.NY[1]
  wallQuad(LX, LY, LX + nx * 0.6, LY + ny * 0.6, 0, 6.5, C.post)
  wallQuad(RX - nx * 0.6, RY - ny * 0.6, RX, RY, 0, 6.5, C.post)
  wallQuad(LX, LY, RX, RY, 5.0, 6.4, C.banner)
end

---------------------------------------------------------------------------
-- The car.
---------------------------------------------------------------------------

local SUN_X, SUN_Y, SUN_Z
do
  local x, y, z = 0.35, -0.55, 0.78
  local l = sqrt(x * x + y * y + z * z)
  SUN_X, SUN_Y, SUN_Z = x / l, y / l, z / l
end

local fa31, fa32, fa33 = 0, 0, 1
local emx_, emy_, emz_, lmx_, lmy_, lmz_ = 0, 0, 0, 0, 0, 1
local facesPending = false
local faceLimit = 0
local allInFront = true
local LOD_PX = 30
local carDepth = 0

local function modelFrame(h)
  modelMode = 0
  local dx, dy, dz = h.px - eyeX, h.py - eyeY, h.pz - eyeZ
  local t1 = dx * cRx + dy * cRy
  local t2 = dx * cUx + dy * cUy + dz * cUz
  local t3 = dx * cFx + dy * cFy + dz * cFz
  carDepth = t3
  if t3 < 1.0 then return end
  local f = focal
  local radiusPx = model.SIZE * f / t3
  modelMode = 2

  local r11, r12, r13 = h.r11, h.r12, h.r13
  local r21, r22, r23 = h.r21, h.r22, h.r23
  local r31, r32, r33 = h.r31, h.r32, h.r33
  local a11, a12, a13 = cRx * r11 + cRy * r21, cRx * r12 + cRy * r22, cRx * r13 + cRy * r23
  local a21 = cUx * r11 + cUy * r21 + cUz * r31
  local a22 = cUx * r12 + cUy * r22 + cUz * r32
  local a23 = cUx * r13 + cUy * r23 + cUz * r33
  local a31 = cFx * r11 + cFy * r21 + cFz * r31
  local a32 = cFx * r12 + cFy * r22 + cFz * r32
  local a33 = cFx * r13 + cFy * r23 + cFz * r33
  local emx = -(r11 * dx + r21 * dy + r31 * dz)
  local emy = -(r12 * dx + r22 * dy + r32 * dz)
  local emz = -(r13 * dx + r23 * dy + r33 * dz)
  local lmx = r11 * SUN_X + r21 * SUN_Y + r31 * SUN_Z
  local lmy = r12 * SUN_X + r22 * SUN_Y + r32 * SUN_Z
  local lmz = r13 * SUN_X + r23 * SUN_Y + r33 * SUN_Z

  local nv = model.vertexCount
  faceLimit = model.faceCount
  if radiusPx < LOD_PX then nv, faceLimit = model.coreVertices, model.coreFaces end

  local MX, MY, MZ = model.MX, model.MY, model.MZ
  allInFront = true
  if t3 > NEAR + 2.5 then
    for i = 1, nv do
      local x, y, z = MX[i], MY[i], MZ[i]
      local k = f / (x * a31 + y * a32 + z * a33 + t3)
      SX[i] = cx + (x * a11 + y * a12 + z * a13 + t1) * k
      SY[i] = cy - (x * a21 + y * a22 + z * a23 + t2) * k
    end
  else
    allInFront = false
    for i = 1, nv do
      local x, y, z = MX[i], MY[i], MZ[i]
      local zc = x * a31 + y * a32 + z * a33 + t3
      if zc < NEAR then
        SX[i] = false
      else
        local k = f / zc
        SX[i] = cx + (x * a11 + y * a12 + z * a13 + t1) * k
        SY[i] = cy - (x * a21 + y * a22 + z * a23 + t2) * k
      end
    end
  end

  fa31, fa32, fa33 = a31, a32, a33
  emx_, emy_, emz_, lmx_, lmy_, lmz_ = emx, emy, emz, lmx, lmy, lmz
  facesPending = true
end

local function modelFaces()
  facesPending = false
  local a31, a32, a33 = fa31, fa32, fa33
  local emx, emy, emz, lmx, lmy, lmz = emx_, emy_, emz_, lmx_, lmy_, lmz_
  local FA, FB, FC, FD, FMAT, FTWO = model.FA, model.FB, model.FC, model.FD, model.FMAT, model.FTWO
  local FNX, FNY, FNZ, FPD = model.FNX, model.FNY, model.FNZ, model.FPD
  local FCX, FCY, FCZ = model.FCX, model.FCY, model.FCZ
  local levels = SHADE_LEVELS - 1
  local inFront = allInFront
  local n = 0
  for i = 1, faceLimit do
    local nx, ny, nz = FNX[i], FNY[i], FNZ[i]
    local facing = nx * emx + ny * emy + nz * emz - FPD[i]
    if facing > 0 or FTWO[i] then
      local d = FD[i]
      if inFront or (SX[FA[i]] and SX[FB[i]] and SX[FC[i]] and (d == 0 or SX[d])) then
        local lam = nx * lmx + ny * lmy + nz * lmz
        if facing < 0 then lam = -lam end
        if lam < 0 then lam = 0 end
        FCOL[i] = SHADES[FMAT[i]][(lam * levels + 1.5) // 1]
        n = n + 1
        KEY[n] = ((8 - (FCX[i] * a31 + FCY[i] * a32 + FCZ[i] * a33)) * 4096) // 1 * KEY_FACES + i
      end
    end
  end
  for i = n + 1, keyCount do KEY[i] = nil end
  keyCount = n
  if n > 1 then t_sort(KEY) end
end

---------------------------------------------------------------------------
-- Public.
---------------------------------------------------------------------------

function render.init(mesh, circuitModule)
  model = mesh
  track = circuitModule
  buildColors()
  hillRing = ringTables(HILLS)
end

-- Follow the car. dist scales the camera distance (rotary); dt smooths.
function render.track(h, dist, dt)
  local yawT = pi * 0.5 - h.yaw
  local dyaw = yawT - camYaw
  dyaw = (dyaw + pi) % (2 * pi) - pi
  local k = min(1, dt * CAM_LAG)
  camYaw = camYaw + dyaw * k
  if camYaw > pi then camYaw = camYaw - 2 * pi elseif camYaw < -pi then camYaw = camYaw + 2 * pi end
  camDist = camDist + ((dist or 1) - camDist) * min(1, dt * 4)
  local back, up = CAM_BACK * camDist, CAM_H * camDist
  local sy, cyw = sin(camYaw), cos(camYaw)
  eyeX, eyeY, eyeZ = h.px - sy * back, h.py - cyw * back, up
  camPitch = atan(LOOK_Z - up, back + LOOK_AHEAD)
  -- Widen the view with speed.
  local s = min(1, (h.speed or 0) / 50)
  hfov = HFOV_SLOW + (HFOV_FAST - HFOV_SLOW) * s
end

function render.snap(h, dist)
  camYaw = pi * 0.5 - h.yaw
  camDist = dist or 1
  render.track(h, dist, 0)
end

local lastPX, lastPY, lastYaw, lastRoll, lastPitch, lastCam, lastEX, lastEY, lastW, lastH, lastKey
local EPS = 1e-4

local function unchanged(h, w, hgt, key)
  return w == lastW and hgt == lastH and key == lastKey
    and abs(h.px - lastPX) < EPS and abs(h.py - lastPY) < EPS and abs(h.yaw - lastYaw) < EPS
    and abs(h.roll - lastRoll) < EPS and abs(h.pitch - lastPitch) < EPS
    and abs(camYaw - lastCam) < EPS and abs(eyeX - lastEX) < EPS and abs(eyeY - lastEY) < EPS
end

-- Build the frame's display lists. animKey changes whenever the car's
-- moving parts do. Returns false (keeping the last lists) when nothing has
-- moved.
function render.frame(h, w, hgt, animKey)
  if lastW and unchanged(h, w, hgt, animKey) then return false end
  lastPX, lastPY, lastYaw, lastRoll, lastPitch = h.px, h.py, h.yaw, h.roll, h.pitch
  lastCam, lastEX, lastEY, lastW, lastH, lastKey = camYaw, eyeX, eyeY, w, hgt, animKey
  scrW, scrH = w, hgt
  cx, cy = floor(w * 0.5), floor(hgt * 0.5)
  focal = (w * 0.5) / tan(hfov * 0.5)
  kx = (w * 0.5 + 8) / focal
  ky = (hgt * 0.5 + 8) / focal

  local sy, cyw = sin(camYaw), cos(camYaw)
  local sp, cp = sin(camPitch), cos(camPitch)
  cFx, cFy, cFz = cp * sy, cp * cyw, sp
  cRx, cRy = cyw, -sy
  cUx, cUy, cUz = -sp * sy, -sp * cyw, cp

  bandCount = 0
  local yPrev = elevY(BAND_R[1])
  for i = 1, #BAND_C do
    local y = elevY(BAND_R[i + 1])
    band(yPrev, y, BAND_C[i])
    yPrev = y
  end

  -- Ground layer.
  triCount = 0
  silhouette(hillRing, C.hills)
  circuit()
  -- Shadow: the car's footprint, a little larger.
  do
    local c, s = cos(h.yaw), sin(h.yaw)
    local hl, hw = model.HALF_LENGTH * 1.02, model.HALF_WIDTH * 0.95
    local ax, ay = c * hl, s * hl
    local bx, by = -s * hw, c * hw
    groundQuad(h.px + ax + bx, h.py + ay + by, h.px - ax + bx, h.py - ay + by,
      h.px - ax - bx, h.py - ay - by, h.px + ax - bx, h.py + ay - by, C.shadow)
  end
  triGround = triCount

  -- Car first (its depth splits the scenery), then scenery far to near.
  modelFrame(h)
  if facesPending then modelFaces() end

  local trees = track.TREES
  local fx, fy = cFx, cFy
  local draw2 = DRAW_DIST * DRAW_DIST
  local n = 0
  for i = 0, #trees do
    local x, y
    if i == 0 then x, y = track.CX[1], track.CY[1] else x, y = trees[i][1], trees[i][2] end
    local dx, dy = x - eyeX, y - eyeY
    local d2 = dx * dx + dy * dy
    if d2 < draw2 and dx * fx + dy * fy > 0 then
      n = n + 1
      SKEY[n] = (100000 - sqrt(d2) * 100) // 1 * SCENE_IDX + i
    end
  end
  for i = n + 1, skeyCount do SKEY[i] = nil end
  skeyCount = n
  if n > 1 then t_sort(SKEY) end
  triFar = -1
  local carKey = (100000 - carDepth * 100) // 1
  for k = 1, n do
    local key = SKEY[k]
    if triFar < 0 and key // SCENE_IDX > carKey then triFar = triCount end
    local i = key % SCENE_IDX
    if i == 0 then gantry() else tree(trees[i]) end
  end
  if triFar < 0 then triFar = triCount end
  return true
end

function render.dirty()
  lastW = nil
end

function render.setDetail(level)
  detail = (level == 2) and 2 or 1
  lastW = nil
end

local function drawTris(from, to)
  local last
  local dtri = lcd.drawFilledTriangle
  for i = from, to do
    local col = TCOL[i]
    if col ~= last then lcd.color(col); last = col end
    dtri(TX1[i], TY1[i], TX2[i], TY2[i], TX3[i], TY3[i])
  end
end

function render.paint(w, h)
  local rect = lcd.drawFilledRectangle
  for i = 1, bandCount do
    lcd.color(BCOL[i])
    rect(0, BY[i], w, BH[i])
  end
  drawTris(1, triFar)

  if modelMode == 2 then
    local FA, FB, FC, FD = model.FA, model.FB, model.FC, model.FD
    local dtri = lcd.drawFilledTriangle
    local last
    for k = 1, keyCount do
      local i = KEY[k] % KEY_FACES
      local col = FCOL[i]
      if col ~= last then lcd.color(col); last = col end
      local a, c = FA[i], FC[i]
      local ax, ay, cx3, cy3 = SX[a], SY[a], SX[c], SY[c]
      local b = FB[i]
      dtri(ax, ay, SX[b], SY[b], cx3, cy3)
      local d = FD[i]
      if d ~= 0 then dtri(ax, ay, cx3, cy3, SX[d], SY[d]) end
    end
  end

  drawTris(triFar + 1, triCount)
end

function render.stats()
  return triCount, keyCount
end

return render
