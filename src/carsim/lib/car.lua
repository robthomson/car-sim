-- The car mesh: the low-poly car from gripora-lua-ethos-suite
-- (app/plane3d.lua, itself the Gripora Configurator's preview car,
-- src/js/car_model.js): orange body, dark cabin glass, a chassis plate,
-- four six-sided wheels with grey hubs, a rear wing on two struts,
-- headlights and red tail lights.
--
-- Parts are written in the glTF file's units (x = width, + is the left seen
-- from behind; y = up; z = aft, nose at -z) and converted at build to the
-- sim's body axes in metres: X = forward (nose), Y = left, Z = up, with the
-- origin on the ground under the middle of the car. At S = 0.68 the car is
-- about 4.2 m long and 2.4 m wide.
--
-- The interface is the one plane-sim's renderer expects (MX/MY/MZ, faces
-- with precomputed normals, centroids and plane offsets, coreFaces for the
-- low detail model). car.animate() turns the front wheels with the steering.
-- Built in stages (car.buildStep() until it returns true).

local cos, sin, sqrt, pi = math.cos, math.sin, math.sqrt, math.pi

local car = {}

local S = 0.68              -- glTF units -> metres
local GROUND = -0.94        -- glTF y of the bottom of the wheels

local function bx(gz) return -gz * S end
local function by(gx) return gx * S end
local function bz(gy) return (gy - GROUND) * S end

-- Materials (the renderer's MODEL_COLORS follow this order).
car.MAT_BODY = 1
car.MAT_DARK = 2
car.MAT_WHITE = 3
car.MAT_GLASS = 4
car.MAT_HUB = 5
car.MAT_TYRE = 6
car.MAT_TAIL = 7
car.MAT_DISC = -1           -- no translucent parts
local MAT_BODY, MAT_DARK, MAT_WHITE, MAT_GLASS, MAT_HUB, MAT_TYRE, MAT_TAIL = 1, 2, 3, 4, 5, 6, 7

local MX, MY, MZ = {}, {}, {}
car.MX, car.MY, car.MZ = MX, MY, MZ
car.vertexCount = 0
car.coreVertices, car.coreFaces = 0, 0

local FA, FB, FC, FD, FMAT, FBACK, FTWO = {}, {}, {}, {}, {}, {}, {}
local FNX, FNY, FNZ, FCX, FCY, FCZ, FPD = {}, {}, {}, {}, {}, {}, {}
car.FA, car.FB, car.FC, car.FD, car.FMAT, car.FBACK, car.FTWO = FA, FB, FC, FD, FMAT, FBACK, FTWO
car.FNX, car.FNY, car.FNZ = FNX, FNY, FNZ
car.FCX, car.FCY, car.FCZ, car.FPD = FCX, FCY, FCZ, FPD
car.faceCount = 0

-- Wheel centres {glTF x, glTF z}: front left, front right, rear left, rear right.
local WHEELS = {{1.50, -1.95}, {-1.50, -1.95}, {1.50, 2.00}, {-1.50, 2.00}}
local WHEEL_R, WHEEL_Y, WHEEL_HW = 0.62, -0.32, 0.28

-- For the physics (body axes, metres).
car.FRONT_AXLE = bx(WHEELS[1][2])          -- CG to front axle (forward, > 0)
car.REAR_AXLE = -bx(WHEELS[3][2])          -- CG to rear axle (> 0)
car.HALF_LENGTH = 3.05 * S
car.HALF_WIDTH = (1.50 + WHEEL_HW) * S
car.SIZE = 3.05 * S         -- the size the renderer judges detail by

local vertexCount, faceCount = 0, 0

local function vtx(gx, gy, gz)
  vertexCount = vertexCount + 1
  MX[vertexCount], MY[vertexCount], MZ[vertexCount] = bx(gz), by(gx), bz(gy)
  return vertexCount
end

local function pushFace(a, b, c, d, mat, twoSided)
  faceCount = faceCount + 1
  FA[faceCount], FB[faceCount], FC[faceCount], FD[faceCount] = a, b, c, d or 0
  FMAT[faceCount] = mat
  FBACK[faceCount] = false
  FTWO[faceCount] = twoSided
  return faceCount
end

local function plate(a, b, c, d, mat)
  return pushFace(a, b, c, d, mat, true)
end

-- One face of a closed part; (rx, ry, rz) is a glTF point inside it, used
-- to turn the face outwards.
local function solid(a, b, c, d, mat, rx, ry, rz)
  local ix, iy, iz = bx(rz), by(rx), bz(ry)
  local ux, uy, uz = MX[b] - MX[a], MY[b] - MY[a], MZ[b] - MZ[a]
  local vx, vy, vz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
  local nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
  local n = d and 4 or 3
  local gx = (MX[a] + MX[b] + MX[c] + (d and MX[d] or 0)) / n - ix
  local gy = (MY[a] + MY[b] + MY[c] + (d and MY[d] or 0)) / n - iy
  local gz = (MZ[a] + MZ[b] + MZ[c] + (d and MZ[d] or 0)) / n - iz
  if nx * gx + ny * gy + nz * gz < 0 then
    if d then a, b, c, d = a, d, c, b else b, c = c, b end
  end
  return pushFace(a, b, c, d, mat, false)
end

-- A closed prism: a side profile of {z, y} points (convex, in order around
-- it) at x = cx - hw and x = cx + hw; its sides in mat, its end caps in capMat.
local function prism(profile, cx, hw, mat, capMat)
  local n = #profile
  local left, right = {}, {}
  local cz, cy = 0, 0
  for i = 1, n do
    local p = profile[i]
    left[i] = vtx(cx + hw, p[2], p[1])
    right[i] = vtx(cx - hw, p[2], p[1])
    cz, cy = cz + p[1], cy + p[2]
  end
  cz, cy = cz / n, cy / n
  for i = 1, n do
    local j = (i % n) + 1
    solid(left[i], left[j], right[j], right[i], mat, cx, cy, cz)
  end
  for i = 2, n - 1 do
    solid(left[1], left[i], left[i + 1], nil, capMat, cx, cy, cz)
    solid(right[1], right[i], right[i + 1], nil, capMat, cx, cy, cz)
  end
end

---------------------------------------------------------------------------
-- Parts
---------------------------------------------------------------------------

local BODY = {{-3.05, -0.33}, {-3.05, 0.00}, {-2.20, 0.28}, {2.40, 0.38}, {3.00, 0.32}, {3.00, -0.33}}
local CABIN = {{-1.10, 0.30}, {-0.15, 0.92}, {1.25, 0.92}, {2.05, 0.36}}
local CHASSIS = {{-3.0, -0.51}, {-3.0, -0.33}, {3.0, -0.33}, {3.0, -0.51}}

-- Front wheels turn with the steering: their vertices, rest positions,
-- faces and centres.
local WV0, WV1, WF0, WF1 = {}, {}, {}, {}
local WCX, WCY = {}, {}
local RX, RY = {}, {}

local function wheel(k)
  local w = WHEELS[k]
  local hex = {}
  for i = 1, 6 do
    local a = (i - 1) * pi / 3
    hex[i] = {w[2] + WHEEL_R * cos(a), WHEEL_Y + WHEEL_R * sin(a)}
  end
  WV0[k], WF0[k] = vertexCount + 1, faceCount + 1
  prism(hex, w[1], WHEEL_HW, MAT_TYRE, MAT_HUB)
  WV1[k], WF1[k] = vertexCount, faceCount
  WCX[k], WCY[k] = bx(w[2]), by(w[1])
  for v = WV0[k], WV1[k] do RX[v], RY[v] = MX[v], MY[v] end
end

local function wing()
  plate(vtx(-1.35, 0.90, 2.45), vtx(1.35, 0.90, 2.45), vtx(1.35, 0.90, 3.05), vtx(-1.35, 0.90, 3.05), MAT_DARK)
  for _, x in ipairs({-0.75, 0.75}) do
    plate(vtx(x, 0.37, 2.61), vtx(x, 0.88, 2.61), vtx(x, 0.88, 2.89), vtx(x, 0.37, 2.89), MAT_DARK)
  end
end

local function lights()
  for _, s in ipairs({-1, 1}) do
    plate(vtx(0.62 * s, -0.05, -3.07), vtx(1.18 * s, -0.05, -3.07), vtx(1.18 * s, 0.09, -3.07), vtx(0.62 * s, 0.09, -3.07), MAT_WHITE)
    plate(vtx(0.62 * s, 0.14, 3.02), vtx(1.18 * s, 0.14, 3.02), vtx(1.18 * s, 0.26, 3.02), vtx(0.62 * s, 0.26, 3.02), MAT_TAIL)
  end
end

---------------------------------------------------------------------------
-- Normals
---------------------------------------------------------------------------

local function finishFace(i)
  local a, b, c, d = FA[i], FB[i], FC[i], FD[i]
  local nx, ny, nz, cx, cy, cz
  if d ~= 0 then
    local ux, uy, uz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
    local vx, vy, vz = MX[d] - MX[b], MY[d] - MY[b], MZ[d] - MZ[b]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    cx = (MX[a] + MX[b] + MX[c] + MX[d]) * 0.25
    cy = (MY[a] + MY[b] + MY[c] + MY[d]) * 0.25
    cz = (MZ[a] + MZ[b] + MZ[c] + MZ[d]) * 0.25
  else
    local ux, uy, uz = MX[b] - MX[a], MY[b] - MY[a], MZ[b] - MZ[a]
    local vx, vy, vz = MX[c] - MX[a], MY[c] - MY[a], MZ[c] - MZ[a]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    cx = (MX[a] + MX[b] + MX[c]) / 3
    cy = (MY[a] + MY[b] + MY[c]) / 3
    cz = (MZ[a] + MZ[b] + MZ[c]) / 3
  end
  local len = sqrt(nx * nx + ny * ny + nz * nz)
  if len > 0 then nx, ny, nz = nx / len, ny / len, nz / len end
  FNX[i], FNY[i], FNZ[i] = nx, ny, nz
  FCX[i], FCY[i], FCZ[i] = cx, cy, cz
  FPD[i] = nx * cx + ny * cy + nz * cz
end

---------------------------------------------------------------------------
-- Animation
---------------------------------------------------------------------------

local lastSteer

-- Turn the front wheels by steer radians (> 0 turns left).
function car.animate(steer)
  if steer == lastSteer then return end
  lastSteer = steer
  local c, s = cos(steer), sin(steer)
  for k = 1, 2 do
    local ox, oy = WCX[k], WCY[k]
    for v = WV0[k], WV1[k] do
      local dx, dy = RX[v] - ox, RY[v] - oy
      MX[v], MY[v] = ox + dx * c - dy * s, oy + dx * s + dy * c
    end
    for f = WF0[k], WF1[k] do finishFace(f) end
  end
end

---------------------------------------------------------------------------
-- Building
---------------------------------------------------------------------------

local STAGES = {
  function()
    vertexCount, faceCount = 0, 0
    prism(BODY, 0, 1.42, MAT_BODY, MAT_BODY)
    prism(CABIN, 0, 1.12, MAT_GLASS, MAT_GLASS)
  end,
  function()
    wheel(1) wheel(2) wheel(3) wheel(4)
    -- Up to here is the low detail model.
    car.coreVertices, car.coreFaces = vertexCount, faceCount
  end,
  function()
    prism(CHASSIS, 0, 1.35, MAT_DARK, MAT_DARK)
    wing()
    lights()
    car.vertexCount, car.faceCount = vertexCount, faceCount
  end,
  function()
    for i = 1, faceCount do finishFace(i) end
    lastSteer = nil
    car.animate(0)
  end,
}

local stage = 0
car.ready = false

function car.build()
  stage = 1
  car.ready = false
end

-- One build stage; true once the mesh is complete.
function car.buildStep()
  if stage == 0 then return car.ready end
  STAGES[stage]()
  if stage < #STAGES then
    stage = stage + 1
    return false
  end
  stage = 0
  car.ready = true
  return true
end

return car
