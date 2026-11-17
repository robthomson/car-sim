-- The circuit: a closed Catmull-Rom loop through control points, sampled
-- about every SEG_LEN metres. Each sample has its centre, unit tangent,
-- left normal, the track edges and the outer edges of the kerbs, and its
-- distance along the lap.
--
-- World axes: X east, Y north, metres. The lap runs anticlockwise and
-- starts at sample 1, on the main straight heading east.
--
-- track.nearest() finds the closest sample from a hint (the last one), so
-- following the car costs a few samples a step. Built in stages
-- (track.buildStep() until it returns true): the trees are placed a few at
-- a time, as each is checked against the whole circuit.

local sqrt, floor, min, max, abs = math.sqrt, math.floor, math.min, math.max, math.abs

local track = {}

track.HALF_WIDTH = 6.0       -- asphalt, either side of the centre line
track.KERB = 1.2             -- kerb width outside the asphalt
local SEG_LEN = 8

-- Control points (x, y). The lap starts mid-straight at (0, 0), heading
-- east: the points either side are level with it, so the spline's tangent
-- there is exactly east.
local POINTS = {
  {0, 0}, {120, 0},
  {190, 20}, {210, 80}, {170, 130}, {100, 120},
  {60, 160}, {80, 220}, {20, 260}, {-60, 240},
  {-100, 180}, {-160, 200}, {-220, 160}, {-230, 80}, {-190, 20}, {-120, 0},
}

-- Samples.
local CX, CY, TX, TY, NX, NY = {}, {}, {}, {}, {}, {}
local LX, LY, RX, RY = {}, {}, {}, {}           -- asphalt edges (left, right)
local KLX, KLY, KRX, KRY = {}, {}, {}, {}       -- kerb outer edges
local DIST = {}
track.CX, track.CY, track.TX, track.TY, track.NX, track.NY = CX, CY, TX, TY, NX, NY
track.LX, track.LY, track.RX, track.RY = LX, LY, RX, RY
track.KLX, track.KLY, track.KRX, track.KRY = KLX, KLY, KRX, KRY
track.DIST = DIST
track.count = 0
track.length = 0

-- Trees: x, y, height, width, colour index (1..3).
track.TREES = {}
local TREE_COUNT = 60

local function catmull(p0, p1, p2, p3, t)
  local t2, t3 = t * t, t * t * t
  local x = 0.5 * (2 * p1[1] + (p2[1] - p0[1]) * t + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
    + (3 * p1[1] - p0[1] - 3 * p2[1] + p3[1]) * t3)
  local y = 0.5 * (2 * p1[2] + (p2[2] - p0[2]) * t + (2 * p0[2] - 5 * p1[2] + 4 * p2[2] - p3[2]) * t2
    + (3 * p1[2] - p0[2] - 3 * p2[2] + p3[2]) * t3)
  return x, y
end

local function sampleSpline()
  local n = #POINTS
  local k = 0
  for i = 1, n do
    local p0 = POINTS[((i - 2) % n) + 1]
    local p1 = POINTS[i]
    local p2 = POINTS[(i % n) + 1]
    local p3 = POINTS[((i + 1) % n) + 1]
    local dx, dy = p2[1] - p1[1], p2[2] - p1[2]
    local steps = max(2, floor(sqrt(dx * dx + dy * dy) / SEG_LEN + 0.5))
    for s = 0, steps - 1 do
      k = k + 1
      CX[k], CY[k] = catmull(p0, p1, p2, p3, s / steps)
    end
  end
  track.count = k
end

local function finishSamples()
  local n = track.count
  local hw, kw = track.HALF_WIDTH, track.HALF_WIDTH + track.KERB
  local d = 0
  for i = 1, n do
    local a = (i == 1) and n or i - 1
    local b = (i == n) and 1 or i + 1
    local tx, ty = CX[b] - CX[a], CY[b] - CY[a]
    local l = sqrt(tx * tx + ty * ty)
    tx, ty = tx / l, ty / l
    TX[i], TY[i] = tx, ty
    NX[i], NY[i] = -ty, tx
    LX[i], LY[i] = CX[i] - ty * hw, CY[i] + tx * hw
    RX[i], RY[i] = CX[i] + ty * hw, CY[i] - tx * hw
    KLX[i], KLY[i] = CX[i] - ty * kw, CY[i] + tx * kw
    KRX[i], KRY[i] = CX[i] + ty * kw, CY[i] - tx * kw
    DIST[i] = d
    local ex, ey = CX[b] - CX[i], CY[b] - CY[i]
    d = d + sqrt(ex * ex + ey * ey)
  end
  track.length = d
end

-- Distance squared from (x, y) to the closest sample of the whole circuit.
local function clearance2(x, y)
  local best = 1e12
  for i = 1, track.count do
    local dx, dy = x - CX[i], y - CY[i]
    local d2 = dx * dx + dy * dy
    if d2 < best then best = d2 end
  end
  return best
end

local seed = 12345
local function rnd()
  seed = (seed * 1103515245 + 12345) % 2147483648
  return seed / 2147483648
end

-- Place up to `count` trees beside the circuit: next to a random sample,
-- out to one side, kept clear of every part of the track.
local function placeTrees(count)
  local trees = track.TREES
  local tries = 0
  while count > 0 and #trees < TREE_COUNT and tries < count * 4 do
    tries = tries + 1
    local i = floor(rnd() * track.count) + 1
    local side = (rnd() < 0.5) and -1 or 1
    local off = track.HALF_WIDTH + 12 + rnd() * 40
    local x = CX[i] + NX[i] * off * side
    local y = CY[i] + NY[i] * off * side
    if clearance2(x, y) > (track.HALF_WIDTH + 9) ^ 2 then
      local h = 9 + rnd() * 7
      trees[#trees + 1] = {x, y, h, h * 0.45, floor(rnd() * 3) + 1}
      count = count - 1
    end
  end
end

-- The sample nearest (x, y), searching out from hint. Returns the sample,
-- the signed offset from the centre line (> 0 left) and the distance along
-- the lap.
function track.nearest(x, y, hint)
  local n = track.count
  local best, bestD2 = hint, 1e12
  for k = -6, 6 do
    local i = ((hint - 1 + k) % n) + 1
    local dx, dy = x - CX[i], y - CY[i]
    local d2 = dx * dx + dy * dy
    if d2 < bestD2 then best, bestD2 = i, d2 end
  end
  local dx, dy = x - CX[best], y - CY[best]
  local along = dx * TX[best] + dy * TY[best]
  return best, dx * NX[best] + dy * NY[best], DIST[best] + along
end

-- A full search, for when the hint is lost (a reset).
function track.nearestAll(x, y)
  local best, bestD2 = 1, 1e12
  for i = 1, track.count do
    local dx, dy = x - CX[i], y - CY[i]
    local d2 = dx * dx + dy * dy
    if d2 < bestD2 then best, bestD2 = i, d2 end
  end
  return best
end

-- The grid: on the centre line 10 m behind the line, heading along the
-- straight. Returns x, y, heading and the nearest sample.
function track.start()
  local x, y = CX[1] - TX[1] * 10, CY[1] - TY[1] * 10
  return x, y, math.atan(TY[1], TX[1]), track.nearestAll(x, y)
end

local STAGES = {
  function()
    seed = 12345
    for k = #track.TREES, 1, -1 do track.TREES[k] = nil end
    sampleSpline()
    finishSamples()
  end,
}
for _ = 1, 6 do
  STAGES[#STAGES + 1] = function() placeTrees(TREE_COUNT // 6) end
end

local stage = 0
track.ready = false

function track.build()
  stage = 1
  track.ready = false
end

function track.buildStep()
  if stage == 0 then return track.ready end
  STAGES[stage]()
  if stage < #STAGES then
    stage = stage + 1
    return false
  end
  stage = 0
  track.ready = true
  return true
end

return track
