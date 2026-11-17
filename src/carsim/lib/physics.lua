-- Car physics: a single-track ("bicycle") model with tyre forces that
-- saturate, so the car grips, then slides.
--
-- World axes: X east, Y north, metres. h.yaw is the heading (0 east,
-- anticlockwise). Body velocities: vx forward, vy to the left; r is the
-- yaw rate (> 0 turns left). Forces are per unit mass (accelerations).
--
--   * front and rear axles each give a side force that grows with the slip
--     angle and is capped by grip times the axle's load
--   * the drive is to the rear axle and shares its grip (a friction
--     circle), so a lot of throttle in a corner steps the tail out
--   * the engine is power-limited, with aerodynamic and rolling drag; the
--     trigger pushed forward brakes, then reverses once stopped
--   * steering lock shrinks with speed, as in most racing games
--   * below a few m/s the car follows its wheels (kinematic), which keeps
--     parking-speed manoeuvres and reversing stable
--
-- The body leans and pitches with the accelerations (for show only).
-- step() allocates nothing.

local sqrt, abs, min, max, atan = math.sqrt, math.abs, math.min, math.max, math.atan
local sin, cos, tan = math.sin, math.cos, math.tan

local physics = {}

local G = 9.81
local A, B = 1.33, 1.36             -- CG to front and rear axle (m); set from the mesh
local K2 = A * B                    -- yaw inertia / mass (m^2)

local ACCEL = 8.5                   -- drive accel at low speed (m/s^2)
local POWER = 190                   -- drive accel x speed above that (m^2/s^3)
local DRAG = 0.0009                  -- aero drag (1/m): top speed about 57 m/s
local ROLL = 0.12                   -- rolling drag (1/s)
local BRAKE = 11                    -- full brake (m/s^2)
local REVERSE_ACCEL, REVERSE_MAX = 3, 6
local ENGINE_BRAKE = 0.6            -- off-throttle decel (m/s^2)

local CF, CR = 75, 90               -- cornering stiffness per axle (m/s^2 per rad)
local GRIP_ASPHALT, GRIP_KERB, GRIP_GRASS = 1.15, 1.0, 0.55
local GRASS_DRAG = 0.22             -- extra drag on the grass (1/s): about 60 km/h flat out
local TRACTION = 0.8                -- most drive the tyres take, x grip x g
local DRIVE_SHARE = 0.4            -- how much the drive eats into the rear grip

local MAX_STEER = 0.55              -- rad at standstill
local STEER_FADE = 22               -- m/s where the lock has halved
local STEER_RATE = 3.5              -- rad/s the wheels turn

local KIN_LO, KIN_HI = 1.5, 4.0     -- m/s: kinematic below, dynamic above

local ROLL_K, PITCH_K = 0.010, 0.006  -- body lean (rad per m/s^2)

physics.SURFACE_ASPHALT, physics.SURFACE_KERB, physics.SURFACE_GRASS = 1, 2, 3
local GRIPS = {GRIP_ASPHALT, GRIP_KERB, GRIP_GRASS}

function physics.setGeometry(mesh)
  A, B = mesh.FRONT_AXLE, mesh.REAR_AXLE
  K2 = A * B
end

local function updateMatrix(h)
  local cy, sy = cos(h.yaw), sin(h.yaw)
  local cp, sp = cos(h.pitch), sin(h.pitch)
  local cr, sr = cos(h.roll), sin(h.roll)
  -- Nose, left and up axes in the world (columns of R).
  local nx, ny, nz = cy * cp, sy * cp, sp
  local lx, ly = -sy, cy
  local ux, uy, uz = -cy * sp, -sy * sp, cp
  h.r11, h.r21, h.r31 = nx, ny, nz
  h.r12, h.r22, h.r32 = lx * cr + ux * sr, ly * cr + uy * sr, uz * sr
  h.r13, h.r23, h.r33 = ux * cr - lx * sr, uy * cr - ly * sr, uz * cr
end

function physics.reset(h, x, y, yaw)
  h.px, h.py, h.pz = x, y, 0
  h.yaw = yaw
  h.vx, h.vy, h.r = 0, 0, 0
  h.steer = 0
  h.pitch, h.roll = 0, 0
  h.speed = 0
  h.surface = physics.SURFACE_ASPHALT
  h.sliding = false
  updateMatrix(h)
end

-- One step. inp.steer: -1..1 (> 0 right); inp.trigger: -1..1 (> 0
-- throttle, < 0 brake / reverse); inp.steerScale: 0..1.
function physics.step(h, inp, dt)
  local vx, vy, r = h.vx, h.vy, h.r
  local grip = GRIPS[h.surface] or GRIP_ASPHALT

  -- Steering: the lock shrinks with speed; the wheels turn at a limited rate.
  local lock = MAX_STEER / (1 + abs(vx) / STEER_FADE)
  local target = -inp.steer * lock * inp.steerScale
  local ds = target - h.steer
  local dsMax = STEER_RATE * dt
  if ds > dsMax then ds = dsMax elseif ds < -dsMax then ds = -dsMax end
  local delta = h.steer + ds
  h.steer = delta

  -- Drive and brake.
  local trig = inp.trigger
  local drive, brake = 0, 0
  if trig > 0.02 then
    if vx < -0.3 then
      brake = trig * BRAKE
    else
      drive = trig * min(ACCEL, POWER / max(vx, 1))
      -- Traction control of a sort: the rear tyres can only push so hard,
      -- so full throttle on the grass does not just spin the car.
      local traction = TRACTION * grip * G
      if drive > traction then drive = traction end
    end
  elseif trig < -0.02 then
    if vx > 0.3 then
      brake = -trig * BRAKE
    elseif vx > -REVERSE_MAX then
      drive = trig * REVERSE_ACCEL
    end
  else
    brake = ENGINE_BRAKE
  end
  local speed = sqrt(vx * vx + vy * vy)
  local fx = drive - DRAG * vx * speed - ROLL * vx
  if h.surface == physics.SURFACE_GRASS then fx = fx - GRASS_DRAG * vx end
  -- Braking can stop the car but not reverse it.
  if brake > 0 then
    local bstep = brake * dt
    if abs(vx) <= bstep then
      fx, vx = 0, 0
    else
      fx = fx - ((vx > 0) and brake or -brake)
    end
  end

  -- Side forces (per unit mass), capped by grip x load.
  local L = A + B
  local nf, nr = grip * G * B / L, grip * G * A / L
  local u = max(abs(vx), 1)
  local af = atan((vy + A * r) / u) - delta
  local ar = atan((vy - B * r) / u)
  local fyf = -CF * af
  if fyf > nf then fyf = nf elseif fyf < -nf then fyf = -nf end
  local fxr = drive * DRIVE_SHARE
  local cap2 = nr * nr - fxr * fxr
  local cap = (cap2 > 0.04 * nr * nr) and sqrt(cap2) or 0.2 * nr
  local fyr = -CR * ar
  local sliding = false
  if fyr > cap then fyr, sliding = cap, true elseif fyr < -cap then fyr, sliding = -cap, true end
  if fyf >= nf or fyf <= -nf then sliding = true end

  local cd, sd = cos(delta), sin(delta)
  local ax = fx - fyf * sd
  local ay = fyf * cd + fyr
  vx = vx + (ax + vy * r) * dt
  vy = vy + (ay - vx * r) * dt
  r = r + (A * fyf * cd - B * fyr) / K2 * dt

  -- Slow: follow the wheels.
  local k = (abs(vx) - KIN_LO) / (KIN_HI - KIN_LO)
  if k < 1 then
    if k < 0 then k = 0 end
    local rk = vx * tan(delta) / L
    r = r * k + rk * (1 - k)
    vy = vy * k
    sliding = sliding and k > 0.5
  end

  h.vx, h.vy, h.r = vx, vy, r
  h.sliding = sliding and speed > 5
  h.speed = sqrt(vx * vx + vy * vy)
  local yaw = h.yaw + r * dt
  if yaw > math.pi then yaw = yaw - 2 * math.pi elseif yaw < -math.pi then yaw = yaw + 2 * math.pi end
  h.yaw = yaw
  local c, s = cos(yaw), sin(yaw)
  h.px = h.px + (vx * c - vy * s) * dt
  h.py = h.py + (vx * s + vy * c) * dt

  -- Body lean: outwards in a corner, nose up when accelerating.
  local kl = min(1, dt * 8)
  -- (roll > 0 lifts the left side: a left turn, ay > 0, leans it right)
  h.roll = h.roll + (ay * ROLL_K - h.roll) * kl
  h.pitch = h.pitch + (ax * PITCH_K - h.pitch) * kl
  updateMatrix(h)
end

-- Stop dead (hitting the wall at the edge of the world).
function physics.stop(h)
  h.vx, h.vy, h.r = 0, 0, 0
end

return physics
