-- CarSim: race a low-poly car round a circuit, seen from behind.
--
-- Made for the FrSky Velox VX10 (wheel steers, trigger pulled is throttle,
-- pushed is brake then reverse), and playable on stick radios (aileron
-- steers, the throttle stick is the throttle, elevator pulled back brakes
-- and reverses). The Controls setting picks one; Auto uses the wheel on a
-- VX10 and the sticks elsewhere.
--
-- The race is a time trial: the lights count down, then every lap is
-- timed and the best lap is kept. Enter goes back to the grid, Page opens
-- the settings, the rotary moves the camera, Exit closes the tool.
--
-- wakeup does all the work (inputs, physics, laps, camera, the frame's
-- display lists); paint only replays the lists and draws the HUD.

local car = assert(loadfile("lib/car.lua"))()
local track = assert(loadfile("lib/track.lua"))()
local physics = assert(loadfile("lib/physics.lua"))()
local render = assert(loadfile("lib/render.lua"))()

local floor, sqrt, min, max, abs = math.floor, math.sqrt, math.min, math.max, math.abs

local sim = {}

local STEP = 0.01                -- physics step (s)
local MAX_STEPS = 10             -- per wakeup; a longer gap slows time instead
local COUNTDOWN = 3              -- seconds of red lights
local WALL = 45                  -- metres from the centre line: the edge of the world

-- Analog sources by function, as Ethos numbers them. On the VX10 member 0
-- is the wheel and member 1 the trigger.
local WHEEL, TRIGGER = 0, 1
local MEMBER_ELE, MEMBER_THR, MEMBER_AIL = 1, 2, 3

local CONFIG_FILE = "carsim.cfg"

local CONTROL_NAMES = {"Auto", "Wheel + trigger", "Sticks"}
local SPEED_NAMES = {"km/h", "mph"}
local SPEED_K = {3.6, 2.23694}

local DEFAULTS = {
  controls = 1,       -- 1 auto, 2 wheel + trigger, 3 sticks
  steering = 100,     -- percent of the full lock
  expo = 20,          -- percent
  units = 1,          -- 1 km/h, 2 mph
  camera = 100,       -- camera distance, percent
  detail = 1,         -- 1 low, 2 high
  stats = false,
  best = 0,           -- best lap (ms), 0 none
}
local SAVE_ORDER = {"controls", "steering", "expo", "units", "camera", "detail", "stats", "best"}

---------------------------------------------------------------------------
-- Helpers
---------------------------------------------------------------------------

local function now()
  return os.clock()
end

local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function scriptDir()
  if not (debug and debug.getinfo) then return "" end
  local ok, info = pcall(debug.getinfo, 1, "S")
  if not ok or not info or type(info.source) ~= "string" then return "" end
  local source = info.source
  if source:sub(1, 1) == "@" then source = source:sub(2) end
  return source:match("^(.*[/\\])") or ""
end

local SCRIPT_DIR = scriptDir()

-- Absolute paths only: a bare file name resolves against the current
-- directory, which another script (a widget) may have changed.
local function configPaths()
  local paths = {"SCRIPTS:/carsim/" .. CONFIG_FILE, "/scripts/carsim/" .. CONFIG_FILE}
  if SCRIPT_DIR ~= "" then table.insert(paths, 1, SCRIPT_DIR .. CONFIG_FILE) end
  return paths
end

local function loadConfig()
  local cfg = {}
  for k, v in pairs(DEFAULTS) do cfg[k] = v end
  if not (io and io.open) then return cfg end
  local f
  for _, p in ipairs(configPaths()) do
    f = io.open(p, "r")
    if f then break end
  end
  if not f then return cfg end
  local lines = {}
  while true do
    local ok, line = pcall(f.read, f, "*l")
    if not ok or not line then break end
    lines[#lines + 1] = line
  end
  f:close()
  for _, line in ipairs(lines) do
    local key, value = line:match("^%s*([%w_]+)%s*=%s*(.-)%s*$")
    local d = key and DEFAULTS[key]
    if type(d) == "boolean" then
      cfg[key] = (value == "1" or value == "true")
    elseif type(d) == "number" then
      cfg[key] = tonumber(value) or d
    end
  end
  if not CONTROL_NAMES[cfg.controls] then cfg.controls = 1 end
  if not SPEED_NAMES[cfg.units] then cfg.units = 1 end
  cfg.steering = clamp(floor(cfg.steering), 20, 100)
  cfg.expo = clamp(floor(cfg.expo), 0, 90)
  cfg.camera = clamp(floor(cfg.camera), 50, 200)
  if cfg.detail ~= 2 then cfg.detail = 1 end
  cfg.best = max(0, floor(cfg.best))
  return cfg
end

local function saveConfig(cfg)
  if not (io and io.open) then return end
  local f
  for _, p in ipairs(configPaths()) do
    f = io.open(p, "w")
    if f then break end
  end
  if not f then return end
  for _, key in ipairs(SAVE_ORDER) do
    local v = cfg[key]
    if type(v) == "boolean" then v = v and "1" or "0" end
    f:write(key, "=", tostring(v), "\n")
  end
  f:close()
end

local function analogSource(member)
  if not (system and system.getSource) then return nil end
  local ok, src = pcall(system.getSource, {category = CATEGORY_ANALOG, member = member})
  if ok then return src end
  return nil
end

-- A source in -1..1 (sources give -1024..1024).
local function read(src)
  if not src then return 0 end
  local v = src:value()
  if type(v) ~= "number" then return 0 end
  v = v / 1024
  if v > 1 then return 1 elseif v < -1 then return -1 end
  return v
end

local function shape(v, expo)
  if v > -0.03 and v < 0.03 then return 0 end
  return v * (1 - expo) + v * v * v * expo
end

local function keyIs(value, ...)
  for i = 1, select("#", ...) do
    local k = select(i, ...)
    if k ~= nil and value == k then return true end
  end
  return false
end

local function isWheelRadio()
  if not (system and system.getVersion) then return false end
  local ok, v = pcall(system.getVersion)
  if not ok or type(v) ~= "table" then return false end
  local board = tostring(v.board or "") .. " " .. tostring(v.name or "")
  return board:upper():find("VX", 1, true) ~= nil
end

-- Whether the wheel and trigger are in use (else the sticks).
local function useWheel(state)
  local c = state.cfg.controls
  if c == 2 then return true elseif c == 3 then return false end
  return state.wheelRadio
end

local function lapText(ms)
  if not ms or ms <= 0 then return "-:--.--" end
  local cs = floor(ms / 10 + 0.5)
  return string.format("%d:%02d.%02d", cs // 6000, (cs // 100) % 60, cs % 100)
end

---------------------------------------------------------------------------
-- Race state
---------------------------------------------------------------------------

local function resetCar(state)
  local x, y, yaw, seg = track.start()
  physics.reset(state.h, x, y, yaw)
  state.seg = seg
  state.phase = "grid"
  state.timer = 0
  state.lap = 0
  state.lapTime = 0
  state.lastLap = nil
  state.halfway = false
  state.wrongTime = 0
  state.flash, state.flashTime = nil, 0
  render.snap(state.h, state.cfg.camera / 100)
  render.dirty()
end

-- Lap and track bookkeeping after a physics step.
local function followTrack(state)
  local h = state.h
  local n = track.count
  local prev = state.seg
  local seg, off = track.nearest(h.px, h.py, prev)
  state.seg = seg
  local a = abs(off)
  if a <= track.HALF_WIDTH then
    h.surface = physics.SURFACE_ASPHALT
  elseif a <= track.HALF_WIDTH + track.KERB then
    h.surface = physics.SURFACE_KERB
  else
    h.surface = physics.SURFACE_GRASS
  end
  if a > WALL then
    -- Off the edge of the world: back to the track, stopped.
    local i = track.nearestAll(h.px, h.py)
    h.px, h.py = track.CX[i], track.CY[i]
    h.yaw = math.atan(track.TY[i], track.TX[i])
    physics.stop(h)
    state.seg = i
    state.flash, state.flashTime = "Back on track", 2
  end
  if seg > n * 0.4 and seg < n * 0.6 then state.halfway = true end
  -- Crossing sample 1 (the line) forwards.
  if prev > n * 0.8 and seg < n * 0.2 then
    if (state.halfway or state.lap == 0) and state.phase == "race" then
      local ms = floor(state.lapTime * 1000 + 0.5)
      if state.lap >= 1 then
        state.lastLap = ms
        local best = state.cfg.best
        if best == 0 or ms < best then
          state.cfg.best = ms
          saveConfig(state.cfg)
          state.flash = "Best lap " .. lapText(ms)
          if system.playTone then pcall(system.playTone, 1200, 300, 0) end
        else
          state.flash = string.format("Lap %s  (+%.2f)", lapText(ms), (ms - best) / 1000)
          if system.playTone then pcall(system.playTone, 900, 150, 0) end
        end
        state.flashTime = 3
      end
      state.lap = state.lap + 1
      state.lapTime = 0
    end
    state.halfway = false
  elseif prev < n * 0.2 and seg > n * 0.8 then
    -- Backwards over the line: this lap no longer counts.
    state.halfway = false
  end
  -- Wrong way: moving against the track direction.
  local wx = h.vx * math.cos(h.yaw) - h.vy * math.sin(h.yaw)
  local wy = h.vx * math.sin(h.yaw) + h.vy * math.cos(h.yaw)
  local along = wx * track.TX[seg] + wy * track.TY[seg]
  if along < -3 then
    state.wrongTime = state.wrongTime + STEP
  else
    state.wrongTime = 0
  end
end

local function setFont(f)
  if f then lcd.font(f) end
end

-- HUD strings, rebuilt only when they change. True when any changed.
local function updateHud(state)
  local h, cfg = state.h, state.cfg
  local changed = false
  local spd = floor(h.speed * SPEED_K[cfg.units] + 0.5)
  if spd ~= state.hudSpd or cfg.units ~= state.hudUnits then
    state.hudSpd, state.hudUnits = spd, cfg.units
    state.hudSpeed = tostring(spd)
    state.hudSpeedUnit = SPEED_NAMES[cfg.units]
    changed = true
  end
  local cs = floor(state.lapTime * 100)
  if cs ~= state.hudCs or state.lap ~= state.hudLapN then
    state.hudCs, state.hudLapN = cs, state.lap
    state.hudLap = (state.lap >= 1) and ("LAP " .. state.lap) or "OUT LAP"
    state.hudTime = lapText(state.lapTime * 1000)
    changed = true
  end
  local best, last = cfg.best, state.lastLap or 0
  if best ~= state.hudBestMs or last ~= state.hudLastMs then
    state.hudBestMs, state.hudLastMs = best, last
    state.hudBest = "BEST " .. lapText(best)
    state.hudLast = "LAST " .. lapText(last)
    changed = true
  end

  local msg, sub
  if state.phase == "grid" and not state.wheelRadio then
    -- Keyed radios; the VX10 is touch only and has the buttons.
    msg, sub = nil, "Enter: restart   Page: settings"
  elseif state.flashTime > 0 and state.flash then
    msg = state.flash
  elseif state.wrongTime > 1 then
    msg = "WRONG WAY"
  end
  if msg ~= state.message or sub ~= state.subMessage then
    state.message, state.subMessage = msg, sub
    changed = true
  end
  local lights = (state.phase == "grid") and min(3, floor(state.timer) + 1) or ((state.timer < 1) and 4 or 0)
  if lights ~= state.hudLights then
    state.hudLights = lights
    changed = true
  end
  return changed
end

---------------------------------------------------------------------------
-- Settings
---------------------------------------------------------------------------

local function closeSettings(state)
  state.settingsOpen = false
  pcall(form.clear)
  state.lastTime = nil
  state.framePending = false
  render.dirty()
  lcd.invalidate()
end

local function openSettings(state)
  if not (form and form.clear and form.addLine and form.addChoiceField) then return end
  form.clear()
  state.settingsOpen = true
  local cfg = state.cfg
  local function set(key, v)
    cfg[key] = v
    saveConfig(cfg)
  end
  local function choice(label, names, key, after)
    local list = {}
    for i = 1, #names do list[i] = {names[i], i} end
    form.addChoiceField(form.addLine(label), nil, list,
      function() return cfg[key] end, function(v) set(key, v); if after then after(v) end end)
  end
  local function number(label, lo, hi, step, suffix, key)
    local f = form.addNumberField(form.addLine(label), nil, lo, hi,
      function() return cfg[key] end, function(v) set(key, v) end)
    if f then
      if f.step then f:step(step) end
      if f.suffix then f:suffix(suffix) end
      if f.default then f:default(DEFAULTS[key]) end
    end
  end
  choice("Controls", CONTROL_NAMES, "controls")
  number("Steering", 20, 100, 5, "%", "steering")
  number("Steering expo", 0, 90, 5, "%", "expo")
  number("Camera distance", 50, 200, 10, "%", "camera")
  choice("Speed", SPEED_NAMES, "units")
  choice("Detail", {"Low", "High"}, "detail", function(v) render.setDetail(v) end)
  if form.addBooleanField then
    form.addBooleanField(form.addLine("Show CPU stats"), nil,
      function() return cfg.stats end, function(v) set("stats", v) end)
  end
  local function addButton(text, press)
    if form.addButton then
      form.addButton(form.addLine(""), nil, {text = text, press = press})
    elseif form.addTextButton then
      form.addTextButton(form.addLine(""), nil, text, press)
    end
  end
  addButton("Clear best lap", function() set("best", 0) end)
  addButton("Back to the track", function() closeSettings(state) end)
end

---------------------------------------------------------------------------
-- Callbacks
---------------------------------------------------------------------------

function sim.create()
  local state = {
    cfg = loadConfig(),
    h = {},
    inp = {steer = 0, trigger = 0, steerScale = 1},
    phase = "boot",
    acc = 0,
    timer = 0,
    lap = 0,
    lapTime = 0,
    seg = 1,
    wrongTime = 0,
    flashTime = 0,
    nextAwake = 0,
    frameDt = 0,            -- time since the camera last moved
    framePending = false,   -- a frame is built and waits for its paint
    pendingAt = 0,
    wakes = 0, paints = 0, rateAt = 0, rate = 0, fps = 0,
    hudSpeed = "0", hudSpeedUnit = "", hudLap = "", hudTime = "", hudBest = "", hudLast = "",
    wheelRadio = isWheelRadio(),
    srcWheel = analogSource(WHEEL),
    srcTrigger = analogSource(TRIGGER),
    srcAil = analogSource(MEMBER_AIL),
    srcThr = analogSource(MEMBER_THR),
    srcEle = analogSource(MEMBER_ELE),
  }
  car.build()
  track.build()
  physics.setGeometry(car)
  render.init(car, track)
  render.setDetail(state.cfg.detail)
  return state
end

local function keepAwake(state, t)
  if t < state.nextAwake then return end
  state.nextAwake = t + 1
  if system.resetBacklightTimeout then pcall(system.resetBacklightTimeout) end
  if system.resetFocusTimeout then pcall(system.resetFocusTimeout) end
end

local function usage()
  return system.getInstructionsUsage and system.getInstructionsUsage() or 0
end

-- Steering (-1..1, > 0 right) and trigger (-1..1, > 0 throttle).
local function readInputs(state)
  local expo = state.cfg.expo / 100
  if useWheel(state) then
    return shape(read(state.srcWheel), expo), read(state.srcTrigger)
  end
  -- Sticks: the throttle stick runs bottom (-1) to top (1) and does not
  -- centre, so it is the throttle alone; elevator pulled back brakes.
  local gas = (read(state.srcThr) + 1) * 0.5
  local brake = -read(state.srcEle)
  if brake > 0.05 then return shape(read(state.srcAil), expo), -brake end
  return shape(read(state.srcAil), expo), gas
end

function sim.wakeup(state)
  if state.settingsOpen then return end
  local t = now()
  keepAwake(state, t)

  if state.phase == "boot" then
    local carDone = car.buildStep()
    local trackDone = track.buildStep()
    if carDone and trackDone then resetCar(state) end
    lcd.invalidate()
    return
  end

  local dt = state.lastTime and (t - state.lastTime) or 0
  state.lastTime = t
  if dt < 0 then dt = 0 elseif dt > MAX_STEPS * STEP then dt = MAX_STEPS * STEP end

  local h, inp = state.h, state.inp
  local steer, trig = readInputs(state)
  state.rawSteer, state.rawTrig = steer, trig
  inp.steer = steer
  inp.steerScale = state.cfg.steering / 100
  inp.trigger = (state.phase == "grid") and 0 or trig

  state.timer = state.timer + dt
  if state.phase == "grid" and state.timer >= COUNTDOWN then
    state.phase, state.timer = "race", 0
    if system.playTone then pcall(system.playTone, 1400, 400, 0) end
  elseif state.phase == "grid" and floor(state.timer) ~= floor(state.timer - dt) then
    if system.playTone then pcall(system.playTone, 700, 150, 0) end
  end
  if state.flashTime > 0 then state.flashTime = state.flashTime - dt end

  state.acc = state.acc + dt
  local n = 0
  while state.acc >= STEP and n < MAX_STEPS do
    physics.step(h, inp, STEP)
    followTrack(state)
    state.acc = state.acc - STEP
    n = n + 1
  end
  if state.phase == "race" then state.lapTime = state.lapTime + dt end

  local stats = state.cfg.stats
  if stats then
    state.wakes = state.wakes + 1
    if t >= state.rateAt then
      -- Once a second: wakeups and paints (frames) per second.
      state.rate, state.fps = state.wakes, state.paints
      state.wakes, state.paints, state.rateAt = 0, 0, t + 1
    end
  end

  -- Build the next frame only once the last one is on screen. On a radio a
  -- paint can outlast the wakeup interval, and a frame built in between
  -- would never be seen. (Should a paint never come, rebuild after a while.)
  state.frameDt = state.frameDt + dt
  if state.framePending and t - state.pendingAt < 0.5 then return end

  -- The wheels turn in 1.4 degree steps (under a pixel at the tyre's edge
  -- from the chase camera): each new angle recomputes their faces.
  local steer = floor(h.steer * 40 + 0.5) / 40
  car.animate(steer)
  physics.updateMatrix(h)
  local w, hgt = lcd.getWindowSize()
  render.track(h, state.cfg.camera / 100, state.frameDt)
  state.frameDt = 0
  local drew = render.frame(h, w, hgt, steer)
  local hudChanged = updateHud(state)

  if drew or hudChanged or stats then
    state.framePending, state.pendingAt = true, t
    -- A rectangle, not a plain invalidate(): that makes Ethos clear the
    -- window before the paint, and the scene covers every pixel anyway.
    lcd.invalidate(0, 0, w, hgt)
  end
end

local function shadowText(x, y, text, flags)
  lcd.color(BLACK)
  lcd.drawText(x + 1, y + 1, text, flags)
  lcd.color(WHITE)
  lcd.drawText(x, y, text, flags)
end

local LIGHT_OFF, LIGHT_RED, LIGHT_GREEN

local function drawLights(state, w, h)
  local lit = state.hudLights
  if not lit or lit == 0 then return end
  if not LIGHT_OFF then
    LIGHT_OFF, LIGHT_RED, LIGHT_GREEN = lcd.RGB(40, 40, 44), lcd.RGB(230, 30, 30), lcd.RGB(40, 220, 60)
  end
  local r = floor(h * 0.045)
  local y = floor(h * 0.30)
  lcd.color(BLACK)
  lcd.drawFilledRectangle(w // 2 - 4 * r - 6, y - r - 6, 8 * r + 12, 2 * r + 12)
  for i = 1, 3 do
    local x = w // 2 + (i - 2) * 3 * r
    if lit == 4 then lcd.color(LIGHT_GREEN) elseif i <= lit then lcd.color(LIGHT_RED) else lcd.color(LIGHT_OFF) end
    lcd.drawFilledCircle(x, y, r)
  end
end

-- Touch buttons, bottom left: the VX10 has no keys, only the touch screen.
local BUTTON_W, BUTTON_H, BUTTON_GAP = 120, 40, 8
local BUTTONS = {{"RESTART", "restart"}, {"SETUP", "settings"}}
local BUTTON_BG
local drawButtons

local function buttonRect(k, h)
  return BUTTON_GAP + (k - 1) * (BUTTON_W + BUTTON_GAP), h - BUTTON_H - BUTTON_GAP
end

drawButtons = function(w, h)
  if not BUTTON_BG then BUTTON_BG = lcd.RGB(0, 0, 0, 0.45) end
  setFont(FONT_STD or FONT_S)
  for k = 1, #BUTTONS do
    local x, y = buttonRect(k, h)
    lcd.color(BUTTON_BG)
    lcd.drawFilledRectangle(x, y, BUTTON_W, BUTTON_H)
    lcd.color(WHITE)
    lcd.drawText(x + BUTTON_W // 2, y + BUTTON_H // 2 - 11, BUTTONS[k][1], CENTERED)
  end
end

-- The button under a touch at (x, y), or nil.
local function buttonAt(x, y)
  if not (x and y) then return nil end
  local _, h = lcd.getWindowSize()
  for k = 1, #BUTTONS do
    local bx, by = buttonRect(k, h)
    -- A little slack round each button: they are small for a thumb.
    if x >= bx - 6 and x <= bx + BUTTON_W + 6 and y >= by - 10 and y <= by + BUTTON_H + 6 then
      return BUTTONS[k][2]
    end
  end
  return nil
end

function sim.paint(state)
  if state.settingsOpen then return end
  local w, h = lcd.getWindowSize()
  if state.phase == "boot" then
    lcd.color(lcd.RGB(30, 40, 60))
    lcd.drawFilledRectangle(0, 0, w, h)
    setFont(FONT_L)
    shadowText(w // 2, h // 2 - 12, "CarSim: loading", CENTERED)
    return
  end
  state.framePending = false
  state.paints = state.paints + 1
  render.paint(w, h)

  setFont(FONT_L)
  shadowText(8, 4, state.hudLap)
  shadowText(w - 8, 4, state.hudTime, RIGHT)
  setFont(FONT_S)
  shadowText(w - 8, 34, state.hudBest, RIGHT)
  shadowText(w - 8, 52, state.hudLast, RIGHT)
  setFont(FONT_XXL or FONT_XL or FONT_L)
  shadowText(w - 70, h - 64, state.hudSpeed, RIGHT)
  setFont(FONT_S)
  shadowText(w - 64, h - 30, state.hudSpeedUnit)

  drawLights(state, w, h)
  drawButtons(w, h)

  if state.cfg.stats then
    local tris, faces = render.stats()
    shadowText(6, h - BUTTON_H - 34, string.format("fps %d  wake %d/s  paint %d%%  tri %d  faces %d  mem %dk  S%+d T%+d %s",
      state.fps, state.rate, state.cpuPaint or 0, tris, faces, floor(collectgarbage("count")),
      floor((state.rawSteer or 0) * 100), floor((state.rawTrig or 0) * 100),
      useWheel(state) and "wheel" or "sticks"))
  end
  if state.message then
    setFont(FONT_L)
    shadowText(w // 2, floor(h * 0.18), state.message, CENTERED)
  end
  if state.subMessage then
    setFont(FONT_S)
    shadowText(w // 2, h - 22, state.subMessage, CENTERED)
  end
  if state.cfg.stats and system.getInstructionsUsage then
    state.cpuPaint = usage()
  end
end

local PAGE_KEYS = {KEY_PAGE_BREAK or 32, KEY_PAGE_LONG or 128}

function sim.event(state, category, value, x, y)
  if state.settingsOpen then
    if category == EVT_CLOSE or (category == EVT_KEY and keyIs(value, KEY_EXIT_BREAK, KEY_EXIT_FIRST)) then
      closeSettings(state)
      -- Exit arrives as FIRST then BREAK: the BREAK would close the tool.
      state.swallowExit = (value == KEY_EXIT_FIRST)
      return true
    end
    return false
  end

  if category == EVT_TOUCH then
    if value == TOUCH_END and state.phase ~= "boot" then
      local b = buttonAt(x, y)
      if b == "restart" then
        resetCar(state)
      elseif b == "settings" then
        openSettings(state)
      end
    end
    return true
  end
  if category ~= EVT_KEY then return false end

  if state.swallowExit and keyIs(value, KEY_EXIT_BREAK) then
    state.swallowExit = false
    return true
  end
  state.swallowExit = false

  -- Enter acts on the press: once a settings form has been open, Ethos no
  -- longer passes the Enter release (KEY_ENTER_BREAK) to the tool.
  if keyIs(value, KEY_ENTER_FIRST) then
    if state.phase ~= "boot" then resetCar(state) end
    return true
  end
  if keyIs(value, KEY_ENTER_BREAK, KEY_ENTER_LONG) then
    return true
  end
  if keyIs(value, PAGE_KEYS[1], PAGE_KEYS[2]) then
    openSettings(state)
    return true
  end
  if keyIs(value, KEY_PAGE_FIRST or 96) then
    return true
  end
  local cfg = state.cfg
  if keyIs(value, KEY_ROTARY_RIGHT) then
    cfg.camera = min(200, cfg.camera + 10)
    return true
  end
  if keyIs(value, KEY_ROTARY_LEFT) then
    cfg.camera = max(50, cfg.camera - 10)
    return true
  end
  return false
end

function sim.close(state)
  if state and state.settingsOpen then pcall(form.clear) end
  if state and state.cfg then saveConfig(state.cfg) end
  collectgarbage("collect")
end

return sim
