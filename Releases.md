# 0.2.0

- Higher frame rate on the radio:
  - The window is no longer cleared before every frame.
  - A new frame is built only once the last one is on screen; in between only the physics runs.
  - Less Lua work per frame (about a third less): track pieces outside the view are skipped before they are projected, only polygons that reach far off screen are clipped, distant track is drawn in longer pieces, and the front wheels and the body angle are worked out less often.
  - **Low** detail draws the far hills as a flat band and the car's shadow solid, and leaves out kerbs beyond 100 m.
- The car handles exactly as before.
- The CPU stats show frames and wakeups per second.

# 0.1.0

- First version of CarSim: a time-trial racing game seen from behind the car.
- Made for the FrSky Velox VX10 (wheel and trigger, touch-only buttons); also plays on stick radios (aileron, throttle, elevator to brake).
- Low-poly car from gripora-lua-ethos-suite, with steering front wheels and a ground shadow.
- Car physics: tyre grip that saturates, rear-wheel drive with power oversteer, speed-sensitive steering, brakes and reverse, slower grass.
- 1.25 km circuit with kerbs, trees and a start gantry; start lights, lap timing, best lap saved, wrong-way warning.
