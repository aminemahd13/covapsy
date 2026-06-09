# simulation

Webots R2025a simulation of the TT-02 on the CoVAPSy track. The controller runs
the same follow-the-gap code as the real car (`common/gap_follower.py` +
`config.py`), so what you tune in the sim transfers to the hardware.

## Worlds
- `worlds/standalone.wbt`: single car (`TT02_green`). Use this to develop and
  tune the algorithm.
- `worlds/multicar.wbt`: four cars on a 2x2 grid, all running `covapsy_sim`.
  Race the same code against itself (each car sees the others as LiDAR obstacles).

## Run it
1. Install Webots R2025a (free, cyberbotics.com). The first load needs internet
   to fetch the `Car`, `RpLidarA2`, and background protos.
2. File > Open World > `simulation/worlds/standalone.wbt` (or `multicar.wbt`).
3. Press Play, click the 3D view for keyboard focus, then:
   - `A`: autonomous driving ON
   - `N`: stop (zero speed and steering)
4. Console heartbeat (~1 Hz): `step=... front=...m steer=... speed=... rear_blk=...`.

## How it's wired
- Each car is a `TT02_2025a` proto (`protos/`) with a LiDAR named `"RpLidarA2"`
  and `controller "covapsy_sim"`.
- `controllers/covapsy_sim/covapsy_sim.py` reads the LiDAR into the canonical
  360-float metre scan (index 0 = front, increasing index = left), calls
  `gap_follow()`, and applies the steering and speed. It finds the shared code by
  walking up the tree to `common/`.
- `runtime.ini` adds Webots' Driver (automobile) libraries to the path, required
  for `from vehicle import Driver`. Do not delete it.

## Headless smoke test
- `COVAPSY_AUTOSTART=1`: drive at boot (no keypress).
- `COVAPSY_LOGFILE=<path>`: mirror the heartbeat to a file (the Webots GUI
  process does not pipe stdout to a terminal on Windows).

## Tuning
Edit `common/config.py` (`steer_gain`, `steer_sign`, `front_fov_deg`,
`max_speed_mps`, `safety_radius_m`, and so on). The sim and the car read the same
file, so get it right here before driving the hardware.
