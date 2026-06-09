"""Tuning constants shared by the Webots simulator and the on-car runtime.

See gap_follower.py for how each key is used.

LiDAR convention: list of 360 distances in metres, index = angle in degrees,
index 0 = straight ahead, increasing index = counter-clockwise (left), so
index 90 = 90 deg left, index 270 = 90 deg right. Value <= 0 or non-finite
means no return and is treated as max range.
"""

PARAMS = {
    # --- car geometry ---
    "max_steer_deg": 18.0,        # steering clamp, deg (TT-02 servo limit)
    "car_width_m": 0.30,          # used to inflate obstacles (disparity extension)

    # --- lidar conditioning ---
    "max_range_m": 5.0,           # clip ranges beyond this (track is < 5 m across)
    "min_valid_m": 0.05,          # readings closer than this treated as noise

    # --- gap finding ---
    # FOV kept < 180 deg, centred front: Pi and battery behind the LiDAR
    # occlude the rear, so rear/side-rear beams are unreliable.
    "front_fov_deg": 100,         # sector +/-100 deg so a corner-exit gap near +/-90 isn't truncated
    "gap_select_fov_deg": 90,     # max heading +/-90 deg (car's side, still in front of LiDAR)
    "disparity_threshold_m": 0.35,# depth jump that triggers obstacle inflation
    "safety_radius_m": 0.18,      # bubble carved around nearest obstacle
    "clearance_margin_m": 0.10,   # extra berth on half-car-width when inflating walls
                                  # (raise if it clips corners, lower if it won't fit gaps)
    "steer_gain": 1.0,            # proportional gain, gap angle to steering angle
    "aim_deep_bias": 0.5,         # 0 = aim gap center, 1 = aim deepest point

    # --- speed (conservative, to finish safely) ---
    "max_speed_mps": 1.0,         # cruise pace; ramp with --speed-cap. 2.0 = firmware soft cap
    "min_speed_mps": 0.4,         # floor speed in tight spots
    "reverse_speed_mps": 0.9,     # reverse speed, sim only (real car uses reverse_pwm_us)
    "reverse_pwm_us": 1250,       # real-car reverse pulse width, us; lower = faster reverse
    "ttc_target_s": 1.0,          # keep time-to-collision above this; raise = brake earlier

    # --- blocked / reverse detection ---
    "emergency_front_m": 0.30,    # front clearance below this == blocked
    "emergency_window_deg": 15,   # half-angle of the front-blocked cone
    "side_clear_min_deg": 20,     # side-clearance sampling window (for recovery dir)
    "side_clear_max_deg": 85,

    # --- runtime-only knobs (pi/drive.py and sim controller,
    #     NOT the pure gap_follow function) ---
    "loop_hz": 15,                # control loop rate, Hz (inside 250 ms watchdog)
    "steer_sign": 1,              # flip to -1 if the car steers the wrong way
    "lidar_front_offset_deg": 0,  # raw-lidar-angle to car-front calibration
    "rear_unknown_blocks": True,  # if rear sensor data missing, assume blocked
    "rear_sector_deg": 35,        # sim: +/- this around 180 deg == behind the car
    "rear_block_m": 0.5,          # closer than this behind == rear blocked
    "rear_self_ignore_m": 0.22,   # rear returns nearer than this = own body, ignore
                                  # (TT-02 shell occludes rear LiDAR; real car uses rear IR)
    "stuck_speed_mps": 0.05,      # measured speed below this while commanding fwd...
    "stuck_time_s": 2.0,          # ...for this long == stuck, trigger recovery

    # --- race control (race.py): stuck detection + recovery ---
    # Stuck = commanding forward but wheel encoder shows no rotation.
    "wheel_moving_pps": 5,        # encoder pulses/s below this == wheels not turning
    "stuck_secs": 3.0,            # commanding fwd but stalled this long == stuck
    "recovery_secs": 1.5,         # duration of the reverse-out recovery maneuver
    "control_port": 5005,         # TCP port for remote START/STOP
    "control_dir": "/tmp/covapsy",# control-file dir (touch start|stop|quit)
}
