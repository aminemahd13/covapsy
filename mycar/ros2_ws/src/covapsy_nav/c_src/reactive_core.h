/**
 * reactive_core.h — Corridor-following reactive driving for COVAPSY
 *
 * High-performance C implementation for 1/10-scale RC car racing.
 *
 * Algorithm: Sector gap-finding + corridor centering + wall-ahead escape
 *            + potential-field repulsion + TTC speed control
 *
 * Compile: gcc -shared -fPIC -O2 -o libreactive_core.so reactive_core.c -lm
 */

#ifndef REACTIVE_CORE_H
#define REACTIVE_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define RC_MAX_SCAN    2048
#define RC_MAX_PAIRS   12

/* ── Tuning constants ── */

/* Corridor centering gain: lateral offset → steering.                      */
#define RC_K_CENTER          0.80f

/* Heading-follow gain: chase the best open direction.                      */
#define RC_K_HEADING         0.50f

/* Obstacle repulsion gain: emergency push away from close objects.         */
#define RC_K_REPULSION       1.20f

/* IMU yaw-rate damping gain: suppresses steering oscillations.             */
#define RC_K_IMU_DAMP        0.10f

/* Danger distance = car_width × this factor.  Below this, repulsion
 * kicks in with quadratic strength.                                        */
#define RC_REPULSION_DIST_FACTOR  2.5f

/* Forward clearance reference for speed = max_speed.                       */
#define RC_CLEARANCE_REF_M   2.00f

/* Speed penalties per unit curvature and per unit steering ratio.           */
#define RC_CURV_SPEED_PEN    0.35f
#define RC_STEER_SPEED_PEN   0.25f

/* Emergency stop distance (m).                                             */
#define RC_EMERGENCY_STOP_M  0.18f

/* Creep clearance: above this, enforce min_speed floor.                    */
#define RC_CREEP_CLEAR_M     0.35f

/* ── Data structures ── */

typedef struct {
    float speed;              /* m/s, >= 0                                */
    float steering;           /* rad, positive = left                     */
    float lateral_error;      /* normalised corridor offset [-1, 1]      */
    float forward_clearance;  /* m, fused LiDAR + depth                  */
    float corridor_width;     /* m, estimated passage width              */
} ReactiveResult;

typedef struct {
    float car_width;          /* m, including margin                     */
    float max_speed;          /* m/s                                     */
    float min_speed;          /* m/s                                     */
    float max_range;          /* m, LiDAR clip range                    */
    float max_steering;       /* rad, servo limit                       */
    float fov_deg;            /* degrees, total FOV (e.g. 200)          */
    float prev_steering;      /* rad, previous steering command         */
    float slew_rate;          /* rad, max steering change per tick      */
    float ttc_target;         /* seconds, time-to-collision target      */
    float depth_front;        /* m, depth camera front distance         */
    float camera_offset;      /* rad, camera border-detection offset    */
    float camera_gain;        /* gain applied to camera_offset          */
    float imu_yaw_rate;       /* rad/s, from IMU gyroscope              */
    float imu_speed;          /* m/s, estimated vehicle speed           */
} ReactiveParams;

/**
 * Core reactive driving computation.
 *
 * Takes a LiDAR scan (ranges[] + angles[] of length n), fuses with
 * depth camera and IMU, and produces (speed, steering) via:
 *
 *   1. Sector-based gap finding for escape direction
 *   2. Forward wall detection with progressive escape override
 *   3. Corridor centering from symmetric wall-distance pairs
 *   4. Weighted heading direction following
 *   5. Quadratic obstacle repulsion for lateral avoidance
 *   6. IMU yaw-rate damping for oscillation suppression
 *   7. TTC-limited speed with wall-ahead slowdown
 *
 * Thread-safe: no global state.  Pure function of inputs.
 */
void compute_reactive(
    const float* ranges_raw,
    const float* angles_raw,
    int n_raw,
    const ReactiveParams* params,
    ReactiveResult* out
);

#ifdef __cplusplus
}
#endif

#endif /* REACTIVE_CORE_H */
