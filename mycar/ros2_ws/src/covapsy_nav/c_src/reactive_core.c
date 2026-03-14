/**
 * reactive_core.c — Corridor-following reactive driving for COVAPSY
 *
 * Replaces the gap-following algorithm that caused the car to get stuck.
 *
 * Key improvements over the old gap-follower:
 *   - No safety bubble that eliminates valid paths in narrow corridors
 *   - Continuous potential-field steering instead of discrete gap selection
 *   - Direct corridor centering from symmetric wall-distance pairs
 *   - Quadratic obstacle repulsion for fast emergency response
 *   - IMU yaw-rate damping to prevent steering oscillations
 *   - Simple, transparent speed control with few multiplicative factors
 *   - No global state — pure function, thread-safe
 *
 * Compile:
 *   gcc -shared -fPIC -O2 -o libreactive_core.so reactive_core.c -lm
 *   (or: cc -shared -fPIC -O2 -march=native -o libreactive_core.so reactive_core.c -lm)
 */

#include "reactive_core.h"
#include <math.h>
#include <string.h>

/* ── Internal constants ── */

/* Corridor model */
#define PAIR_HALF_WIDTH_RAD   0.105f   /* ~6° half-width per sector pair     */
#define N_ANGLE_PAIRS         9        /* symmetric pairs for corridor model */

/* Heading scoring */
#define HEADING_ANGLE_SCALE   0.70f    /* cos(angle * scale) for direction   */
#define MIN_USEFUL_RANGE      0.30f    /* ignore very short rays for heading */

/* Wall proximity blending */
#define CLOSE_WALL_FACTOR     2.0f     /* close_to_wall = wall < car_w * F  */
#define W_CENTER_CLOSE        0.75f    /* centering weight near walls        */
#define W_CENTER_OPEN         0.40f
#define W_HEADING_CLOSE       0.25f
#define W_HEADING_OPEN        0.60f

/* Speed computation */
#define FWD_CONE_RAD          0.2618f  /* ±15° forward cone for clearance    */
#define STEER_CONE_RAD        0.3142f  /* ±18° cone in steering direction    */

/* ── Helpers ── */

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }

/* ── Main computation ── */

void compute_reactive(
    const float* ranges_raw,
    const float* angles_raw,
    int n_raw,
    const ReactiveParams* p,
    ReactiveResult* out)
{
    memset(out, 0, sizeof(*out));

    if (n_raw < 10 || !ranges_raw || !angles_raw || !p)
        return;

    float fov_half = p->fov_deg * 0.5f * (float)M_PI / 180.0f;

    /* ================================================================
     * 1. PREPROCESS: filter to FOV, clean ranges
     * ================================================================ */
    float ranges[RC_MAX_SCAN];
    float angles[RC_MAX_SCAN];
    int n = 0;

    for (int i = 0; i < n_raw && n < RC_MAX_SCAN; i++) {
        float a = angles_raw[i];
        if (fabsf(a) > fov_half)
            continue;

        float r = ranges_raw[i];
        if (!isfinite(r) || r < 0.0f)
            r = p->max_range;
        if (r > p->max_range)
            r = p->max_range;

        ranges[n] = r;
        angles[n] = a;
        n++;
    }

    if (n < 10)
        return;

    /* ================================================================
     * 2. CORRIDOR MODEL: symmetric angle-pair wall distances
     *
     * For each angle α ∈ {10°, 20°, ..., 90°}:
     *   left_dist(α)  = min range in [+α - Δ, +α + Δ]
     *   right_dist(α) = min range in [-α - Δ, -α + Δ]
     *
     * Lateral error = weighted(left - right) / weighted(left + right)
     * This naturally centers the car in the corridor.
     * ================================================================ */
    float max_pair_angle = fov_half * 0.85f;
    float left_dist[RC_MAX_PAIRS];
    float right_dist[RC_MAX_PAIRS];
    float pair_angles[RC_MAX_PAIRS];
    int n_pairs = N_ANGLE_PAIRS;

    for (int k = 0; k < n_pairs; k++) {
        float alpha = max_pair_angle * (float)(k + 1) / (float)n_pairs;
        pair_angles[k] = alpha;

        float l_min = p->max_range;
        float r_min = p->max_range;

        for (int i = 0; i < n; i++) {
            /* Left side: rays near +alpha */
            if (fabsf(angles[i] - alpha) <= PAIR_HALF_WIDTH_RAD) {
                if (ranges[i] < l_min) l_min = ranges[i];
            }
            /* Right side: rays near -alpha */
            if (fabsf(angles[i] + alpha) <= PAIR_HALF_WIDTH_RAD) {
                if (ranges[i] < r_min) r_min = ranges[i];
            }
        }

        left_dist[k] = l_min;
        right_dist[k] = r_min;
    }

    /* Weighted lateral balance */
    float balance_num = 0.0f;
    float balance_den = 0.0f;
    float min_left_wall = p->max_range;
    float min_right_wall = p->max_range;

    for (int k = 0; k < n_pairs; k++) {
        float w = cosf(pair_angles[k]);
        balance_num += w * (left_dist[k] - right_dist[k]);
        balance_den += w * (left_dist[k] + right_dist[k]);

        if (left_dist[k] < min_left_wall)  min_left_wall = left_dist[k];
        if (right_dist[k] < min_right_wall) min_right_wall = right_dist[k];
    }

    float lateral_error = balance_num / maxf(balance_den, 0.3f);
    lateral_error = clampf(lateral_error, -1.0f, 1.0f);
    float min_wall = minf(min_left_wall, min_right_wall);
    float corridor_width = balance_den / maxf((float)n_pairs, 1.0f);

    out->lateral_error = lateral_error;
    out->corridor_width = corridor_width;

    /* ================================================================
     * 3. STEERING
     * ================================================================ */

    /* 3a. Corridor centering: steer toward corridor center */
    float steer_center = RC_K_CENTER * lateral_error * p->max_steering;

    /* 3b. Best heading direction (weighted average of good directions)
     *
     * Score each ray: range × cos(angle × scale)
     * This favors long-range, forward-facing directions.
     * Weighted average (not argmax!) gives smooth, continuous output. */
    float heading_num = 0.0f;
    float heading_den = 0.0f;

    for (int i = 0; i < n; i++) {
        if (ranges[i] < MIN_USEFUL_RANGE)
            continue;
        float score = ranges[i] * cosf(angles[i] * HEADING_ANGLE_SCALE);
        if (score < 0.01f)
            score = 0.01f;
        heading_num += score * angles[i];
        heading_den += score;
    }

    float best_heading = (heading_den > 0.01f)
                             ? heading_num / heading_den
                             : 0.0f;
    float steer_heading = RC_K_HEADING * best_heading;

    out->corridor_width = corridor_width;

    /* 3c. Blend centering and heading based on wall proximity.
     *     Close to walls → centering dominates (keep car safe).
     *     Open space → heading dominates (follow track direction). */
    int close_to_wall = (min_wall < p->car_width * CLOSE_WALL_FACTOR);
    float w_center, w_heading;

    if (close_to_wall) {
        w_center  = W_CENTER_CLOSE;
        w_heading = W_HEADING_CLOSE;
    } else {
        w_center  = W_CENTER_OPEN;
        w_heading = W_HEADING_OPEN;
    }

    float w_total = w_center + w_heading;
    float steering = (w_center * steer_center + w_heading * steer_heading) / w_total;

    /* 3d. Obstacle repulsion: quadratic force from very close obstacles.
     *     This overrides centering/heading when the car is about to hit
     *     something.  Quadratic force means stronger response when closer. */
    float danger_dist = p->car_width * RC_REPULSION_DIST_FACTOR;
    float repulsion_sum = 0.0f;

    for (int i = 0; i < n; i++) {
        if (ranges[i] < danger_dist) {
            float force = (danger_dist - ranges[i]) / danger_dist;
            force = force * force;   /* quadratic: emergency response */
            repulsion_sum -= force * sinf(angles[i]);
        }
    }
    steering += repulsion_sum * p->max_steering * RC_K_REPULSION;

    /* 3e. Camera border offset */
    if (fabsf(p->camera_offset) > 0.01f && isfinite(p->camera_offset)) {
        steering += p->camera_gain * p->camera_offset;
    }

    /* 3f. IMU yaw-rate damping: if car is already rotating, reduce
     *     additional steering in the same direction.  This suppresses
     *     oscillations that the old 4-layer filter tried to fix. */
    if (fabsf(p->imu_yaw_rate) > 0.05f) {
        steering -= RC_K_IMU_DAMP * p->imu_yaw_rate;
    }

    /* Clamp to servo limits */
    steering = clampf(steering, -p->max_steering, p->max_steering);

    /* Slew rate: mechanical servo speed limit.
     * This is the ONLY smoothing — no low-pass, no jerk limit,
     * no sign hysteresis.  The algorithm output is inherently smooth. */
    steering = clampf(steering,
                      p->prev_steering - p->slew_rate,
                      p->prev_steering + p->slew_rate);

    out->steering = steering;

    /* ================================================================
     * 4. SPEED CONTROL
     *
     * Simple and transparent: 3 factors × TTC guard.
     * No chain of 6+ multiplicative factors.
     * ================================================================ */

    /* 4a. Forward clearance in steering direction AND straight ahead */
    float steer_clear = p->max_range;
    float straight_clear = p->max_range;

    for (int i = 0; i < n; i++) {
        if (fabsf(angles[i] - steering) <= STEER_CONE_RAD) {
            if (ranges[i] < steer_clear)
                steer_clear = ranges[i];
        }
        if (fabsf(angles[i]) <= FWD_CONE_RAD) {
            if (ranges[i] < straight_clear)
                straight_clear = ranges[i];
        }
    }

    float effective_clear = minf(steer_clear, straight_clear);

    /* Fuse depth camera (takes the closer reading) */
    if (isfinite(p->depth_front) && p->depth_front > 0.0f) {
        effective_clear = minf(effective_clear, p->depth_front);
    }

    out->forward_clearance = effective_clear;

    /* 4b. Curvature estimate from corridor geometry + IMU */
    float curvature = fabsf(lateral_error);
    if (fabsf(p->imu_speed) > 0.15f) {
        float imu_curv = clampf(fabsf(p->imu_yaw_rate) * 2.5f, 0.0f, 1.0f);
        curvature = maxf(curvature, imu_curv * 0.6f);
    }

    /* 4c. Speed = max_speed × clearance × curvature × steering */
    float clearance_factor = clampf(effective_clear / RC_CLEARANCE_REF_M, 0.0f, 1.0f);
    float curvature_factor = 1.0f - RC_CURV_SPEED_PEN * curvature;
    float steer_factor = 1.0f - RC_STEER_SPEED_PEN * fabsf(steering) / maxf(p->max_steering, 0.01f);

    float speed = p->max_speed * clearance_factor * curvature_factor * steer_factor;
    speed = clampf(speed, 0.0f, p->max_speed);

    /* 4d. TTC guard: don't drive faster than safe stopping allows */
    float ttc_target = maxf(p->ttc_target, 0.25f);
    if (speed > 0.05f) {
        float ttc = effective_clear / speed;
        if (ttc < ttc_target) {
            speed = minf(speed, effective_clear / ttc_target);
        }
    }

    /* 4e. Speed floor when clear, emergency stop when close */
    if (effective_clear > RC_CREEP_CLEAR_M) {
        speed = maxf(p->min_speed, speed);
    } else if (effective_clear < RC_EMERGENCY_STOP_M) {
        speed = 0.0f;
    }

    speed = clampf(speed, 0.0f, p->max_speed);
    out->speed = speed;
}
