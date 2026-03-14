/**
 * reactive_core.c — Corridor-following reactive driving for COVAPSY
 *
 * Key fix over the previous version: FORWARD WALL DETECTION.
 *
 * The old algorithm had no concept of "wall ahead → must turn".  The corridor
 * model only measured left/right balance, and the heading averaged away
 * wall-ahead signals.  Result: the car drove straight into walls.
 *
 * New approach:
 *   1. Sector-based depth map for fast gap finding        (O(n))
 *   2. Forward wall detection in ±20° cone                (O(1) lookup)
 *   3. Escape direction = deepest sector with forward bias (O(sectors))
 *   4. Progressive override: as wall gets closer, escape steering dominates
 *   5. Corridor centering + heading for normal driving    (unchanged)
 *   6. Extended-range obstacle repulsion                  (tuned)
 *
 * Compile:
 *   gcc -shared -fPIC -O2 -o libreactive_core.so reactive_core.c -lm
 */

#include "reactive_core.h"
#include <math.h>
#include <string.h>

/* ── Sector-based gap finding ── */
#define N_SECTORS           36       /* FOV divided into 36 sectors (~5.6° each) */
#define SECTOR_SMOOTH_HW    2        /* smoothing half-window: ±2 sectors = 5 total (~28°) */

/* ── Wall-ahead detection ── */
#define WALL_AHEAD_CONE_RAD 0.35f    /* ±20° forward cone for wall detection     */
#define WALL_AHEAD_THRESH_M 1.5f     /* activation distance (m)                  */
#define WALL_AHEAD_CRIT_M   0.45f    /* full escape override distance (m)        */
#define ESCAPE_GAIN         1.8f     /* steering aggressiveness toward escape    */
#define ESCAPE_FWD_BIAS     0.3f     /* prefer more forward escape directions    */
#define WALL_SPEED_PENALTY  0.5f     /* max speed reduction from wall ahead      */

/* ── Corridor model ── */
#define PAIR_HALF_WIDTH_RAD 0.105f   /* ~6° half-width per sector pair           */
#define N_ANGLE_PAIRS       9        /* symmetric pairs for corridor model       */

/* ── Heading scoring ── */
#define HEADING_ANGLE_SCALE 0.55f    /* cos(angle * scale) for direction weight  */
#define MIN_USEFUL_RANGE    0.50f    /* exclude short (wall) rays from heading   */

/* ── Wall proximity blending ── */
#define CLOSE_WALL_FACTOR   2.0f     /* close_to_wall = wall < car_w * F        */
#define W_CENTER_CLOSE      0.75f
#define W_CENTER_OPEN       0.40f
#define W_HEADING_CLOSE     0.25f
#define W_HEADING_OPEN      0.60f

/* ── Speed computation ── */
#define FWD_CONE_RAD        0.2618f  /* ±15° forward cone for clearance          */
#define STEER_CONE_RAD      0.3142f  /* ±18° cone in steering direction          */

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
     * 2. SECTOR DEPTH MAP
     *
     * Divide the FOV into N_SECTORS sectors.  For each sector, compute
     * the sum and count of ranges (for averaging).  This is O(n) and
     * gives us a compact representation for gap-finding.
     * ================================================================ */
    float sector_sum[N_SECTORS];
    int   sector_count[N_SECTORS];
    memset(sector_sum, 0, sizeof(sector_sum));
    memset(sector_count, 0, sizeof(sector_count));

    float sector_width = (fov_half * 2.0f) / (float)N_SECTORS;

    for (int i = 0; i < n; i++) {
        int s = (int)((angles[i] + fov_half) / sector_width);
        if (s < 0) s = 0;
        if (s >= N_SECTORS) s = N_SECTORS - 1;

        sector_sum[s] += ranges[i];
        sector_count[s]++;
    }

    /* Smoothed sector averages: sliding window of ±SECTOR_SMOOTH_HW.
     * This makes the escape direction robust to single noisy rays. */
    float sector_avg[N_SECTORS];
    for (int s = 0; s < N_SECTORS; s++) {
        float total = 0.0f;
        int cnt = 0;
        int lo = s - SECTOR_SMOOTH_HW;
        int hi = s + SECTOR_SMOOTH_HW;
        if (lo < 0) lo = 0;
        if (hi >= N_SECTORS) hi = N_SECTORS - 1;

        for (int j = lo; j <= hi; j++) {
            total += sector_sum[j];
            cnt += sector_count[j];
        }
        sector_avg[s] = (cnt > 0) ? total / (float)cnt : 0.0f;
    }

    /* ================================================================
     * 3. FORWARD WALL DETECTION + ESCAPE DIRECTION
     *
     * This is the critical fix: detect walls ahead and find the best
     * direction to escape.  Without this, the car drives into walls.
     * ================================================================ */

    /* 3a. Forward wall: minimum range in ±20° cone */
    float fwd_min = p->max_range;
    for (int i = 0; i < n; i++) {
        if (fabsf(angles[i]) <= WALL_AHEAD_CONE_RAD) {
            if (ranges[i] < fwd_min)
                fwd_min = ranges[i];
        }
    }

    /* Fuse depth camera into forward wall detection */
    float fwd_min_fused = fwd_min;
    if (isfinite(p->depth_front) && p->depth_front > 0.0f) {
        fwd_min_fused = minf(fwd_min_fused, p->depth_front);
    }

    /* 3b. Wall-ahead urgency: 0 = clear, 1 = critical.
     * sqrt() curve gives faster initial ramp-up for earlier response. */
    float wall_urgency = 0.0f;
    if (fwd_min_fused < WALL_AHEAD_THRESH_M) {
        float raw = (WALL_AHEAD_THRESH_M - fwd_min_fused)
                   / (WALL_AHEAD_THRESH_M - WALL_AHEAD_CRIT_M);
        raw = clampf(raw, 0.0f, 1.0f);
        wall_urgency = sqrtf(raw);
    }

    /* 3c. Escape direction: sector with deepest smoothed average.
     * Forward bias (cos weighting) prefers less aggressive turns. */
    float escape_angle = 0.0f;
    float best_escape_score = -1.0f;

    for (int s = 0; s < N_SECTORS; s++) {
        float dir = -fov_half + ((float)s + 0.5f) * sector_width;
        float score = sector_avg[s] * (1.0f + ESCAPE_FWD_BIAS * cosf(dir));
        if (score > best_escape_score) {
            best_escape_score = score;
            escape_angle = dir;
        }
    }

    /* ================================================================
     * 4. CORRIDOR MODEL: symmetric angle-pair wall distances
     *
     * For each angle alpha in {~10°, ~20°, ..., ~85°}:
     *   left_dist(alpha)  = min range near +alpha
     *   right_dist(alpha) = min range near -alpha
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
            if (fabsf(angles[i] - alpha) <= PAIR_HALF_WIDTH_RAD) {
                if (ranges[i] < l_min) l_min = ranges[i];
            }
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
     * 5. STEERING
     * ================================================================ */

    /* 5a. Corridor centering: steer toward corridor center */
    float steer_center = RC_K_CENTER * lateral_error * p->max_steering;

    /* 5b. Best heading direction (weighted average of good directions) */
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

    /* 5c. Blend centering and heading (normal driving mode) */
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
    float steering_normal = (w_center * steer_center + w_heading * steer_heading) / w_total;

    /* 5d. WALL-AHEAD ESCAPE OVERRIDE
     *
     * When a wall is detected in the forward cone, progressively blend
     * toward escape steering (deepest sector direction).  sqrt urgency
     * curve gives faster initial response.
     *
     * wall_urgency = 0: pure normal driving
     * wall_urgency = 1: full escape steering (max steering toward gap)
     *
     * THIS is what prevents the car from driving straight into walls. */
    float steer_escape = clampf(
        escape_angle * ESCAPE_GAIN,
        -p->max_steering, p->max_steering);

    float steering = (1.0f - wall_urgency) * steering_normal
                   + wall_urgency * steer_escape;

    /* 5e. Obstacle repulsion: quadratic force from close obstacles.
     * Extended range (2.5× car width) for earlier lateral avoidance. */
    float danger_dist = p->car_width * RC_REPULSION_DIST_FACTOR;
    float repulsion_sum = 0.0f;

    for (int i = 0; i < n; i++) {
        if (ranges[i] < danger_dist) {
            float force = (danger_dist - ranges[i]) / danger_dist;
            force = force * force;   /* quadratic: stronger when closer */
            repulsion_sum -= force * sinf(angles[i]);
        }
    }
    steering += repulsion_sum * p->max_steering * RC_K_REPULSION;

    /* 5f. Camera border offset */
    if (fabsf(p->camera_offset) > 0.01f && isfinite(p->camera_offset)) {
        steering += p->camera_gain * p->camera_offset;
    }

    /* 5g. IMU yaw-rate damping */
    if (fabsf(p->imu_yaw_rate) > 0.05f) {
        steering -= RC_K_IMU_DAMP * p->imu_yaw_rate;
    }

    /* Clamp to servo limits */
    steering = clampf(steering, -p->max_steering, p->max_steering);

    /* Slew rate: mechanical servo speed limit */
    steering = clampf(steering,
                      p->prev_steering - p->slew_rate,
                      p->prev_steering + p->slew_rate);

    out->steering = steering;

    /* ================================================================
     * 6. SPEED CONTROL
     * ================================================================ */

    /* 6a. Forward clearance in steering direction AND straight ahead */
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

    /* Fuse depth camera */
    if (isfinite(p->depth_front) && p->depth_front > 0.0f) {
        effective_clear = minf(effective_clear, p->depth_front);
    }

    out->forward_clearance = effective_clear;

    /* 6b. Curvature estimate from corridor geometry + IMU */
    float curvature = fabsf(lateral_error);
    if (fabsf(p->imu_speed) > 0.15f) {
        float imu_curv = clampf(fabsf(p->imu_yaw_rate) * 2.5f, 0.0f, 1.0f);
        curvature = maxf(curvature, imu_curv * 0.6f);
    }

    /* 6c. Speed = max_speed × clearance × curvature × steering × wall_ahead */
    float clearance_factor = clampf(effective_clear / RC_CLEARANCE_REF_M, 0.0f, 1.0f);
    float curvature_factor = 1.0f - RC_CURV_SPEED_PEN * curvature;
    float steer_factor = 1.0f - RC_STEER_SPEED_PEN * fabsf(steering) / maxf(p->max_steering, 0.01f);
    float wall_speed_factor = 1.0f - WALL_SPEED_PENALTY * wall_urgency;

    float speed = p->max_speed * clearance_factor * curvature_factor
                * steer_factor * wall_speed_factor;
    speed = clampf(speed, 0.0f, p->max_speed);

    /* 6d. TTC guard */
    float ttc_target = maxf(p->ttc_target, 0.25f);
    if (speed > 0.05f) {
        float ttc = effective_clear / speed;
        if (ttc < ttc_target) {
            speed = minf(speed, effective_clear / ttc_target);
        }
    }

    /* 6e. Speed floor when clear, emergency stop when close */
    if (effective_clear > RC_CREEP_CLEAR_M) {
        speed = maxf(p->min_speed, speed);
    } else if (effective_clear < RC_EMERGENCY_STOP_M) {
        speed = 0.0f;
    }

    speed = clampf(speed, 0.0f, p->max_speed);
    out->speed = speed;
}
