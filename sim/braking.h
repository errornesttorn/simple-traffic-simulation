#pragma once

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ── constants ─────────────────────────────────────────────────── */

#define SIM_SAMPLES          96
#define SIM_SAMPLE_COUNT     (SIM_SAMPLES + 1)  /* 97 */
#define MAX_TRAJ_SAMPLES     22   /* ceil(3.0/0.15) + 1 */
#define MAX_HITBOX_CIRCLES   8
#define DEADLOCK_MAX_PATH    6

#define PREDICTION_HORIZON_S 3.0f
#define PREDICTION_STEP_S    0.15f
#define BLAME_ANGLE_THRESH   45.0f
#define BRAKE_DECEL_MULT     2.5f
#define ACCEL_ESC_LOOK_S     1.0f
#define ACCEL_ESC_SHORT_S    0.25f
#define ACCEL_ESC_MED_S      0.5f
#define WHEELBASE_FRAC       0.6f
#define BROAD_PHASE_SLACK_M  5.0f
#define ROUTE_COST_INF       1e30f

#define VEHICLE_CAR  0
#define VEHICLE_BUS  1

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

/* ── core types ────────────────────────────────────────────────── */

typedef struct { float x, y; } CVec2;

typedef struct {
    int id, priority, bus_only;
    CVec2 p0, p1, p2, p3;
    float length, speed_factor;
    CVec2 samples[SIM_SAMPLE_COUNT];
    float cum_len[SIM_SAMPLE_COUNT];
    CVec2 sample_tangents[SIM_SAMPLE_COUNT];
    float sample_curv[SIM_SAMPLE_COUNT];
    int hard_coupled_offset, hard_coupled_count;
} CSpline;

typedef struct {
    int current_spline_id, destination_spline_id;
    int prev_spline_ids[2];
    float distance_on_spline;
    CVec2 rear_position;
    float lateral_offset, speed, max_speed, accel, length, width;
    int vehicle_kind, lane_changing, lane_change_spline_id;
    int after_spline_id;
    float after_spline_dist;
    int has_trailer;
    float trailer_length, trailer_width;
    CVec2 trailer_rear_position;
    int route_id;
    int route_tree_index;
} CCar;

typedef struct {
    float time;
    CVec2 position, heading;
    int priority, spline_id, has_trailer;
    CVec2 trailer_position, trailer_heading;
} CTrajSample;

typedef struct {
    float body_radius, trailer_radius, coarse_radius;
    int body_offset_count, trailer_offset_count;
    float body_offsets[MAX_HITBOX_CIRCLES];
    float trailer_offsets[MAX_HITBOX_CIRCLES];
} CCollGeom;

typedef struct { float min_x, min_y, max_x, max_y; } CAABB;
typedef struct { CVec2 pos, heading; } CCarPose;
typedef struct { int from, to; } CBlameLink;

typedef struct {
    float time;
    CVec2 pos_a, pos_b, prev_pos_a, prev_pos_b;
    CVec2 heading_a, heading_b;
    int already_collided, priority_a, priority_b;
    int spline_a_id, spline_b_id;
} CCollisionPred;

/* ── graph ─────────────────────────────────────────────────────── */

typedef struct {
    float *costs;     /* [num_splines] */
    int   *next_hop;  /* [num_splines] */
} CRouteTree;

typedef struct {
    CSpline *splines;
    int      num_splines;
    int     *index_by_id;
    int      max_spline_id;
    int     *sbn_offsets;     /* per-spline start into sbn_data */
    int     *sbn_counts;      /* per-spline successor count     */
    int     *sbn_data;        /* flat successor indices          */
    int     *rev_offsets;     /* per-spline start into rev_data */
    int     *rev_counts;      /* per-spline predecessor count   */
    int     *rev_data;        /* flat predecessor indices        */
    int     *hard_coupled_ids;
    float   *segment_costs;   /* [num_splines], dynamic per frame */
} CGraph;

/* ── spatial grid ──────────────────────────────────────────────── */

typedef struct {
    int32_t cx, cy;
    int    *entries;
    int     count, cap;
    int     used;
} SGBucket;

typedef struct {
    float    cell_size, inv_cell_size;
    SGBucket *buckets;
    int       bucket_count;
    int       bucket_mask;
    int      used_count;
} CSpatialGrid;

/* ── pipeline context ──────────────────────────────────────────── */

typedef struct {
    int cars, base_predictions, stationary_predictions;
    int escape_predictions, faster_predictions;
    int total_predictions, total_prediction_samples;
    int primary_pair_candidates, primary_broad_phase_pairs;
    int primary_collision_checks, primary_collision_hits;
    int stationary_collision_checks, stationary_collision_hits;
    int escape_collision_checks, escape_collision_hits;
    int hold_collision_checks, hold_collision_hits;
    int initially_blamed_cars, braking_cars, hold_cars;
    double route_tree_ms, marshal_setup_ms, base_predict_ms, conflict_scan_ms, brake_probe_ms;
    double hold_probe_ms, finalize_ms, total_ms;
} CBrakingProfile;

typedef struct {
    /* inputs */
    CCar       *cars;
    int         num_cars;
    CGraph      graph;
    int        *route_tree_dest_ids;
    int        *route_tree_vehicle_kinds;
    int         num_route_trees;
    int         debug_selected_car;
    int         debug_selected_car_mode;

    /* output flags (pre-allocated, length = num_cars) */
    int *brake_flags;
    int *hold_flags;

    /* output blame links (pre-allocated) */
    CBlameLink *debug_links;
    int         debug_links_count, debug_links_cap;
    CBlameLink *hold_links;
    int         hold_links_count, hold_links_cap;
    CBlameLink *candidate_links;
    int         candidate_links_count, candidate_links_cap;

    /* output profile */
    CBrakingProfile profile;
} CBrakingCtx;

/* ── public API ────────────────────────────────────────────────── */

void compute_braking_decisions(CBrakingCtx *ctx);
