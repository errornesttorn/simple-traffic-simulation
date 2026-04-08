#include "braking.h"
#include <omp.h>
#include <float.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════
   Vector math (static inline for maximum inlining)
   ═══════════════════════════════════════════════════════════════════ */

static inline CVec2 cv2(float x, float y) { return (CVec2){x, y}; }
static inline CVec2 vadd(CVec2 a, CVec2 b) { return cv2(a.x+b.x, a.y+b.y); }
static inline CVec2 vsub(CVec2 a, CVec2 b) { return cv2(a.x-b.x, a.y-b.y); }
static inline CVec2 vscale(CVec2 v, float s) { return cv2(v.x*s, v.y*s); }
static inline float vdot(CVec2 a, CVec2 b) { return a.x*b.x + a.y*b.y; }
static inline float vlen_sq(CVec2 v) { return v.x*v.x + v.y*v.y; }
static inline float vdist_sq(CVec2 a, CVec2 b) { float dx=a.x-b.x, dy=a.y-b.y; return dx*dx+dy*dy; }
static inline float cross2d(CVec2 a, CVec2 b) { return a.x*b.y - a.y*b.x; }

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float absf(float v) { return v < 0 ? -v : v; }

static inline CVec2 vnorm(CVec2 v) {
    float ls = v.x*v.x + v.y*v.y;
    if (ls <= 1e-9f) return cv2(1, 0);
    float inv = 1.0f / sqrtf(ls);
    return cv2(v.x*inv, v.y*inv);
}

static inline int32_t floori32(float v) {
    int32_t i = (int32_t)v;
    if (v < (float)i) i--;
    return i;
}

static inline float heading_angle_deg(CVec2 a, CVec2 b) {
    CVec2 da = vnorm(a), db = vnorm(b);
    float d = clampf(vdot(da, db), -1, 1);
    return acosf(d) * (180.0f / M_PIf);
}

static inline float signed_angle_deg(CVec2 from, CVec2 to) {
    CVec2 f = vnorm(from), t = vnorm(to);
    return atan2f(cross2d(f, t), vdot(f, t)) * (180.0f / M_PIf);
}

/* ═══════════════════════════════════════════════════════════════════
   Spline sampling
   ═══════════════════════════════════════════════════════════════════ */

static CVec2 bezier_point(CVec2 p0, CVec2 p1, CVec2 p2, CVec2 p3, float t) {
    float u = 1-t, uu = u*u, tt = t*t, uuu = uu*u, ttt = tt*t;
    return cv2(
        uuu*p0.x + 3*uu*t*p1.x + 3*u*tt*p2.x + ttt*p3.x,
        uuu*p0.y + 3*uu*t*p1.y + 3*u*tt*p2.y + ttt*p3.y);
}

/* Binary/linear search in cumulative length array */
static int spline_segment_index(const CSpline *s, float distance, int *cursor_spline_id, int *cursor_idx) {
    if (s->length <= 0) return 0;
    if (cursor_spline_id && *cursor_spline_id == s->id) {
        int idx = *cursor_idx;
        if (idx < 1) idx = 1;
        if (idx > SIM_SAMPLES) idx = SIM_SAMPLES;
        while (idx < SIM_SAMPLES && distance > s->cum_len[idx]) idx++;
        while (idx > 1 && distance <= s->cum_len[idx-1]) idx--;
        *cursor_idx = idx;
        return idx;
    }
    /* binary search */
    int lo = 0, hi = SIM_SAMPLES - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (distance <= s->cum_len[mid + 1]) hi = mid;
        else lo = mid + 1;
    }
    int idx = lo + 1;
    if (idx > SIM_SAMPLES) idx = SIM_SAMPLES;
    if (cursor_spline_id) {
        *cursor_spline_id = s->id;
        *cursor_idx = idx;
    }
    return idx;
}

/* Returns position, tangent, curvature */
static void sample_spline_state(const CSpline *s, float distance,
                                int *cursor_id, int *cursor_idx,
                                CVec2 *out_pos, CVec2 *out_tan, float *out_curv) {
    if (s->length <= 0) {
        *out_pos = s->p0;
        *out_tan = cv2(1, 0);
        *out_curv = 0;
        return;
    }
    if (distance < 0) distance = 0;
    if (distance > s->length) distance = s->length;
    int idx = spline_segment_index(s, distance, cursor_id, cursor_idx);
    if (idx <= 0) {
        *out_pos = s->p0;
        *out_tan = cv2(1, 0);
        *out_curv = 0;
        return;
    }
    float span = s->cum_len[idx] - s->cum_len[idx-1];
    float alpha = (span > 0) ? (distance - s->cum_len[idx-1]) / span : 0;
    float t0 = (float)(idx-1) / (float)SIM_SAMPLES;
    float t1 = (float)idx / (float)SIM_SAMPLES;
    float t = t0 + (t1 - t0) * alpha;
    *out_pos = bezier_point(s->p0, s->p1, s->p2, s->p3, t);
    CVec2 tan_prev = s->sample_tangents[idx-1];
    CVec2 tan_next = s->sample_tangents[idx];
    *out_tan = vnorm(cv2(
        tan_prev.x*(1-alpha) + tan_next.x*alpha,
        tan_prev.y*(1-alpha) + tan_next.y*alpha));
    *out_curv = s->sample_curv[idx-1]*(1-alpha) + s->sample_curv[idx]*alpha;
}

static void sample_spline_pos_tan(const CSpline *s, float distance, CVec2 *pos, CVec2 *tan) {
    float curv;
    sample_spline_state(s, distance, NULL, NULL, pos, tan, &curv);
}

static CVec2 nearest_sample_on_spline(const CSpline *s, CVec2 pos) {
    CVec2 best = s->samples[0];
    float best_d = vdist_sq(best, pos);
    for (int i = 1; i < SIM_SAMPLE_COUNT; i++) {
        float d = vdist_sq(s->samples[i], pos);
        if (d < best_d) { best_d = d; best = s->samples[i]; }
    }
    return best;
}

/* ═══════════════════════════════════════════════════════════════════
   Graph helpers
   ═══════════════════════════════════════════════════════════════════ */

static inline const CSpline *graph_spline_by_id(const CGraph *g, int id) {
    if (id < 0 || id > g->max_spline_id) return NULL;
    int idx = g->index_by_id[id];
    if (idx < 0) return NULL;
    return &g->splines[idx];
}

static inline int graph_index_by_id(const CGraph *g, int id) {
    if (id < 0 || id > g->max_spline_id) return -1;
    return g->index_by_id[id];
}

static inline int is_spline_usable(const CSpline *s, int vehicle_kind) {
    return vehicle_kind == VEHICLE_BUS || !s->bus_only;
}

/* ═══════════════════════════════════════════════════════════════════
   Route tree navigation
   ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    int idx;
    float dist;
} RouteHeapItem;

typedef struct {
    RouteHeapItem *items;
    int len;
    int cap;
} RouteHeap;

static void route_heap_push(RouteHeap *h, int idx, float dist) {
    if (h->len >= h->cap) {
        int next_cap = h->cap > 0 ? h->cap * 2 : 64;
        h->items = (RouteHeapItem *)realloc(h->items, sizeof(RouteHeapItem) * next_cap);
        h->cap = next_cap;
    }
    int at = h->len++;
    h->items[at] = (RouteHeapItem){idx, dist};
    while (at > 0) {
        int parent = (at - 1) / 2;
        if (h->items[parent].dist <= h->items[at].dist) break;
        RouteHeapItem tmp = h->items[parent];
        h->items[parent] = h->items[at];
        h->items[at] = tmp;
        at = parent;
    }
}

static RouteHeapItem route_heap_pop(RouteHeap *h) {
    RouteHeapItem out = h->items[0];
    h->len--;
    if (h->len > 0) {
        h->items[0] = h->items[h->len];
        int at = 0;
        for (;;) {
            int left = at * 2 + 1;
            int right = left + 1;
            int best = at;
            if (left < h->len && h->items[left].dist < h->items[best].dist) best = left;
            if (right < h->len && h->items[right].dist < h->items[best].dist) best = right;
            if (best == at) break;
            RouteHeapItem tmp = h->items[best];
            h->items[best] = h->items[at];
            h->items[at] = tmp;
            at = best;
        }
    }
    return out;
}

static CRouteTree build_route_tree(const CGraph *g, int destination_spline_id, int vehicle_kind) {
    int n = g->num_splines;
    int alloc_n = n > 0 ? n : 1;
    float *costs = (float *)malloc(sizeof(float) * alloc_n);
    int *next_hop = (int *)malloc(sizeof(int) * alloc_n);
    for (int i = 0; i < n; i++) {
        costs[i] = ROUTE_COST_INF;
        next_hop[i] = -1;
    }
    if (n == 0) {
        return (CRouteTree){costs, next_hop};
    }

    int dest_idx = graph_index_by_id(g, destination_spline_id);
    if (dest_idx < 0 || dest_idx >= n) {
        return (CRouteTree){costs, next_hop};
    }

    costs[dest_idx] = g->segment_costs[dest_idx];
    next_hop[dest_idx] = dest_idx;
    if (vehicle_kind == VEHICLE_CAR && g->splines[dest_idx].bus_only) {
        return (CRouteTree){costs, next_hop};
    }

    RouteHeap heap = {0};
    route_heap_push(&heap, dest_idx, costs[dest_idx]);
    while (heap.len > 0) {
        RouteHeapItem item = route_heap_pop(&heap);
        if (item.dist != costs[item.idx]) continue;
        int off = g->rev_offsets[item.idx];
        int cnt = g->rev_counts[item.idx];
        for (int i = 0; i < cnt; i++) {
            int prev_idx = g->rev_data[off + i];
            if (prev_idx < 0 || prev_idx >= n) continue;
            if (vehicle_kind == VEHICLE_CAR && g->splines[prev_idx].bus_only) continue;
            float alt = item.dist + g->segment_costs[prev_idx];
            if (alt < costs[prev_idx]) {
                costs[prev_idx] = alt;
                next_hop[prev_idx] = item.idx;
                route_heap_push(&heap, prev_idx, alt);
            }
        }
    }
    free(heap.items);
    return (CRouteTree){costs, next_hop};
}

static int choose_next_spline(const CGraph *g, const CRouteTree *tree,
                              int current_spline_id, int vehicle_kind) {
    const CSpline *cur = graph_spline_by_id(g, current_spline_id);
    if (!cur) return -1;
    int ci = graph_index_by_id(g, current_spline_id);
    if (ci < 0) return -1;
    int off = g->sbn_offsets[ci], cnt = g->sbn_counts[ci];
    int best_id = -1;
    float best_cost = ROUTE_COST_INF;
    for (int k = 0; k < cnt; k++) {
        int idx = g->sbn_data[off + k];
        if (idx < 0 || idx >= g->num_splines) continue;
        const CSpline *cand = &g->splines[idx];
        if (!is_spline_usable(cand, vehicle_kind)) continue;
        if (idx >= g->num_splines) continue;
        float cost = tree->costs[idx];
        if (cost >= ROUTE_COST_INF / 2) continue;
        if (best_id < 0 || cost < best_cost) {
            best_cost = cost;
            best_id = cand->id;
        }
    }
    return best_id;
}

static float path_cost_to_dest(const CGraph *g, const CRouteTree *tree,
                               int start_id, int dest_id, int vehicle_kind) {
    int si = graph_index_by_id(g, start_id);
    int ei = graph_index_by_id(g, dest_id);
    if (si < 0 || ei < 0) return ROUTE_COST_INF;
    if (vehicle_kind == VEHICLE_CAR && si != ei && g->splines[ei].bus_only)
        return ROUTE_COST_INF;
    if (si >= g->num_splines) return ROUTE_COST_INF;
    float c = tree->costs[si];
    return (c >= ROUTE_COST_INF / 2) ? ROUTE_COST_INF : c;
}

static int find_forced_lane_change(const CGraph *g, const CRouteTree *tree,
                                   int current_spline_id, int dest_spline_id,
                                   int vehicle_kind, int *out_desired) {
    const CSpline *cur = graph_spline_by_id(g, current_spline_id);
    if (!cur) return -1;
    int ci = graph_index_by_id(g, current_spline_id);
    if (ci < 0) return -1;
    int off = g->sbn_offsets[ci], cnt = g->sbn_counts[ci];
    for (int k = 0; k < cnt; k++) {
        int next_idx = g->sbn_data[off + k];
        if (next_idx < 0 || next_idx >= g->num_splines) continue;
        const CSpline *next_sp = &g->splines[next_idx];
        if (!is_spline_usable(next_sp, vehicle_kind)) continue;
        int hc_off = next_sp->hard_coupled_offset;
        int hc_cnt = next_sp->hard_coupled_count;
        for (int h = 0; h < hc_cnt; h++) {
            int coupled_id = g->hard_coupled_ids[hc_off + h];
            const CSpline *coupled = graph_spline_by_id(g, coupled_id);
            if (coupled && !is_spline_usable(coupled, vehicle_kind)) continue;
            float c = path_cost_to_dest(g, tree, coupled_id, dest_spline_id, vehicle_kind);
            if (c < ROUTE_COST_INF) {
                *out_desired = coupled_id;
                return next_sp->id;
            }
        }
    }
    return -1;
}

/* ═══════════════════════════════════════════════════════════════════
   Collision geometry
   ═══════════════════════════════════════════════════════════════════ */

static float hitbox_radius(float width) { return width / 2 + 0.5f; }

static int hitbox_circle_offsets(float length, float width, float *out, int max_out) {
    float r = hitbox_radius(width);
    float span = 2 * (length / 2 + 1.0f - r);
    if (span < 0) span = 0;
    float max_spacing = 2 * r + 1.0f;
    int count = (int)ceilf(span / max_spacing) + 1;
    if (count < 2) count = 2;
    if (count > max_out) count = max_out;
    float start = -span / 2;
    float step = (count > 1) ? span / (float)(count - 1) : 0;
    for (int i = 0; i < count; i++) out[i] = start + (float)i * step;
    return count;
}

static CCollGeom build_collision_geometry(const CCar *car) {
    CCollGeom g;
    memset(&g, 0, sizeof(g));
    g.body_radius = hitbox_radius(car->width);
    g.body_offset_count = hitbox_circle_offsets(car->length, car->width, g.body_offsets, MAX_HITBOX_CIRCLES);
    g.coarse_radius = car->length / 2 + 1.0f;
    if (car->has_trailer) {
        g.trailer_radius = hitbox_radius(car->trailer_width);
        g.trailer_offset_count = hitbox_circle_offsets(car->trailer_length, car->trailer_width, g.trailer_offsets, MAX_HITBOX_CIRCLES);
        g.coarse_radius += car->trailer_length + 1.0f;
    }
    return g;
}

/* ═══════════════════════════════════════════════════════════════════
   Trajectory prediction
   ═══════════════════════════════════════════════════════════════════ */

static int predict_car_trajectory(const CCar *car, const CGraph *g,
                                  const CRouteTree *tree,
                                  float horizon, float step,
                                  CTrajSample *out) {
    if (g->num_splines == 0 || car->current_spline_id < 0) return 0;
    int steps = (int)ceilf(horizon / step);
    if (steps + 1 > MAX_TRAJ_SAMPLES) steps = MAX_TRAJ_SAMPLES - 1;

    /* Mutable simulation state — copied from car */
    int    sim_spline_id     = car->current_spline_id;
    int    sim_dest_id       = car->destination_spline_id;
    float  sim_dist          = car->distance_on_spline;
    float  sim_lateral       = car->lateral_offset;
    CVec2  sim_rear          = car->rear_position;
    CVec2  sim_trailer_rear  = car->trailer_rear_position;
    float  speed             = car->speed > 0 ? car->speed : 0;
    int    active            = 1;
    int    sim_lane_changing = car->lane_changing;
    int    sim_lc_spline     = car->lane_change_spline_id;
    int    sim_after_id      = car->after_spline_id;
    float  sim_after_dist    = car->after_spline_dist;
    int    sim_prev[2]       = { car->prev_spline_ids[0], car->prev_spline_ids[1] };
    int    sim_vk            = car->vehicle_kind;

    int cursor_id = -1, cursor_idx = 0;

    /* Stationary car — replicate single position */
    if (speed <= 0.01f) {
        const CSpline *sp = graph_spline_by_id(g, sim_spline_id);
        if (!sp) return 0;
        CVec2 pos, tan; float curv;
        sample_spline_state(sp, sim_dist, &cursor_id, &cursor_idx, &pos, &tan, &curv);
        CVec2 rn = cv2(tan.y, -tan.x);
        CVec2 fp = vadd(pos, vscale(rn, sim_lateral));
        CVec2 r2f = vsub(fp, sim_rear);
        if (vlen_sq(r2f) > 1e-9f)
            sim_rear = vsub(fp, vscale(vnorm(r2f), car->length * WHEELBASE_FRAC));
        else
            sim_rear = vsub(fp, vscale(tan, car->length * WHEELBASE_FRAC));
        CVec2 center = vscale(vadd(fp, sim_rear), 0.5f);
        CVec2 bh = vnorm(vsub(fp, sim_rear));
        CTrajSample base;
        memset(&base, 0, sizeof(base));
        base.position = center;
        base.heading = bh;
        base.priority = sp->priority;
        base.spline_id = sp->id;
        if (car->has_trailer) {
            CVec2 hitch = sim_rear;
            CVec2 t2h = vsub(hitch, sim_trailer_rear);
            if (vlen_sq(t2h) > 1e-9f)
                sim_trailer_rear = vsub(hitch, vscale(vnorm(t2h), car->trailer_length * WHEELBASE_FRAC));
            else
                sim_trailer_rear = vsub(hitch, vscale(tan, car->trailer_length * WHEELBASE_FRAC));
            base.has_trailer = 1;
            base.trailer_position = vscale(vadd(hitch, sim_trailer_rear), 0.5f);
            base.trailer_heading = vnorm(vsub(hitch, sim_trailer_rear));
        }
        for (int si = 0; si <= steps; si++) {
            base.time = (float)si * step;
            out[si] = base;
        }
        return steps + 1;
    }

    int count = 0;
    for (int si = 0; si <= steps; si++) {
        const CSpline *sp = graph_spline_by_id(g, sim_spline_id);
        if (!sp) break;
        CVec2 pos, tan; float kappa;
        sample_spline_state(sp, sim_dist, &cursor_id, &cursor_idx, &pos, &tan, &kappa);
        float wb = car->length * WHEELBASE_FRAC;
        float target_off = kappa * wb * wb / 6;
        if (car->has_trailer) {
            float tw = car->trailer_length * WHEELBASE_FRAC;
            target_off = kappa * (wb*wb + tw*tw) / 6;
        }
        float lerp_rate = speed / (2 * (wb + 0.01f));
        float blend = lerp_rate * step;
        if (blend > 1) blend = 1;
        sim_lateral += (target_off - sim_lateral) * blend;
        CVec2 rn = cv2(tan.y, -tan.x);
        CVec2 fp = vadd(pos, vscale(rn, sim_lateral));
        CVec2 r2f = vsub(fp, sim_rear);
        if (vlen_sq(r2f) > 1e-9f)
            sim_rear = vsub(fp, vscale(vnorm(r2f), car->length * WHEELBASE_FRAC));
        else
            sim_rear = vsub(fp, vscale(tan, car->length * WHEELBASE_FRAC));
        CVec2 center = vscale(vadd(fp, sim_rear), 0.5f);
        CVec2 bh = vnorm(vsub(fp, sim_rear));

        CTrajSample *s = &out[count];
        memset(s, 0, sizeof(*s));
        s->time = (float)si * step;
        s->position = center;
        s->heading = bh;
        s->priority = sp->priority;
        s->spline_id = sp->id;

        if (car->has_trailer) {
            CVec2 hitch = sim_rear;
            CVec2 t2h = vsub(hitch, sim_trailer_rear);
            if (vlen_sq(t2h) > 1e-9f)
                sim_trailer_rear = vsub(hitch, vscale(vnorm(t2h), car->trailer_length * WHEELBASE_FRAC));
            else
                sim_trailer_rear = vsub(hitch, vscale(tan, car->trailer_length * WHEELBASE_FRAC));
            s->has_trailer = 1;
            s->trailer_position = vscale(vadd(hitch, sim_trailer_rear), 0.5f);
            s->trailer_heading = vnorm(vsub(hitch, sim_trailer_rear));
        }
        count++;

        if (!active) break;
        if (si == steps || speed <= 0.01f) continue;

        float move = speed * step;
        while (move > 0 && active) {
            const CSpline *csp = graph_spline_by_id(g, sim_spline_id);
            if (!csp) { active = 0; break; }
            float remaining = csp->length - sim_dist;
            if (move <= remaining) { sim_dist += move; move = 0; break; }
            move -= remaining;

            if (sim_spline_id == sim_dest_id) {
                sim_dist = csp->length;
                active = 0;
                break;
            }

            sim_dist = 0;
            cursor_id = -1;

            if (sim_lane_changing && sim_spline_id == sim_lc_spline) {
                sim_prev[1] = sim_prev[0];
                sim_prev[0] = sim_spline_id;
                sim_spline_id = sim_after_id;
                sim_dist = sim_after_dist;
                sim_lane_changing = 0;
                sim_lc_spline = -1;
                continue;
            }

            int next_id = choose_next_spline(g, tree, sim_spline_id, sim_vk);
            if (next_id < 0) {
                int desired;
                next_id = find_forced_lane_change(g, tree, sim_spline_id, sim_dest_id, sim_vk, &desired);
                if (next_id < 0) { active = 0; break; }
            }
            sim_spline_id = next_id;
        }
    }
    return count;
}

/* ═══════════════════════════════════════════════════════════════════
   Trajectory AABB
   ═══════════════════════════════════════════════════════════════════ */

static CAABB build_traj_aabb(const CTrajSample *s, int n, float cr) {
    if (n == 0) return (CAABB){0,0,0,0};
    CAABB bb = { s[0].position.x, s[0].position.y,
                 s[0].position.x, s[0].position.y };
    for (int i = 1; i < n; i++) {
        float x = s[i].position.x, y = s[i].position.y;
        if (x < bb.min_x) bb.min_x = x;
        if (x > bb.max_x) bb.max_x = x;
        if (y < bb.min_y) bb.min_y = y;
        if (y > bb.max_y) bb.max_y = y;
        if (s[i].has_trailer) {
            float tx = s[i].trailer_position.x, ty = s[i].trailer_position.y;
            if (tx < bb.min_x) bb.min_x = tx;
            if (tx > bb.max_x) bb.max_x = tx;
            if (ty < bb.min_y) bb.min_y = ty;
            if (ty > bb.max_y) bb.max_y = ty;
        }
    }
    bb.min_x -= cr; bb.min_y -= cr;
    bb.max_x += cr; bb.max_y += cr;
    return bb;
}

static inline int aabb_overlap(CAABB a, CAABB b) {
    return a.min_x <= b.max_x && a.max_x >= b.min_x &&
           a.min_y <= b.max_y && a.max_y >= b.min_y;
}

/* ═══════════════════════════════════════════════════════════════════
   Collision detection
   ═══════════════════════════════════════════════════════════════════ */

static int check_circle_groups(CVec2 cen_a, CVec2 h_a, const float *offs_a, int n_a, float r_a,
                               CVec2 cen_b, CVec2 h_b, const float *offs_b, int n_b, float r_b) {
    float thresh_sq = (r_a + r_b) * (r_a + r_b);
    for (int ia = 0; ia < n_a; ia++) {
        CVec2 ca = vadd(cen_a, vscale(h_a, offs_a[ia]));
        for (int ib = 0; ib < n_b; ib++) {
            if (vdist_sq(ca, vadd(cen_b, vscale(h_b, offs_b[ib]))) <= thresh_sq)
                return 1;
        }
    }
    return 0;
}

static int predict_collision(const CTrajSample *a, int na,
                             const CTrajSample *b, int nb,
                             const CCollGeom *ga, const CCollGeom *gb,
                             CCollisionPred *out) {
    int count = na < nb ? na : nb;
    if (count == 0) return 0;
    float coarse_sq = (ga->coarse_radius + gb->coarse_radius) *
                      (ga->coarse_radius + gb->coarse_radius);
    for (int i = 0; i < count; i++) {
        CVec2 pa = a[i].position, ha = a[i].heading;
        CVec2 pb = b[i].position, hb = b[i].heading;
        if (vdist_sq(pa, pb) > coarse_sq) continue;

        int collides = check_circle_groups(pa, ha, ga->body_offsets, ga->body_offset_count, ga->body_radius,
                                           pb, hb, gb->body_offsets, gb->body_offset_count, gb->body_radius);
        if (!collides && a[i].has_trailer)
            collides = check_circle_groups(a[i].trailer_position, a[i].trailer_heading,
                                           ga->trailer_offsets, ga->trailer_offset_count, ga->trailer_radius,
                                           pb, hb, gb->body_offsets, gb->body_offset_count, gb->body_radius);
        if (!collides && b[i].has_trailer)
            collides = check_circle_groups(pa, ha, ga->body_offsets, ga->body_offset_count, ga->body_radius,
                                           b[i].trailer_position, b[i].trailer_heading,
                                           gb->trailer_offsets, gb->trailer_offset_count, gb->trailer_radius);
        if (!collides && a[i].has_trailer && b[i].has_trailer)
            collides = check_circle_groups(a[i].trailer_position, a[i].trailer_heading,
                                           ga->trailer_offsets, ga->trailer_offset_count, ga->trailer_radius,
                                           b[i].trailer_position, b[i].trailer_heading,
                                           gb->trailer_offsets, gb->trailer_offset_count, gb->trailer_radius);
        if (!collides) continue;

        int pi = i > 0 ? i - 1 : 0;
        out->time = a[i].time;
        out->pos_a = pa; out->pos_b = pb;
        out->prev_pos_a = a[pi].position; out->prev_pos_b = b[pi].position;
        out->heading_a = ha; out->heading_b = hb;
        out->already_collided = (i == 0);
        out->priority_a = a[i].priority; out->priority_b = b[i].priority;
        out->spline_a_id = a[i].spline_id; out->spline_b_id = b[i].spline_id;
        return 1;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
   Blame
   ═══════════════════════════════════════════════════════════════════ */

static void blame_rear_car(const CCollisionPred *c, const CCar *a, const CCar *b,
                           int *blame_a, int *blame_b) {
    CVec2 ref = vnorm(vadd(c->heading_a, c->heading_b));
    if (vlen_sq(ref) <= 1e-6f) ref = vnorm(c->heading_a);
    CVec2 rel = vsub(c->prev_pos_a, c->prev_pos_b);
    float proj = vdot(rel, ref);
    if (absf(proj) < 1e-4f) proj = vdot(vsub(c->pos_a, c->pos_b), ref);
    if (absf(proj) < 1e-4f) {
        if (a->speed > b->speed) { *blame_a = 1; *blame_b = 0; }
        else if (b->speed > a->speed) { *blame_a = 0; *blame_b = 1; }
        else if (a->route_id <= b->route_id) { *blame_a = 1; *blame_b = 0; }
        else { *blame_a = 0; *blame_b = 1; }
        return;
    }
    if (proj < 0) { *blame_a = 1; *blame_b = 0; }
    else { *blame_a = 0; *blame_b = 1; }
}

static void blame_left_car(const CCollisionPred *c, const CCar *a, const CCar *b,
                           int *blame_a, int *blame_b) {
    CVec2 r_ba = vsub(c->prev_pos_b, c->prev_pos_a);
    CVec2 r_ab = vsub(c->prev_pos_a, c->prev_pos_b);
    if (vlen_sq(r_ba) <= 1e-6f || vlen_sq(r_ab) <= 1e-6f) {
        r_ba = vsub(c->pos_b, c->pos_a);
        r_ab = vsub(c->pos_a, c->pos_b);
    }
    float bearing_ba = signed_angle_deg(c->heading_a, r_ba);
    float bearing_ab = signed_angle_deg(c->heading_b, r_ab);
    const float eps = 1.0f;
    if (bearing_ba < -eps && bearing_ab > eps) { *blame_a = 0; *blame_b = 1; return; }
    if (bearing_ab < -eps && bearing_ba > eps) { *blame_a = 1; *blame_b = 0; return; }
    float ch = cross2d(c->heading_a, c->heading_b);
    if (ch < 0) { *blame_a = 1; *blame_b = 0; return; }
    if (ch > 0) { *blame_a = 0; *blame_b = 1; return; }
    if (a->route_id <= b->route_id) { *blame_a = 0; *blame_b = 1; }
    else { *blame_a = 1; *blame_b = 0; }
}

static int car_hitbox_touches_spline(const CCar *car, const CSpline *target, const CGraph *g) {
    const CSpline *cur = graph_spline_by_id(g, car->current_spline_id);
    if (!cur) return 0;
    CVec2 sp_pos, sp_tan;
    sample_spline_pos_tan(cur, car->distance_on_spline, &sp_pos, &sp_tan);
    CVec2 rn = cv2(sp_tan.y, -sp_tan.x);
    CVec2 fp = vadd(sp_pos, vscale(rn, car->lateral_offset));
    CVec2 diff = vsub(fp, car->rear_position);
    CVec2 bh = (vlen_sq(diff) > 1e-9f) ? vnorm(diff) : sp_tan;
    CVec2 center = vscale(vadd(fp, car->rear_position), 0.5f);
    float r = hitbox_radius(car->width);
    float r_sq = r * r;
    float offs[MAX_HITBOX_CIRCLES];
    int noffs = hitbox_circle_offsets(car->length, car->width, offs, MAX_HITBOX_CIRCLES);
    for (int k = 0; k < noffs; k++) {
        CVec2 circle = vadd(center, vscale(bh, offs[k]));
        CVec2 nearest = nearest_sample_on_spline(target, circle);
        if (vdist_sq(circle, nearest) <= r_sq) return 1;
    }
    return 0;
}

static int normal_car_occupies_priority_spline(const CCar *car, int prio_spline_id, const CGraph *g) {
    if (prio_spline_id < 0) return 0;
    const CSpline *ps = graph_spline_by_id(g, prio_spline_id);
    if (!ps || !ps->priority) return 0;
    return car_hitbox_touches_spline(car, ps, g);
}

static void determine_blame(const CCollisionPred *c, const CCar *a, const CCar *b,
                            const CGraph *g, int *blame_a, int *blame_b) {
    *blame_a = 0; *blame_b = 0;
    if (c->already_collided) {
        if (heading_angle_deg(c->heading_a, c->heading_b) >= 60)
            return;
        blame_rear_car(c, a, b, blame_a, blame_b);
        return;
    }
    if (c->priority_a != c->priority_b) {
        if (c->priority_a) {
            if (normal_car_occupies_priority_spline(b, c->spline_a_id, g))
                { *blame_a = 1; return; }
            *blame_b = 1; return;
        }
        if (normal_car_occupies_priority_spline(a, c->spline_b_id, g))
            { *blame_b = 1; return; }
        *blame_a = 1; return;
    }
    float angle = heading_angle_deg(c->heading_a, c->heading_b);
    if (angle < BLAME_ANGLE_THRESH)
        blame_rear_car(c, a, b, blame_a, blame_b);
    else
        blame_left_car(c, a, b, blame_a, blame_b);
}

/* ═══════════════════════════════════════════════════════════════════
   Closing rate scale & recently-left
   ═══════════════════════════════════════════════════════════════════ */

static inline float closing_rate_scale(CVec2 disp, float d_sq,
                                       CVec2 hi, CVec2 hj,
                                       float si, float sj) {
    if (d_sq < 1e-12f) return 1.0f;
    float mc = si + sj;
    if (mc < 1e-6f) return 1.0f;
    float num = vdot(vscale(hi, si), disp) - vdot(vscale(hj, sj), disp);
    float inv = 1.0f / (sqrtf(d_sq) * mc);
    float ratio = num * inv;
    if (ratio > 1) ratio = 1;
    else if (ratio < -1) ratio = -1;
    return 0.65f + 0.35f * ratio;
}

static inline int recently_left(const CCar *car, int spline_id) {
    return spline_id >= 0 && (car->prev_spline_ids[0] == spline_id || car->prev_spline_ids[1] == spline_id);
}

/* ═══════════════════════════════════════════════════════════════════
   Spatial grid (open-addressing hash map)
   ═══════════════════════════════════════════════════════════════════ */

static int sgrid_next_pow2(int v) {
    int out = 16;
    while (out < v && out < (1 << 30)) out <<= 1;
    return out;
}

static inline uint32_t sgrid_hash_for_mask(int32_t cx, int32_t cy, int mask) {
    uint32_t h = (uint32_t)cx * 73856093u ^ (uint32_t)cy * 19349663u;
    return h & (uint32_t)mask;
}

static void sgrid_init(CSpatialGrid *g, float cell_size, int initial_cap) {
    g->cell_size = cell_size;
    g->inv_cell_size = 1.0f / cell_size;
    g->used_count = 0;
    g->bucket_count = sgrid_next_pow2(initial_cap > 0 ? initial_cap : 16);
    g->bucket_mask = g->bucket_count - 1;
    g->buckets = (SGBucket *)calloc((size_t)g->bucket_count, sizeof(SGBucket));
}

static void sgrid_free(CSpatialGrid *g) {
    if (!g->buckets) return;
    for (int i = 0; i < g->bucket_count; i++)
        free(g->buckets[i].entries);
    free(g->buckets);
    g->buckets = NULL;
    g->bucket_count = 0;
    g->bucket_mask = 0;
    g->used_count = 0;
}

static const SGBucket *sgrid_find_bucket(const CSpatialGrid *g, int32_t cx, int32_t cy) {
    uint32_t h = sgrid_hash_for_mask(cx, cy, g->bucket_mask);
    for (;;) {
        const SGBucket *b = &g->buckets[h];
        if (!b->used) return NULL;
        if (b->cx == cx && b->cy == cy) return b;
        h = (h + 1) & (uint32_t)g->bucket_mask;
    }
}

static void sgrid_rehash(CSpatialGrid *g, int new_bucket_count) {
    SGBucket *old = g->buckets;
    int old_count = g->bucket_count;

    g->bucket_count = sgrid_next_pow2(new_bucket_count);
    g->bucket_mask = g->bucket_count - 1;
    g->buckets = (SGBucket *)calloc((size_t)g->bucket_count, sizeof(SGBucket));
    g->used_count = 0;

    for (int i = 0; i < old_count; i++) {
        SGBucket *src = &old[i];
        if (!src->used) continue;
        uint32_t h = sgrid_hash_for_mask(src->cx, src->cy, g->bucket_mask);
        for (;;) {
            SGBucket *dst = &g->buckets[h];
            if (!dst->used) {
                *dst = *src;
                dst->used = 1;
                g->used_count++;
                break;
            }
            h = (h + 1) & (uint32_t)g->bucket_mask;
        }
    }

    free(old);
}

static void sgrid_ensure_bucket_capacity(CSpatialGrid *g) {
    if (g->used_count * 4 < g->bucket_count * 3) return;
    sgrid_rehash(g, g->bucket_count * 2);
}

static void sgrid_append_entry(SGBucket *b, int index) {
    if (b->count == b->cap) {
        int new_cap = b->cap > 0 ? b->cap * 2 : 4;
        b->entries = (int *)realloc(b->entries, sizeof(int) * new_cap);
        b->cap = new_cap;
    }
    b->entries[b->count++] = index;
}

static SGBucket *sgrid_insert_cell(CSpatialGrid *g, int32_t cx, int32_t cy) {
    sgrid_ensure_bucket_capacity(g);
    uint32_t h = sgrid_hash_for_mask(cx, cy, g->bucket_mask);
    for (;;) {
        SGBucket *b = &g->buckets[h];
        if (!b->used) {
            b->cx = cx; b->cy = cy;
            b->used = 1;
            g->used_count++;
            return b;
        }
        if (b->cx == cx && b->cy == cy) return b;
        h = (h + 1) & (uint32_t)g->bucket_mask;
    }
}

static void sgrid_insert(CSpatialGrid *g, int index, CVec2 p, float radius) {
    if (radius <= g->cell_size) {
        int32_t cx = floori32(p.x * g->inv_cell_size);
        int32_t cy = floori32(p.y * g->inv_cell_size);
        sgrid_append_entry(sgrid_insert_cell(g, cx, cy), index);
        return;
    }
    int32_t min_cx = floori32((p.x - radius) * g->inv_cell_size);
    int32_t max_cx = floori32((p.x + radius) * g->inv_cell_size);
    int32_t min_cy = floori32((p.y - radius) * g->inv_cell_size);
    int32_t max_cy = floori32((p.y + radius) * g->inv_cell_size);
    for (int32_t cx = min_cx; cx <= max_cx; cx++)
        for (int32_t cy = min_cy; cy <= max_cy; cy++)
            sgrid_append_entry(sgrid_insert_cell(g, cx, cy), index);
}

static int sgrid_query(const CSpatialGrid *g, CVec2 p, int *buf, int buf_cap) {
    int32_t c0 = floori32(p.x * g->inv_cell_size);
    int32_t c1 = floori32(p.y * g->inv_cell_size);
    int n = 0;
    for (int32_t dx = -1; dx <= 1; dx++) {
        for (int32_t dy = -1; dy <= 1; dy++) {
            int32_t cx = c0 + dx, cy = c1 + dy;
            const SGBucket *b = sgrid_find_bucket(g, cx, cy);
            if (!b) continue;
            int avail = buf_cap - n;
            int copy = b->count < avail ? b->count : avail;
            memcpy(&buf[n], b->entries, sizeof(int) * copy);
            n += copy;
        }
    }
    return n;
}

static int compact_unique_indices(int *buf, int n) {
    int out = 0;
    for (int i = 0; i < n; i++) {
        int v = buf[i];
        int seen = 0;
        for (int j = 0; j < out; j++) {
            if (buf[j] == v) {
                seen = 1;
                break;
            }
        }
        if (!seen) buf[out++] = v;
    }
    return out;
}

/* ═══════════════════════════════════════════════════════════════════
   Median reach (for grid cell sizing)
   ═══════════════════════════════════════════════════════════════════ */

static int float_cmp(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static float median_reach(const float *reach, int n) {
    if (n == 0) return 1;
    float *tmp = (float *)malloc(sizeof(float) * n);
    memcpy(tmp, reach, sizeof(float) * n);
    qsort(tmp, n, sizeof(float), float_cmp);
    float m = tmp[n / 2];
    free(tmp);
    return m;
}

/* ═══════════════════════════════════════════════════════════════════
   Deadlock detection
   ═══════════════════════════════════════════════════════════════════ */

static void detect_deadlock_releases(int num_cars,
                                     const CBlameLink *brake_links, int n_brake,
                                     const CBlameLink *hold_links, int n_hold,
                                     const int *braking, const int *hold_speed,
                                     int *released) {
    if (num_cars <= 1) return;

    /* Build adjacency: count pass then fill */
    int *counts = (int *)calloc(num_cars, sizeof(int));
    for (int i = 0; i < n_brake; i++) {
        int f = brake_links[i].from;
        if (f < 0 || f >= num_cars || !braking[f]) continue;
        int t = brake_links[i].to;
        if (t < 0 || t >= num_cars) continue;
        counts[f]++;
    }
    for (int i = 0; i < n_hold; i++) {
        int f = hold_links[i].from;
        if (f < 0 || f >= num_cars || !hold_speed[f]) continue;
        int t = hold_links[i].to;
        if (t < 0 || t >= num_cars) continue;
        counts[f]++;
    }
    int *offsets = (int *)malloc(sizeof(int) * (num_cars + 1));
    offsets[0] = 0;
    for (int i = 0; i < num_cars; i++) offsets[i+1] = offsets[i] + counts[i];
    int total = offsets[num_cars];
    int *adj = (int *)malloc(sizeof(int) * (total + 1));
    int *pos = (int *)calloc(num_cars, sizeof(int));

    for (int i = 0; i < n_brake; i++) {
        int f = brake_links[i].from;
        if (f < 0 || f >= num_cars || !braking[f]) continue;
        int t = brake_links[i].to;
        if (t < 0 || t >= num_cars) continue;
        adj[offsets[f] + pos[f]++] = t;
    }
    for (int i = 0; i < n_hold; i++) {
        int f = hold_links[i].from;
        if (f < 0 || f >= num_cars || !hold_speed[f]) continue;
        int t = hold_links[i].to;
        if (t < 0 || t >= num_cars) continue;
        adj[offsets[f] + pos[f]++] = t;
    }

    /* DFS for short cycles (max length 4) */
    int path[DEADLOCK_MAX_PATH];
    for (int start = 0; start < num_cars; start++) {
        if (offsets[start+1] == offsets[start]) continue;
        path[0] = start;

        /* Iterative DFS with explicit stack */
        /* Stack frame: (path_len, neighbor_index) */
        int stack_pl[DEADLOCK_MAX_PATH * 16];
        int stack_ni[DEADLOCK_MAX_PATH * 16];
        int sp = 0;
        stack_pl[0] = 1;
        stack_ni[0] = offsets[start];
        while (sp >= 0) {
            int pl = stack_pl[sp];
            int *ni = &stack_ni[sp];
            int cur = path[pl - 1];
            int end_ni = offsets[cur + 1];
            if (*ni >= end_ni) { sp--; continue; }
            int next = adj[(*ni)++];
            if (next == start) {
                int min_idx = path[0];
                for (int k = 1; k < pl; k++)
                    if (path[k] < min_idx) min_idx = path[k];
                released[min_idx] = 1;
                continue;
            }
            if (pl >= DEADLOCK_MAX_PATH) continue;
            int repeated = 0;
            for (int k = 0; k < pl; k++)
                if (path[k] == next) { repeated = 1; break; }
            if (repeated) continue;
            path[pl] = next;
            sp++;
            stack_pl[sp] = pl + 1;
            stack_ni[sp] = offsets[next];
        }
    }

    free(counts); free(offsets); free(adj); free(pos);
}

/* ═══════════════════════════════════════════════════════════════════
   Brake probe: can a blamed car escape by accelerating?
   ═══════════════════════════════════════════════════════════════════ */

static int has_blamed_conflict(int ci, const CCar *test_car,
                               const CTrajSample *test_pred, int test_len,
                               CAABB test_aabb,
                               const CCar *cars, int num_cars,
                               const CGraph *g,
                               const CTrajSample *all_traj, const int *traj_lens,
                               const CCollGeom *geoms, const CCarPose *poses,
                               const float *reach, const CAABB *aabbs,
                               const CSpatialGrid *grid,
                               int *escape_checks, int *escape_hits) {
    int nbuf[256];
    int nn = sgrid_query(grid, poses[ci].pos, nbuf, 256);
    nn = compact_unique_indices(nbuf, nn);
    for (int ni = 0; ni < nn; ni++) {
        int oi = nbuf[ni];
        if (oi == ci || oi < 0 || oi >= num_cars || traj_lens[oi] == 0) continue;
        CVec2 disp = vsub(poses[oi].pos, poses[ci].pos);
        float d_sq = vlen_sq(disp);
        float scale = closing_rate_scale(disp, d_sq, poses[ci].heading, poses[oi].heading,
                                         cars[ci].speed, cars[oi].speed);
        float bpd = (reach[ci] + reach[oi]) * scale;
        if (d_sq > bpd * bpd) continue;
        if (!aabb_overlap(test_aabb, aabbs[oi])) continue;
        if (recently_left(test_car, cars[oi].current_spline_id)) continue;

        (*escape_checks)++;
        const CTrajSample *other = &all_traj[oi * MAX_TRAJ_SAMPLES];
        CCollisionPred coll;
        if (!predict_collision(test_pred, test_len, other, traj_lens[oi],
                               &geoms[ci], &geoms[oi], &coll))
            continue;
        (*escape_hits)++;
        int ba, bb;
        determine_blame(&coll, test_car, &cars[oi], g, &ba, &bb);
        if (ba) return 1;
    }
    return 0;
}

static int should_brake_for_blamed(int ci, const CCar *cars, int num_cars,
                                   const CGraph *g, const CRouteTree *trees,
                                   const CTrajSample *all_traj, const int *traj_lens,
                                   const CCollGeom *geoms, const CCarPose *poses,
                                   const float *reach, const CAABB *aabbs,
                                   const CSpatialGrid *grid,
                                   int *escape_preds, int *escape_samples,
                                   int *escape_checks, int *escape_hits) {
    const CCar *car = &cars[ci];
    const CSpline *cur_sp = graph_spline_by_id(g, car->current_spline_id);
    if (!cur_sp) return 1;
    float target_speed = car->max_speed * cur_sp->speed_factor;
    if (car->speed >= target_speed - ACCEL_ESC_LOOK_S * car->accel)
        return 1;

    float test_secs[2] = { ACCEL_ESC_SHORT_S, ACCEL_ESC_MED_S };
    CTrajSample test_traj[MAX_TRAJ_SAMPLES];
    for (int ti = 0; ti < 2; ti++) {
        CCar test_car = *car;
        test_car.speed = minf(target_speed, car->speed + car->accel * test_secs[ti]);
        if (test_car.speed <= car->speed + 1e-4f) return 1;

        const CRouteTree *tree = &trees[car->route_tree_index];
        int tlen = predict_car_trajectory(&test_car, g, tree,
                                          PREDICTION_HORIZON_S, PREDICTION_STEP_S, test_traj);
        (*escape_preds)++;
        *escape_samples += tlen;
        if (tlen == 0) return 1;

        CAABB taabb = build_traj_aabb(test_traj, tlen, geoms[ci].coarse_radius);
        if (has_blamed_conflict(ci, &test_car, test_traj, tlen, taabb,
                                cars, num_cars, g, all_traj, traj_lens,
                                geoms, poses, reach, aabbs, grid,
                                escape_checks, escape_hits))
            return 1;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
   Shared stationary prediction (atomic compute-once)
   ═══════════════════════════════════════════════════════════════════ */

/* Atomically compute stationary prediction for car k.
   Returns 1 if this call actually computed it (for profiling). */
static int ensure_stationary_pred(int k, const CCar *cars, const CGraph *g,
                                  const CRouteTree *trees,
                                  CTrajSample *stat_traj, int *stat_lens,
                                  int *stat_lock) {
    /* Try to claim: 0 -> 1 */
    int expected = 0;
    if (__atomic_compare_exchange_n(&stat_lock[k], &expected, 1, 0,
                                    __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        /* We won — compute it */
        CCar sc = cars[k]; sc.speed = 0;
        const CRouteTree *tree = &trees[sc.route_tree_index];
        int len = predict_car_trajectory(&sc, g, tree,
                                         PREDICTION_HORIZON_S, PREDICTION_STEP_S,
                                         &stat_traj[k * MAX_TRAJ_SAMPLES]);
        stat_lens[k] = len;
        __atomic_store_n(&stat_lock[k], 2, __ATOMIC_RELEASE);
        return 1;
    }
    /* Another thread is computing or has computed — spin until done */
    while (__atomic_load_n(&stat_lock[k], __ATOMIC_ACQUIRE) != 2)
        ;
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
   Hold probe: should a non-braking car hold its speed?
   ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    int should_hold;
    CBlameLink hold_link;
    int has_hold_link;
    int faster_preds, stationary_preds, total_pred_samples;
    int hold_checks, hold_hits, stat_checks, stat_hits;
} HoldResult;

static HoldResult compute_hold_for_car(int ci, const CCar *cars, int num_cars,
                                       const CGraph *g, const CRouteTree *trees,
                                       const int *flags,
                                       const CTrajSample *all_traj, const int *traj_lens,
                                       CTrajSample *stat_traj, int *stat_lens, int *stat_lock,
                                       const CCollGeom *geoms, const CCarPose *poses,
                                       const float *reach, const CAABB *aabbs,
                                       const CSpatialGrid *grid,
                                       int debug_car, int debug_mode,
                                       CBlameLink *cand_buf, int *cand_count, int cand_cap) {
    HoldResult r;
    memset(&r, 0, sizeof(r));
    if (ci < 0 || ci >= num_cars || traj_lens[ci] == 0 || flags[ci]) return r;

    const CCar *car = &cars[ci];
    CCar faster = *car;
    faster.speed += 0.25f * faster.accel;
    if (faster.speed <= car->speed + 1e-4f) return r;

    /* Pre-filter: does the car's existing AABB overlap any neighbor? */
    int nbuf[256];
    int nn = sgrid_query(grid, poses[ci].pos, nbuf, 256);
    nn = compact_unique_indices(nbuf, nn);
    int has_nearby = 0;
    for (int ni = 0; ni < nn; ni++) {
        int j = nbuf[ni];
        if (j == ci || j < 0 || j >= num_cars || traj_lens[j] == 0) continue;
        CVec2 disp = vsub(poses[j].pos, poses[ci].pos);
        float d_sq = vlen_sq(disp);
        float scale = closing_rate_scale(disp, d_sq, poses[ci].heading, poses[j].heading,
                                         cars[ci].speed, cars[j].speed);
        float bpd = (reach[ci] + reach[j]) * scale * scale;
        if (d_sq > bpd * bpd) continue;
        if (!aabb_overlap(aabbs[ci], aabbs[j])) continue;
        has_nearby = 1;
        break;
    }
    if (!has_nearby) return r;

    const CRouteTree *tree = &trees[car->route_tree_index];
    CTrajSample faster_traj[MAX_TRAJ_SAMPLES];
    int flen = predict_car_trajectory(&faster, g, tree,
                                      PREDICTION_HORIZON_S, PREDICTION_STEP_S, faster_traj);
    r.faster_preds = 1;
    r.total_pred_samples += flen;
    if (flen == 0) return r;

    CAABB faster_aabb = build_traj_aabb(faster_traj, flen, geoms[ci].coarse_radius);

    for (int ni = 0; ni < nn; ni++) {
        int j = nbuf[ni];
        if (j == ci || j < 0 || j >= num_cars || traj_lens[j] == 0) continue;
        CVec2 disp = vsub(poses[j].pos, poses[ci].pos);
        float d_sq = vlen_sq(disp);
        float scale = closing_rate_scale(disp, d_sq, poses[ci].heading, poses[j].heading,
                                         cars[ci].speed, cars[j].speed);
        float bpd = (reach[ci] + reach[j]) * scale * scale;
        if (d_sq > bpd * bpd) continue;
        if (!aabb_overlap(faster_aabb, aabbs[j])) continue;

        if (debug_car >= 0 && debug_mode == 1 && ci == debug_car) {
            if (*cand_count < cand_cap)
                cand_buf[(*cand_count)++] = (CBlameLink){ci, j};
        }

        r.hold_checks++;
        const CTrajSample *other = &all_traj[j * MAX_TRAJ_SAMPLES];
        CCollisionPred coll;
        if (!predict_collision(faster_traj, flen, other, traj_lens[j],
                               &geoms[ci], &geoms[j], &coll))
            continue;
        r.hold_hits++;
        int ba, bb_;
        determine_blame(&coll, &faster, &cars[j], g, &ba, &bb_);
        if (!ba || recently_left(&faster, cars[j].current_spline_id)) continue;

        if (coll.already_collided) {
            r.should_hold = 1;
            r.hold_link = (CBlameLink){ci, j};
            r.has_hold_link = 1;
            break;
        }

        /* Lazy compute stationary prediction */
        if (ensure_stationary_pred(ci, cars, g, trees, stat_traj, stat_lens, stat_lock)) {
            r.stationary_preds++;
            r.total_pred_samples += stat_lens[ci];
        }
        r.stat_checks++;
        CCollisionPred dummy;
        if (!predict_collision(&stat_traj[ci * MAX_TRAJ_SAMPLES], stat_lens[ci],
                               other, traj_lens[j], &geoms[ci], &geoms[j], &dummy)) {
            r.should_hold = 1;
            r.hold_link = (CBlameLink){ci, j};
            r.has_hold_link = 1;
            break;
        }
        r.stat_hits++;
    }
    return r;
}

/* ═══════════════════════════════════════════════════════════════════
   Main pipeline
   ═══════════════════════════════════════════════════════════════════ */

static double now_ms(void) {
    return omp_get_wtime() * 1000.0;
}

void compute_braking_decisions(CBrakingCtx *ctx) {
    int N = ctx->num_cars;
    CBrakingProfile *prof = &ctx->profile;
    memset(prof, 0, sizeof(*prof));
    prof->cars = N;
    double t_all = now_ms();
    ctx->debug_links_count = 0;
    ctx->hold_links_count = 0;
    ctx->candidate_links_count = 0;
    memset(ctx->brake_flags, 0, sizeof(int) * N);
    memset(ctx->hold_flags, 0, sizeof(int) * N);

    if (N < 2) return;

    CGraph *g = &ctx->graph;
    int tree_count = ctx->num_route_trees > 0 ? ctx->num_route_trees : 1;
    double t_route = now_ms();
    CRouteTree *trees = (CRouteTree *)malloc(sizeof(CRouteTree) * tree_count);
    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < tree_count; i++) {
        int dest_id = -1;
        int vehicle_kind = VEHICLE_CAR;
        if (i < ctx->num_route_trees) {
            dest_id = ctx->route_tree_dest_ids[i];
            vehicle_kind = ctx->route_tree_vehicle_kinds[i];
        }
        trees[i] = build_route_tree(g, dest_id, vehicle_kind);
    }
    prof->route_tree_ms = now_ms() - t_route;

    /* ── Base predictions ─────────────────────────────────── */
    double t0 = now_ms();
    CTrajSample *all_traj = (CTrajSample *)malloc(sizeof(CTrajSample) * N * MAX_TRAJ_SAMPLES);
    int *traj_lens = (int *)malloc(sizeof(int) * N);
    CCollGeom *geoms = (CCollGeom *)malloc(sizeof(CCollGeom) * N);
    CCarPose *poses = (CCarPose *)malloc(sizeof(CCarPose) * N);
    float *reach = (float *)malloc(sizeof(float) * N);

    #pragma omp parallel for schedule(dynamic, 8)
    for (int i = 0; i < N; i++) {
        const CCar *car = &ctx->cars[i];
        /* pose */
        const CSpline *sp = graph_spline_by_id(g, car->current_spline_id);
        if (sp) {
            CVec2 pos, tan; float curv;
            sample_spline_state(sp, car->distance_on_spline, NULL, NULL, &pos, &tan, &curv);
            poses[i].pos = pos;
            poses[i].heading = tan;
        } else {
            poses[i].pos = cv2(0,0);
            poses[i].heading = cv2(1,0);
        }
        /* reach */
        float phys = maxf(car->length, car->width);
        if (car->has_trailer) phys = car->length + car->trailer_length;
        float spd = car->speed > 0 ? car->speed : 0;
        reach[i] = spd * PREDICTION_HORIZON_S +
                   0.5f * car->accel * PREDICTION_HORIZON_S * PREDICTION_HORIZON_S +
                   phys + BROAD_PHASE_SLACK_M;
        /* geometry */
        geoms[i] = build_collision_geometry(car);
        /* trajectory */
        const CRouteTree *tree = &trees[car->route_tree_index];
        traj_lens[i] = predict_car_trajectory(car, g, tree,
                                              PREDICTION_HORIZON_S, PREDICTION_STEP_S,
                                              &all_traj[i * MAX_TRAJ_SAMPLES]);
        #pragma omp atomic
        prof->total_prediction_samples += traj_lens[i];
    }
    prof->base_predictions = N;
    prof->total_predictions = N;
    prof->base_predict_ms = now_ms() - t0;

    /* ── Trajectory AABBs / grid setup ─────────────────────── */
    double t_setup = now_ms();
    CAABB *aabbs = (CAABB *)malloc(sizeof(CAABB) * N);
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < N; i++)
        aabbs[i] = build_traj_aabb(&all_traj[i * MAX_TRAJ_SAMPLES], traj_lens[i], geoms[i].coarse_radius);

    /* ── Spatial grid ──────────────────────────────────────── */
    float cell_size = median_reach(reach, N);
    if (cell_size < 1) cell_size = 1;
    CSpatialGrid grid;
    sgrid_init(&grid, cell_size, N * 4);
    for (int i = 0; i < N; i++) {
        if (traj_lens[i] > 0)
            sgrid_insert(&grid, i, poses[i].pos, reach[i]);
    }

    /* ── Stationary predictions (shared, lazily filled) ────── */
    CTrajSample *stat_traj = (CTrajSample *)malloc(sizeof(CTrajSample) * N * MAX_TRAJ_SAMPLES);
    int *stat_lens = (int *)calloc(N, sizeof(int));
    /* Per-car lock: 0=free, 1=computing, 2=done */
    int *stat_lock = (int *)calloc(N, sizeof(int));
    prof->marshal_setup_ms = now_ms() - t_setup;

    /* ── Conflict scan ─────────────────────────────────────── */
    double t1 = now_ms();
    int *initial_blame = (int *)calloc(N, sizeof(int));

    /* Per-thread accumulators for blame links and profile counters */
    int max_threads = omp_get_max_threads();

    /* Per-thread link buffers */
    typedef struct {
        CBlameLink *tentative; int tent_count, tent_cap;
        CBlameLink *candidate; int cand_count, cand_cap;
        int pair_candidates, broad_pairs, coll_checks, coll_hits;
        int stat_preds, stat_coll_checks, stat_coll_hits;
        int total_preds, total_pred_samples;
    } ThreadScan;
    ThreadScan *tscan = (ThreadScan *)calloc(max_threads, sizeof(ThreadScan));
    for (int t = 0; t < max_threads; t++) {
        tscan[t].tent_cap = N;
        tscan[t].tentative = (CBlameLink *)malloc(sizeof(CBlameLink) * N);
        tscan[t].cand_cap = N;
        tscan[t].candidate = (CBlameLink *)malloc(sizeof(CBlameLink) * N);
    }

    int chunk = (N + max_threads - 1) / max_threads;
    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        ThreadScan *ts = &tscan[tid];
        int istart = tid * chunk, iend = istart + chunk;
        if (iend > N) iend = N;

        int *seen = (int *)calloc(N, sizeof(int));
        int *seen_reset = (int *)malloc(sizeof(int) * N);
        int seen_reset_count = 0;
        int nbuf[256];

        for (int i = istart; i < iend; i++) {
            if (traj_lens[i] == 0) continue;
            /* Reset seen flags */
            for (int k = 0; k < seen_reset_count; k++) seen[seen_reset[k]] = 0;
            seen_reset_count = 0;

            int nn = sgrid_query(&grid, poses[i].pos, nbuf, 256);
            for (int ni = 0; ni < nn; ni++) {
                int j = nbuf[ni];
                if (j <= i) continue;
                if (seen[j]) continue;
                seen[j] = 1;
                seen_reset[seen_reset_count++] = j;
                ts->pair_candidates++;

                CVec2 disp = vsub(poses[j].pos, poses[i].pos);
                float d_sq = vlen_sq(disp);
                float scale = closing_rate_scale(disp, d_sq, poses[i].heading, poses[j].heading,
                                                 ctx->cars[i].speed, ctx->cars[j].speed);
                float bpd = (reach[i] + reach[j]) * scale;
                if (d_sq > bpd * bpd) continue;
                ts->broad_pairs++;
                if (!aabb_overlap(aabbs[i], aabbs[j])) continue;

                if (ctx->debug_selected_car >= 0 && ctx->debug_selected_car_mode == 0 &&
                    (i == ctx->debug_selected_car || j == ctx->debug_selected_car)) {
                    if (ts->cand_count < ts->cand_cap)
                        ts->candidate[ts->cand_count++] = (CBlameLink){i, j};
                }

                ts->coll_checks++;
                const CTrajSample *ti_s = &all_traj[i * MAX_TRAJ_SAMPLES];
                const CTrajSample *tj_s = &all_traj[j * MAX_TRAJ_SAMPLES];
                CCollisionPred coll;
                if (!predict_collision(ti_s, traj_lens[i], tj_s, traj_lens[j],
                                       &geoms[i], &geoms[j], &coll))
                    continue;
                ts->coll_hits++;
                int bi, bj;
                determine_blame(&coll, &ctx->cars[i], &ctx->cars[j], g, &bi, &bj);
                int ac = coll.already_collided;
                if (bi && recently_left(&ctx->cars[i], ctx->cars[j].current_spline_id)) bi = 0;
                if (bj && recently_left(&ctx->cars[j], ctx->cars[i].current_spline_id)) bj = 0;

                /* Stationary check for blame_i */
                if (bi && !ac) {
                    if (ensure_stationary_pred(i, ctx->cars, g, trees, stat_traj, stat_lens, stat_lock)) {
                        ts->stat_preds++;
                        ts->total_preds++;
                        ts->total_pred_samples += stat_lens[i];
                    }
                    ts->stat_coll_checks++;
                    CCollisionPred dummy;
                    if (predict_collision(&stat_traj[i * MAX_TRAJ_SAMPLES], stat_lens[i],
                                          tj_s, traj_lens[j], &geoms[i], &geoms[j], &dummy)) {
                        ts->stat_coll_hits++;
                        bi = 0;
                    }
                }
                /* Stationary check for blame_j */
                if (bj && !ac) {
                    if (ensure_stationary_pred(j, ctx->cars, g, trees, stat_traj, stat_lens, stat_lock)) {
                        ts->stat_preds++;
                        ts->total_preds++;
                        ts->total_pred_samples += stat_lens[j];
                    }
                    ts->stat_coll_checks++;
                    CCollisionPred dummy;
                    if (predict_collision(&stat_traj[j * MAX_TRAJ_SAMPLES], stat_lens[j],
                                          ti_s, traj_lens[i], &geoms[j], &geoms[i], &dummy)) {
                        ts->stat_coll_hits++;
                        bj = 0;
                    }
                }

                if (bi) {
                    __atomic_store_n(&initial_blame[i], 1, __ATOMIC_RELAXED);
                    if (ts->tent_count < ts->tent_cap)
                        ts->tentative[ts->tent_count++] = (CBlameLink){i, j};
                }
                if (bj) {
                    __atomic_store_n(&initial_blame[j], 1, __ATOMIC_RELAXED);
                    if (ts->tent_count < ts->tent_cap)
                        ts->tentative[ts->tent_count++] = (CBlameLink){j, i};
                }
            }
        }
        free(seen); free(seen_reset);
    }

    /* Merge per-thread scan results */
    int tent_total = 0, cand_total = 0;
    for (int t = 0; t < max_threads; t++) {
        prof->primary_pair_candidates   += tscan[t].pair_candidates;
        prof->primary_broad_phase_pairs += tscan[t].broad_pairs;
        prof->primary_collision_checks  += tscan[t].coll_checks;
        prof->primary_collision_hits    += tscan[t].coll_hits;
        prof->stationary_predictions    += tscan[t].stat_preds;
        prof->stationary_collision_checks += tscan[t].stat_coll_checks;
        prof->stationary_collision_hits += tscan[t].stat_coll_hits;
        prof->total_predictions         += tscan[t].total_preds;
        prof->total_prediction_samples  += tscan[t].total_pred_samples;
        tent_total += tscan[t].tent_count;
        cand_total += tscan[t].cand_count;
    }
    for (int i = 0; i < N; i++)
        if (initial_blame[i]) prof->initially_blamed_cars++;

    /* Collect tentative links */
    CBlameLink *tent_links = (CBlameLink *)malloc(sizeof(CBlameLink) * (tent_total + 1));
    int tent_off = 0;
    for (int t = 0; t < max_threads; t++) {
        memcpy(&tent_links[tent_off], tscan[t].tentative, sizeof(CBlameLink) * tscan[t].tent_count);
        tent_off += tscan[t].tent_count;
    }
    /* Collect candidate links into ctx output */
    for (int t = 0; t < max_threads; t++) {
        for (int k = 0; k < tscan[t].cand_count; k++) {
            if (ctx->candidate_links_count < ctx->candidate_links_cap)
                ctx->candidate_links[ctx->candidate_links_count++] = tscan[t].candidate[k];
        }
    }
    for (int t = 0; t < max_threads; t++) {
        free(tscan[t].tentative);
        free(tscan[t].candidate);
    }
    free(tscan);
    prof->conflict_scan_ms = now_ms() - t1;

    /* ── Brake probe ───────────────────────────────────────── */
    double t2 = now_ms();
    #pragma omp parallel for schedule(dynamic, 4)
    for (int i = 0; i < N; i++) {
        if (!initial_blame[i]) continue;
        int ep = 0, es = 0, ec = 0, eh = 0;
        if (should_brake_for_blamed(i, ctx->cars, N, g, trees,
                                    all_traj, traj_lens, geoms, poses,
                                    reach, aabbs, &grid,
                                    &ep, &es, &ec, &eh)) {
            ctx->brake_flags[i] = 1;
        }
        #pragma omp atomic
        prof->escape_predictions += ep;
        #pragma omp atomic
        prof->total_predictions += ep;
        #pragma omp atomic
        prof->total_prediction_samples += es;
        #pragma omp atomic
        prof->escape_collision_checks += ec;
        #pragma omp atomic
        prof->escape_collision_hits += eh;
    }
    prof->brake_probe_ms = now_ms() - t2;

    /* ── Hold probe ────────────────────────────────────────── */
    double t3 = now_ms();
    /* Per-thread candidate link buffers for hold probe */
    typedef struct {
        CBlameLink *candidate_links; int candidate_count, candidate_cap;
        CBlameLink *hold_links; int hold_count, hold_cap;
        int faster_p, stat_p, total_ps, hc, hh, sc, sh;
    } ThreadHold;
    int max_t2 = omp_get_max_threads();
    ThreadHold *thold = (ThreadHold *)calloc(max_t2, sizeof(ThreadHold));
    for (int t = 0; t < max_t2; t++) {
        thold[t].candidate_cap = N > 64 ? N : 64;
        thold[t].candidate_links = (CBlameLink *)malloc(sizeof(CBlameLink) * thold[t].candidate_cap);
        thold[t].hold_cap = N > 64 ? N : 64;
        thold[t].hold_links = (CBlameLink *)malloc(sizeof(CBlameLink) * thold[t].hold_cap);
    }

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        ThreadHold *th = &thold[tid];
        #pragma omp for schedule(dynamic, 4)
        for (int i = 0; i < N; i++) {
            HoldResult hr = compute_hold_for_car(i, ctx->cars, N, g, trees,
                                                  ctx->brake_flags,
                                                  all_traj, traj_lens,
                                                  stat_traj, stat_lens, stat_lock,
                                                  geoms, poses, reach, aabbs, &grid,
                                                  ctx->debug_selected_car, ctx->debug_selected_car_mode,
                                                  th->candidate_links, &th->candidate_count, th->candidate_cap);
            if (hr.should_hold) ctx->hold_flags[i] = 1;
            if (hr.has_hold_link) {
                if (th->hold_count < th->hold_cap)
                    th->hold_links[th->hold_count++] = hr.hold_link;
            }
            th->faster_p  += hr.faster_preds;
            th->stat_p    += hr.stationary_preds;
            th->total_ps  += hr.total_pred_samples;
            th->hc        += hr.hold_checks;
            th->hh        += hr.hold_hits;
            th->sc        += hr.stat_checks;
            th->sh        += hr.stat_hits;
        }
    }
    for (int t = 0; t < max_t2; t++) {
        prof->faster_predictions        += thold[t].faster_p;
        prof->stationary_predictions    += thold[t].stat_p;
        prof->total_predictions         += thold[t].faster_p + thold[t].stat_p;
        prof->total_prediction_samples  += thold[t].total_ps;
        prof->hold_collision_checks     += thold[t].hc;
        prof->hold_collision_hits       += thold[t].hh;
        prof->stationary_collision_checks += thold[t].sc;
        prof->stationary_collision_hits += thold[t].sh;
        for (int k = 0; k < thold[t].candidate_count; k++) {
            if (ctx->candidate_links_count < ctx->candidate_links_cap)
                ctx->candidate_links[ctx->candidate_links_count++] = thold[t].candidate_links[k];
        }
        for (int k = 0; k < thold[t].hold_count; k++) {
            if (ctx->hold_links_count < ctx->hold_links_cap)
                ctx->hold_links[ctx->hold_links_count++] = thold[t].hold_links[k];
        }
        free(thold[t].candidate_links);
        free(thold[t].hold_links);
    }
    free(thold);
    prof->hold_probe_ms = now_ms() - t3;

    /* ── Deadlock release ──────────────────────────────────── */
    double t4 = now_ms();
    int *released = (int *)calloc(N, sizeof(int));
    detect_deadlock_releases(N, tent_links, tent_off,
                             ctx->hold_links, ctx->hold_links_count,
                             ctx->brake_flags, ctx->hold_flags, released);
    for (int i = 0; i < N; i++) {
        if (released[i]) {
            ctx->brake_flags[i] = 0;
            ctx->hold_flags[i] = 0;
        }
    }

    /* Filter debug links to only active ones */
    for (int i = 0; i < tent_off; i++) {
        int f = tent_links[i].from;
        if (f >= 0 && f < N && ctx->brake_flags[f]) {
            if (ctx->debug_links_count < ctx->debug_links_cap)
                ctx->debug_links[ctx->debug_links_count++] = tent_links[i];
        }
    }
    /* Filter hold links to only active ones */
    int active_hold = 0;
    for (int i = 0; i < ctx->hold_links_count; i++) {
        int f = ctx->hold_links[i].from;
        if (f >= 0 && f < N && ctx->hold_flags[f])
            ctx->hold_links[active_hold++] = ctx->hold_links[i];
    }
    ctx->hold_links_count = active_hold;

    for (int i = 0; i < N; i++) {
        if (ctx->brake_flags[i]) prof->braking_cars++;
        if (ctx->hold_flags[i]) prof->hold_cars++;
    }
    prof->finalize_ms = now_ms() - t4;

    /* ── Cleanup ───────────────────────────────────────────── */
    free(released);
    free(tent_links);
    free(initial_blame);
    free(stat_traj);
    free(stat_lens);
    free(stat_lock);
    sgrid_free(&grid);
    free(aabbs);
    free(reach);
    free(poses);
    free(geoms);
    free(traj_lens);
    free(all_traj);
    for (int i = 0; i < tree_count; i++) {
        free(trees[i].costs);
        free(trees[i].next_hop);
    }
    free(trees);
    prof->total_ms = now_ms() - t_all;
}
