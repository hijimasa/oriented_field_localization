// Branch-and-bound / correlative scan matching baseline for the global
// localization comparison (Olson, ICRA 2009 "Real-Time Correlative Scan
// Matching"; Hess et al., ICRA 2016 "Real-Time Loop Closure in 2D LIDAR SLAM"
// §IV.C branch-and-bound).
//
// Reads the *same* dumped disturbance scans as the OFL evaluator, searches the whole map over the
// full 360 deg, and writes cond,trial,gt,est,score,time to stdout.
//
// build:
//   g++ -O3 -fopenmp -std=c++17 bbs_eval.cpp $(pkg-config --cflags --libs opencv4) -o bbs_eval
// run:
//   ./bbs_eval scans_dump.csv ../maps/intel_research_lab/intel_research_lab.pgm
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// ---------------- parameters ----------------
static double g_res = 0.05;        // search grid resolution [m/cell]
static double g_sigma = 0.10;      // likelihood-field sigma [m]
static double g_trunc = 0.50;      // likelihood-field truncation [m]
static double g_max_range = 10.0;  // sensor max range [m]
static int g_levels = 7;           // branch-and-bound depth (2^7 = 128 cells)
static double g_angle_step_deg = 0;  // 0 => Cartographer's rule from max range
// Same close-range gate the OFL evaluator uses (min_range): returns
// closer than this are dropped, so a wall pressed against the sensor cannot
// dominate the score.  0 disables it.
static double g_min_range = 0.0;
// Second-peak search: after the global optimum, re-run with a suppression zone
// around it (same rule as gl_node::nmsCandidates: within 2 m AND within 15 deg)
// so that top1/top2 measures "is there a competing hypothesis elsewhere".
static bool g_second_peak = false;
static double g_nms_m = 2.0;
static double g_nms_deg = 15.0;
// Top-K output for the union-pool experiment: after the global optimum, re-run
// K-1 times, each time suppressing all peaks found so far (same NMS rule), and
// append "cond,trial,k,x,y,yaw,score" rows to BBS_TOPK_OUT.  BBS_TOPK=0 disables.
static int g_topk = 0;
static FILE* g_topk_out = nullptr;

static const int kNumBeams = 360;

// ---------------- multi-resolution precomputation grids ----------------
// level h holds, for every cell, the max of the level-0 score over the
// [x, x+2^h) x [y, y+2^h) box.  That is the upper bound the branch-and-bound
// needs for a translation window of 2^h cells.
struct PrecomputedGrids {
    int w = 0, h = 0;
    std::vector<std::vector<float>> level;  // level[k][y*w + x]

    inline float at(int k, int x, int y) const {
        if (x < 0 || y < 0 || x >= w || y >= h) return 0.0f;
        return level[k][(size_t)y * w + x];
    }
};

// Sliding-window maximum along one axis, window size `win`.
static void maxFilter1D(const std::vector<float>& src, std::vector<float>& dst,
                        int w, int h, int win, bool along_x) {
    dst.assign((size_t)w * h, 0.0f);
    int outer = along_x ? h : w;
    int inner = along_x ? w : h;
    std::vector<int> dq(inner);
    for (int o = 0; o < outer; o++) {
        int head = 0, tail = 0;  // monotonic deque of indices
        auto get = [&](int i) {
            return along_x ? src[(size_t)o * w + i] : src[(size_t)i * w + o];
        };
        auto put = [&](int i, float v) {
            if (along_x) dst[(size_t)o * w + i] = v;
            else dst[(size_t)i * w + o] = v;
        };
        for (int i = 0; i < inner; i++) {
            while (tail > head && get(dq[tail - 1]) <= get(i)) tail--;
            dq[tail++] = i;
            // window for output index (i - win + 1) is [i-win+1, i]
            int out = i - win + 1;
            if (out >= 0) {
                while (dq[head] < out) head++;
                put(out, get(dq[head]));
            }
        }
        // tail of the image: windows that run past the border
        for (int out = std::max(0, inner - win + 1); out < inner; out++) {
            while (head < tail && dq[head] < out) head++;
            put(out, head < tail ? get(dq[head]) : 0.0f);
        }
    }
}

static PrecomputedGrids buildGrids(const std::vector<float>& base, int w, int h,
                                   int levels) {
    PrecomputedGrids g;
    g.w = w; g.h = h;
    g.level.resize(levels + 1);
    g.level[0] = base;
    for (int k = 1; k <= levels; k++) {
        int win = 1 << k;
        std::vector<float> tmp;
        maxFilter1D(g.level[0], tmp, w, h, win, true);
        maxFilter1D(tmp, g.level[k], w, h, win, false);
    }
    return g;
}

// ---------------- branch and bound ----------------
struct Candidate {
    int angle_idx;
    int x, y;      // cell offset of the window's lower-left corner
    int level;
    float score;
};

struct RotatedScan {
    std::vector<int> px, py;   // point offsets in cells, relative to sensor cell
};

static float scoreCandidate(const PrecomputedGrids& g, const RotatedScan& rs,
                            int level, int ox, int oy) {
    float s = 0.0f;
    const size_t n = rs.px.size();
    for (size_t i = 0; i < n; i++)
        s += g.at(level, ox + rs.px[i], oy + rs.py[i]);
    return s;
}

struct BBSResult {
    double x, y, yaw;
    double score;
    long nodes;
};

// A region of pose space to exclude from the search, so a second run finds the
// best *competing* hypothesis instead of a neighbour of the first peak.  Mirrors
// gl_node::nmsCandidates: a pose is a duplicate when it is within `dist_cells`
// AND within `yaw_deg` of the kept peak.
struct Suppression {
    bool active = false;
    int cx = 0, cy = 0;          // first peak, in cells
    double yaw_deg = 0;          // first peak heading
    double dist_cells = 0;
    double sep_deg = 15.0;

    inline bool sameYaw(double a_deg) const {
        double d = std::fabs(a_deg - yaw_deg);
        if (d > 180.0) d = 360.0 - d;
        return d < sep_deg;
    }
    // leaf test
    inline bool suppressesLeaf(int x, int y, double a_deg) const {
        if (!active || !sameYaw(a_deg)) return false;
        double dx = x - cx, dy = y - cy;
        return dx * dx + dy * dy < dist_cells * dist_cells;
    }
    // whole-window test: true only if every cell of the window is suppressed,
    // which makes skipping the node safe.
    inline bool suppressesWindow(int x, int y, int span, double a_deg) const {
        if (!active || !sameYaw(a_deg)) return false;
        double far = 0.0;
        for (int iy = 0; iy < 2; iy++)
            for (int ix = 0; ix < 2; ix++) {
                double dx = (x + ix * (span - 1)) - cx;
                double dy = (y + iy * (span - 1)) - cy;
                far = std::max(far, dx * dx + dy * dy);
            }
        return far < dist_cells * dist_cells;
    }
};

// Scores are non-negative, so the IEEE-754 bit pattern of a float orders the
// same way the float does -- which lets the shared incumbent live in a plain
// atomic<uint32_t> and be raised with a CAS loop.
static inline uint32_t f2u(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
static inline float u2f(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

// Multiple exclusion zones: a leaf is skipped if ANY zone suppresses it; a
// window may be skipped only when a single zone covers it entirely (sufficient
// condition -- union coverage by several zones is not checked, which is safe).
struct MultiSuppression {
    std::vector<Suppression> zones;
    inline bool suppressesLeaf(int x, int y, double a_deg) const {
        for (const auto& z : zones)
            if (z.suppressesLeaf(x, y, a_deg)) return true;
        return false;
    }
    inline bool suppressesWindow(int x, int y, int span, double a_deg) const {
        for (const auto& z : zones)
            if (z.suppressesWindow(x, y, span, a_deg)) return true;
        return false;
    }
};

static BBSResult branchAndBound(const PrecomputedGrids& g,
                                const std::vector<RotatedScan>& rots,
                                const std::vector<double>& angles,
                                float min_score,
                                const MultiSuppression& sup = MultiSuppression()) {
    const int L = g_levels;
    const int n_ang = (int)rots.size();
    const int step = 1 << L;

    std::atomic<uint32_t> shared_best(f2u(min_score));
    Candidate global_best{0, 0, 0, 0, min_score};
    long total_nodes = 0;

    // One independent depth-first search per rotation; they share only the
    // incumbent used for bounding, which is monotonically non-decreasing, so a
    // stale read can only make the pruning weaker -- never wrong.
#pragma omp parallel
    {
        Candidate local_best{0, 0, 0, 0, min_score};
        long local_nodes = 0;
        std::vector<Candidate> roots, stack, ch;

#pragma omp for schedule(dynamic, 1) nowait
        for (int a = 0; a < n_ang; a++) {
            const double a_deg = angles[a] * 180.0 / M_PI;
            roots.clear();
            for (int y = 0; y < g.h; y += step)
                for (int x = 0; x < g.w; x += step) {
                    if (sup.suppressesWindow(x, y, step, a_deg)) continue;
                    roots.push_back({a, x, y, L,
                                     scoreCandidate(g, rots[a], L, x, y)});
                }
            // best-first at the root, so the incumbent climbs fast
            std::sort(roots.begin(), roots.end(),
                      [](const Candidate& p, const Candidate& q) { return p.score < q.score; });
            stack.assign(roots.begin(), roots.end());

            while (!stack.empty()) {
                Candidate c = stack.back();
                stack.pop_back();
                local_nodes++;
                float bound = u2f(shared_best.load(std::memory_order_relaxed));
                if (c.score <= bound) continue;
                if (c.level == 0) {
                    if (sup.suppressesLeaf(c.x, c.y, a_deg)) continue;
                    if (c.score > local_best.score) local_best = c;
                    // raise the shared incumbent
                    uint32_t cur = shared_best.load(std::memory_order_relaxed);
                    while (u2f(cur) < c.score &&
                           !shared_best.compare_exchange_weak(cur, f2u(c.score),
                                                              std::memory_order_relaxed)) {}
                    continue;
                }
                int half = 1 << (c.level - 1);
                ch.clear();
                for (int dy = 0; dy < 2; dy++)
                    for (int dx = 0; dx < 2; dx++) {
                        int nx = c.x + dx * half, ny = c.y + dy * half;
                        if (nx >= g.w || ny >= g.h) continue;
                        if (sup.suppressesWindow(nx, ny, half, a_deg)) continue;
                        ch.push_back({a, nx, ny, c.level - 1,
                                      scoreCandidate(g, rots[a], c.level - 1, nx, ny)});
                    }
                std::sort(ch.begin(), ch.end(),
                          [](const Candidate& p, const Candidate& q) { return p.score < q.score; });
                for (const Candidate& k : ch)
                    if (k.score > bound) stack.push_back(k);
            }
        }

#pragma omp critical
        {
            if (local_best.score > global_best.score) global_best = local_best;
            total_nodes += local_nodes;
        }
    }

    BBSResult r;
    r.x = global_best.x; r.y = global_best.y;
    r.yaw = angles[global_best.angle_idx];
    r.score = global_best.score;
    r.nodes = total_nodes;
    return r;
}

// ---------------- main ----------------
int main(int argc, char** argv) {
    std::string scans_csv = (argc > 1) ? argv[1] : "scans_dump.csv";
    std::string map_path = (argc > 2) ? argv[2]
        : "../maps/intel_research_lab/intel_research_lab.pgm";
    if (argc > 3) g_angle_step_deg = std::atof(argv[3]);
    if (argc > 4) g_res = std::atof(argv[4]);
    if (argc > 5) g_min_range = std::atof(argv[5]);
    if (const char* e = getenv("BBS_TOPK")) g_topk = std::atoi(e);
    if (const char* e = getenv("BBS_TOPK_OUT")) {
        g_topk_out = fopen(e, "w");
        if (!g_topk_out) { fprintf(stderr, "cannot open %s\n", e); return 1; }
        fprintf(g_topk_out, "cond,trial,k,x,y,yaw,score\n");
    }
    // AMCL の尤度場と同じ設定で回せるようにする (sigma_hit / laser_likelihood_max_dist)
    if (const char* e = getenv("BBS_SIGMA")) g_sigma = std::atof(e);
    if (const char* e = getenv("OFL_MAX_RANGE")) g_max_range = std::atof(e);
    if (const char* e = getenv("BBS_SECOND_PEAK")) g_second_peak = std::atoi(e) != 0;
    if (const char* e = getenv("BBS_NMS_M")) g_nms_m = std::atof(e);
    if (const char* e = getenv("BBS_NMS_DEG")) g_nms_deg = std::atof(e);
    if (const char* e = getenv("BBS_TRUNC")) g_trunc = std::atof(e);
    double map_res = 0.02;
    const double origin_x = -10.0, origin_y = -23.0;
    if (const char* e = getenv("OFL_MAP_RES")) map_res = std::atof(e);

    cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    if (map_img.empty()) { fprintf(stderr, "map load failed: %s\n", map_path.c_str()); return 1; }

    // ---- occupancy at the search resolution: a cell is occupied if any
    //      original pixel inside it is occupied (standard grid downsampling).
    int gw = (int)std::ceil(map_img.cols * map_res / g_res);
    int gh = (int)std::ceil(map_img.rows * map_res / g_res);
    cv::Mat occ = cv::Mat::zeros(gh, gw, CV_8UC1);   // row 0 = bottom (y up)
    for (int py = 0; py < map_img.rows; py++)
        for (int px = 0; px < map_img.cols; px++)
            if (map_img.at<uint8_t>(py, px) < 89) {
                double wx = px * map_res + origin_x;
                double wy = (map_img.rows - 1 - py) * map_res + origin_y;
                int gx = (int)((wx - origin_x) / g_res);
                int gy = (int)((wy - origin_y) / g_res);
                if (gx >= 0 && gx < gw && gy >= 0 && gy < gh) occ.at<uint8_t>(gy, gx) = 255;
            }

    // ---- likelihood field (Olson's rasterized lookup table)
    cv::Mat free_mask, dist;
    cv::bitwise_not(occ, free_mask);
    cv::distanceTransform(free_mask, dist, cv::DIST_L2, 3);
    std::vector<float> base((size_t)gw * gh);
    for (int y = 0; y < gh; y++)
        for (int x = 0; x < gw; x++) {
            double d = dist.at<float>(y, x) * g_res;
            base[(size_t)y * gw + x] =
                (d > g_trunc) ? 0.0f : (float)std::exp(-d * d / (2 * g_sigma * g_sigma));
        }

    auto tp0 = std::chrono::steady_clock::now();
    PrecomputedGrids grids = buildGrids(base, gw, gh, g_levels);
    auto tp1 = std::chrono::steady_clock::now();
    fprintf(stderr, "grid %dx%d @ %.3f m, sigma %.2f, trunc %.2f, %d levels, precompute %.2fs\n",
            gw, gh, g_res, g_sigma, g_trunc, g_levels,
            std::chrono::duration<double>(tp1 - tp0).count());

    // ---- angular resolution.  Hess et al. eq. (2): the step at which a point
    //      at max range moves less than one cell.
    if (g_angle_step_deg <= 0) {
        double c = 1.0 - g_res * g_res / (2.0 * g_max_range * g_max_range);
        g_angle_step_deg = std::acos(std::max(-1.0, std::min(1.0, c))) * 180.0 / M_PI;
    }
    int n_ang = (int)std::round(360.0 / g_angle_step_deg);
    std::vector<double> angles(n_ang);
    for (int i = 0; i < n_ang; i++) angles[i] = 2 * M_PI * i / n_ang;
    fprintf(stderr, "angle step %.4f deg -> %d rotations, min_range %.2f m\n",
            360.0 / n_ang, n_ang, g_min_range);

    // ---- scans
    FILE* f = fopen(scans_csv.c_str(), "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", scans_csv.c_str()); return 1; }
    std::vector<char> line(1 << 16);
    if (!fgets(line.data(), (int)line.size(), f)) return 1;   // header

    printf("cond,trial,gt_x,gt_y,gt_th_rad,est_x,est_y,est_yaw_rad,pos_err,ang_err,"
           "score,time_s,nodes,score2,margin\n");

    while (fgets(line.data(), (int)line.size(), f)) {
        char cond[64];
        int trial;
        double gx, gy, gth;
        std::vector<double> ranges(kNumBeams);
        char* p = line.data();
        char* tok = strtok(p, ",");
        if (!tok) continue;
        snprintf(cond, sizeof(cond), "%s", tok);
        trial = std::atoi(strtok(nullptr, ","));
        gx = std::atof(strtok(nullptr, ","));
        gy = std::atof(strtok(nullptr, ","));
        gth = std::atof(strtok(nullptr, ","));
        bool ok = true;
        for (int i = 0; i < kNumBeams; i++) {
            char* t = strtok(nullptr, ",");
            if (!t) { ok = false; break; }
            ranges[i] = std::atof(t);
        }
        if (!ok) continue;

        // valid returns only (max range == no return, same rule as the OFL evaluator)
        std::vector<double> bx, by;
        for (int i = 0; i < kNumBeams; i++) {
            if (ranges[i] >= g_max_range - 0.01 || ranges[i] < 0.05) continue;
            if (g_min_range > 0.0 && ranges[i] < g_min_range) continue;
            double a = 2.0 * M_PI * i / kNumBeams;
            bx.push_back(ranges[i] * std::cos(a));
            by.push_back(ranges[i] * std::sin(a));
        }

        auto t0 = std::chrono::steady_clock::now();
        std::vector<RotatedScan> rots(n_ang);
#pragma omp parallel for schedule(static)
        for (int a = 0; a < n_ang; a++) {
            double ca = std::cos(angles[a]), sa = std::sin(angles[a]);
            rots[a].px.resize(bx.size());
            rots[a].py.resize(bx.size());
            for (size_t i = 0; i < bx.size(); i++) {
                double rx = ca * bx[i] - sa * by[i];
                double ry = sa * bx[i] + ca * by[i];
                rots[a].px[i] = (int)std::lround(rx / g_res);
                rots[a].py[i] = (int)std::lround(ry / g_res);
            }
        }
        BBSResult r = branchAndBound(grids, rots, angles, 0.0f);
        double score2 = 0.0;
        if (g_second_peak) {
            MultiSuppression sup;
            Suppression z;
            z.active = true;
            z.cx = (int)r.x; z.cy = (int)r.y;
            z.yaw_deg = r.yaw * 180.0 / M_PI;
            z.dist_cells = g_nms_m / g_res;
            z.sep_deg = g_nms_deg;
            sup.zones.push_back(z);
            BBSResult r2 = branchAndBound(grids, rots, angles, 0.0f, sup);
            score2 = r2.score;
            r.nodes += r2.nodes;
        }
        if (g_topk > 0 && g_topk_out) {
            MultiSuppression sup;
            BBSResult rk = r;
            for (int k = 1; k <= g_topk; k++) {
                if (k > 1) {
                    Suppression z;
                    z.active = true;
                    z.cx = (int)rk.x; z.cy = (int)rk.y;
                    z.yaw_deg = rk.yaw * 180.0 / M_PI;
                    z.dist_cells = g_nms_m / g_res;
                    z.sep_deg = g_nms_deg;
                    sup.zones.push_back(z);
                    rk = branchAndBound(grids, rots, angles, 0.0f, sup);
                    r.nodes += rk.nodes;
                    if (rk.score <= 0.0) break;
                }
                fprintf(g_topk_out, "%s,%d,%d,%.4f,%.4f,%.5f,%.2f\n",
                        cond, trial, k,
                        (rk.x + 0.5) * g_res + origin_x,
                        (rk.y + 0.5) * g_res + origin_y,
                        rk.yaw, rk.score);
            }
            fflush(g_topk_out);
        }
        auto t1 = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t1 - t0).count();

        // cell -> world (cell centre)
        double ex = (r.x + 0.5) * g_res + origin_x;
        double ey = (r.y + 0.5) * g_res + origin_y;
        double pe = std::hypot(ex - gx, ey - gy);
        double ae = std::fabs(std::atan2(std::sin(r.yaw - gth), std::cos(r.yaw - gth)))
                    * 180.0 / M_PI;

        double margin = (score2 > 0.0) ? (r.score / score2) : 0.0;
        printf("%s,%d,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%.3f,%.2f,%.4f,%ld,%.2f,%.4f\n",
               cond, trial, gx, gy, gth, ex, ey, r.yaw, pe, ae, r.score, dt, r.nodes,
               score2, margin);
        fflush(stdout);
    }
    fclose(f);
    return 0;
}
