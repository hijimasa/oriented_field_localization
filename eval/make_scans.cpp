// 姿勢サンプリングと外乱付きスキャンの生成。
//
// 占有格子地図の自由空間から姿勢を撒き、レイキャストで 360 本のスキャンを作り、
// 15 種類の外乱を掛けて CSV へ書き出す。手法には一切依存しないので、
// 同じ CSV を本手法 (ofl_eval) と BBS (bbs_eval) の両方へ渡せば対応比較になる。
//
//   ./make_scans <poses> > scans.csv
//   環境変数: OFL_MAP_PGM, OFL_MAP_RES, OFL_MAX_RANGE, OFL_NEAR_WALL_M
//
// 出力: cond,trial,gt_x,gt_y,gt_th_rad,r0,...,r359
//
// 姿勢・外乱の乱数系列は再現性のため固定してある。同じ地図・同じ試行数なら同じ
// スキャンが出るため、条件の順序と乱数の引き方を変える場合はベンチマークを再生成すること。
#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace
{
constexpr int kNumBeams = 360;
double g_max_range = 10.0;

struct Disc { double cx, cy, r; };

struct World
{
  cv::Mat map;
  double res = 0.02;
  double origin_x = 0, origin_y = 0;
  std::vector<Disc> phantom;
  std::vector<Disc> removed;
  bool sector_active = false;
  double sector_center = 0, sector_half = 0, sector_range = 0.6;
  bool sector_dropout = false;
  double thin_frac = 0.0;
  unsigned thin_seed = 0;
};

inline bool inDiscs(const std::vector<Disc> & ds, double x, double y)
{
  for (const Disc & d : ds) {
    double dx = x - d.cx, dy = y - d.cy;
    if (dx * dx + dy * dy < d.r * d.r) return true;
  }
  return false;
}

inline bool isOccupied(const cv::Mat & m, double res, double ox, double oy, double wx, double wy)
{
  int px = static_cast<int>((wx - ox) / res);
  int py = m.rows - 1 - static_cast<int>((wy - oy) / res);
  if (px < 0 || px >= m.cols || py < 0 || py >= m.rows) return false;
  return m.at<uint8_t>(py, px) < 89;
}

inline bool isFree(const cv::Mat & m, double res, double ox, double oy, double wx, double wy)
{
  int px = static_cast<int>((wx - ox) / res);
  int py = m.rows - 1 - static_cast<int>((wy - oy) / res);
  if (px < 0 || px >= m.cols || py < 0 || py >= m.rows) return false;
  return m.at<uint8_t>(py, px) > 205;
}

double rayDisc(double px, double py, double dx, double dy, const Disc & d)
{
  double ox = d.cx - px, oy = d.cy - py;
  double t = ox * dx + oy * dy;
  if (t < 0) return 1e30;
  double perp2 = (ox * ox + oy * oy) - t * t;
  double r2 = d.r * d.r;
  if (perp2 > r2) return 1e30;
  double th = t - std::sqrt(r2 - perp2);
  return th > 0 ? th : 1e30;
}

std::vector<double> simulateScan(const World & w, double rx, double ry, double rtheta)
{
  std::vector<double> ranges(kNumBeams, g_max_range);
  double step = w.res * 0.5;
  int max_steps = static_cast<int>(g_max_range / step);
  for (int i = 0; i < kNumBeams; i++) {
    double a = 2.0 * M_PI * i / kNumBeams;
    double wa = a + rtheta;
    double cx = std::cos(wa), cy = std::sin(wa);
    if (w.sector_active) {
      double rel = std::atan2(std::sin(a - w.sector_center), std::cos(a - w.sector_center));
      if (std::fabs(rel) < w.sector_half) {
        ranges[i] = w.sector_dropout ? g_max_range : w.sector_range;
        continue;
      }
    }
    double map_hit = g_max_range;
    double x = rx, y = ry;
    for (int s = 1; s <= max_steps; s++) {
      x += cx * step; y += cy * step;
      int px = static_cast<int>((x - w.origin_x) / w.res);
      int py = w.map.rows - 1 - static_cast<int>((y - w.origin_y) / w.res);
      if (px < 0 || px >= w.map.cols || py < 0 || py >= w.map.rows) break;
      if (w.map.at<uint8_t>(py, px) < 89) {
        if (!w.removed.empty() && inDiscs(w.removed, x, y)) continue;
        map_hit = s * step;
        break;
      }
    }
    double obs_hit = 1e30;
    for (const Disc & d : w.phantom) obs_hit = std::min(obs_hit, rayDisc(rx, ry, cx, cy, d));
    ranges[i] = std::min(map_hit, std::min(obs_hit, g_max_range));
  }
  if (w.thin_frac > 0.0) {
    std::mt19937 trng(w.thin_seed);
    std::uniform_real_distribution<double> u(0, 1);
    for (int i = 0; i < kNumBeams; i++) {
      if (u(trng) < w.thin_frac) ranges[i] = g_max_range;
    }
  }
  return ranges;
}

struct Cond
{
  std::string name;
  int n_phantom = 0;
  int n_removed = 0;
  double sector_deg = 0;
  bool dropout = false;
  double thin = 0.0;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string map_path = "out/maps/synthetic.pgm";
  if (const char * e = getenv("OFL_MAP_PGM")) map_path = e;
  double map_res = 0.02;
  const double origin_x = -10.0, origin_y = -23.0;
  if (const char * e = getenv("OFL_MAP_RES")) map_res = std::atof(e);
  if (const char * e = getenv("OFL_MAX_RANGE")) g_max_range = std::atof(e);
  int n_trials = (argc > 1) ? std::atoi(argv[1]) : 20;

  cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
  if (map_img.empty()) {
    fprintf(stderr, "map load failed: %s\n", map_path.c_str());
    return 1;
  }
  fprintf(stderr, "map %dx%d @ %.3f m/px, max_range %.1f m\n",
    map_img.cols, map_img.rows, map_res, g_max_range);

  // ---- 姿勢のサンプル: 壁から 0.6 m 以上離れた自由空間 ----
  cv::Mat free_mask = map_img > 205, eroded;
  int er = std::max(1, static_cast<int>(std::lround(0.6 / map_res)));
  cv::erode(free_mask, eroded,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * er + 1, 2 * er + 1)));
  // 実地図では未探索のキャンバスも free で保存されているので、そのまま撒くと
  // 「10 m 以内に何も無い」場所が大量に選ばれる。壁の近くだけに限定する。
  if (const char * e = getenv("OFL_NEAR_WALL_M")) {
    double near_m = std::atof(e);
    if (near_m > 0) {
      cv::Mat occ_mask = map_img < 89, near;
      int r = std::max(1, static_cast<int>(std::lround(near_m / map_res)));
      cv::dilate(occ_mask, near,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1)));
      cv::bitwise_and(eroded, near, eroded);
      fprintf(stderr, "pose sampling restricted to within %.1f m of a wall\n", near_m);
    }
  }
  std::vector<cv::Point> free_pts;
  cv::findNonZero(eroded, free_pts);
  if (free_pts.empty()) { fprintf(stderr, "no free space to sample\n"); return 1; }
  fprintf(stderr, "pose candidates: %zu px\n", free_pts.size());

  std::mt19937 pose_rng(42);
  std::uniform_int_distribution<int> pick(0, static_cast<int>(free_pts.size()) - 1);
  std::uniform_real_distribution<double> uang(0, 2 * M_PI);
  struct Pose { double x, y, th; };
  std::vector<Pose> poses;
  for (int t = 0; t < n_trials; t++) {
    cv::Point p = free_pts[pick(pose_rng)];
    poses.push_back({p.x * map_res + origin_x,
        (map_img.rows - 1 - p.y) * map_res + origin_y, uang(pose_rng)});
  }

  // 条件の順序は乱数系列を決めるので変えないこと (末尾への追加のみ可)
  const std::vector<Cond> conds = {
    {"clean"},
    {"phantom3", 3}, {"phantom8", 8}, {"phantom15", 15}, {"phantom25", 25},
    {"sector60", 0, 0, 60}, {"sector120", 0, 0, 120}, {"sector180", 0, 0, 180},
    {"dropout90", 0, 0, 90, true}, {"dropout180", 0, 0, 180, true},
    {"mapremove3", 0, 3}, {"mapremove8", 0, 8},
    {"thin50", 0, 0, 0, false, 0.50}, {"thin75", 0, 0, 0, false, 0.75},
    {"thin90", 0, 0, 0, false, 0.90},
  };

  printf("cond,trial,gt_x,gt_y,gt_th_rad");
  for (int i = 0; i < kNumBeams; i++) printf(",r%d", i);
  printf("\n");

  World base;
  base.map = map_img;
  base.res = map_res;
  base.origin_x = origin_x;
  base.origin_y = origin_y;

  for (size_t ci = 0; ci < conds.size(); ci++) {
    const Cond & cond = conds[ci];
    for (int t = 0; t < n_trials; t++) {
      const Pose & P = poses[t];
      std::mt19937 rng(1000 * static_cast<int>(ci) + t + 7);
      std::uniform_real_distribution<double> ur01(0, 1);

      World w = base;
      int placed = 0, guard = 0;
      while (placed < cond.n_phantom && guard++ < 2000) {
        double ang = ur01(rng) * 2 * M_PI;
        double dist = 1.0 + ur01(rng) * 7.0;
        double cx = P.x + dist * std::cos(ang);
        double cy = P.y + dist * std::sin(ang);
        double r = 0.15 + ur01(rng) * 0.20;
        if (!isFree(w.map, w.res, w.origin_x, w.origin_y, cx, cy)) continue;
        w.phantom.push_back({cx, cy, r});
        placed++;
      }
      placed = 0; guard = 0;
      while (placed < cond.n_removed && guard++ < 5000) {
        double ang = ur01(rng) * 2 * M_PI;
        double dist = 0.5 + ur01(rng) * 7.5;
        double cx = P.x + dist * std::cos(ang);
        double cy = P.y + dist * std::sin(ang);
        if (!isOccupied(w.map, w.res, w.origin_x, w.origin_y, cx, cy)) continue;
        double r = 0.3 + ur01(rng) * 0.5;
        w.removed.push_back({cx, cy, r});
        placed++;
      }
      if (cond.thin > 0.0) {
        w.thin_frac = cond.thin;
        w.thin_seed = 9000u + 100u * static_cast<unsigned>(t);
      }
      if (cond.sector_deg > 0) {
        w.sector_active = true;
        w.sector_center = ur01(rng) * 2 * M_PI;
        w.sector_half = cond.sector_deg * M_PI / 180.0 / 2.0;
        w.sector_range = 0.5 + ur01(rng) * 0.5;
        w.sector_dropout = cond.dropout;
      }

      std::vector<double> ranges = simulateScan(w, P.x, P.y, P.th);
      printf("%s,%d,%.4f,%.4f,%.5f", cond.name.c_str(), t, P.x, P.y, P.th);
      for (int i = 0; i < kNumBeams; i++) printf(",%.3f", ranges[i]);
      printf("\n");
    }
    fprintf(stderr, "cond %s done\n", cond.name.c_str());
  }
  return 0;
}
