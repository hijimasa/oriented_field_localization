// AMCL 監視 (reseed_policy.hpp) の幾何判定を統計的に較正する評価器。
//
// docs/amcl_supervision.md の限界に挙がっていた「監視の発火の実測が 1 件しか
// なく、しきい値 (supervise_max_wfrac / supervise_wfrac_excess) が統計的に
// 較正されていない」を埋める。閉ループで壊れた AMCL を大量に観測するのは
// 高くつくので、make_scans の外乱付きスキャン (15 条件) の上で **壊れ方を
// 姿勢のずらしとして合成し**、幾何判定 (wfrac_other > max_wfrac かつ
// excess > wfrac_excess) の分離を条件横断で測る。
//
//   ./reseed_margin_eval scans.csv map.pgm > reseed_margin.csv
//
// 各試行で真姿勢の周りに 5 種類の姿勢を撒く:
//   self     真姿勢 + OFL の実測精度ぶんの雑音 (<=0.10 m / <=3 deg)。基準線
//   healthy  健全な AMCL (<=0.15 m / <=5 deg。閉ループの実測中央 0.055 m)。
//            **発火してはいけない**
//   drifted  劣化した AMCL (0.15–1.5 m / <=20 deg)。発火が始まる境界を曲線で見る
//   mislock  誤ロック (1–3 m、角度は任意)。実測で捕まえた 1 件 (1.55 m / 94 deg)
//            がこの帯に居る。**捕まえたい**
//   kidnap   遠方の自由空間 (>=3 m、角度は任意)。**捕まえたい**
//
// 各サンプルについて、判定の 2 段 (幾何: wfrac_other > max_wfrac かつ
// excess > wfrac_excess、不一致: self との距離 > 0.5 m または > 20 deg) に
// 必要な量をすべて書き出す。集計 (summarize_reseed.py) は完全な per-scan
// ルール (幾何 AND 不一致) で発火率を出す。連続 10 スキャンの鈍化は
// この上に乗るので、ここの発火率は per-scan の上界である。
//
// **不一致は真姿勢ではなく self (基準線側の姿勢) から測る。**ノードが比べるのは
// OFL の推定と AMCL の推定であって、どちらも真姿勢ではない。
//
// 乱数は (条件, 試行) から決まるので、同じ設定なら常に同じ姿勢が出る。
#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using oriented_field_localization::Beam;
using oriented_field_localization::MatcherParams;
using oriented_field_localization::OrientedFieldLocalizer;
using oriented_field_localization::PoseCandidate;

namespace
{
constexpr int kNumBeams = 360;

double envD(const char * k, double v) { const char * e = getenv(k); return e ? std::atof(e) : v; }
int envI(const char * k, int v) { const char * e = getenv(k); return e ? std::atoi(e) : v; }

/// make_scans と同じ自由空間判定 (行 0 が上端 = 最大 y)。
bool isFree(const cv::Mat & m, double res, double ox, double oy, double wx, double wy)
{
  int px = static_cast<int>((wx - ox) / res);
  int py = m.rows - 1 - static_cast<int>((wy - oy) / res);
  if (px < 0 || px >= m.cols || py < 0 || py >= m.rows) return false;
  return m.at<uint8_t>(py, px) > 205;
}

struct Sample { double x, y, yaw, d_m, d_deg; };

}  // namespace

int main(int argc, char ** argv)
{
  std::string scans_csv = (argc > 1) ? argv[1] : "out/scans.csv";
  std::string map_path = (argc > 2) ? argv[2] : "out/maps/synthetic.pgm";
  const double map_res = envD("OFL_MAP_RES", 0.02);
  const double origin_x = -10.0, origin_y = -23.0;   // make_scans と同一

  MatcherParams p;
  p.match_resolution = envD("OFL_MATCH_RES", 0.05);
  p.max_range = envD("OFL_MAX_RANGE", 10.0);
  p.min_range = envD("OFL_MIN_RANGE", 0.0);
  p.margin_pixels = std::max(4, static_cast<int>(std::lround(
        envD("OFL_MARGIN_M", 10.0) / p.match_resolution)));
  const int per_class = envI("OFL_RESEED_SAMPLES", 3);

  cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
  if (map_img.empty()) { fprintf(stderr, "map load failed: %s\n", map_path.c_str()); return 1; }

  OrientedFieldLocalizer loc(p);
  loc.setMap(map_img, map_res, origin_x, origin_y);

  // kidnap の撒き先: ロボット半径ぶん壁から離れた自由空間
  cv::Mat free_mask = map_img > 205, eroded;
  const int er = std::max(1, static_cast<int>(std::lround(0.3 / map_res)));
  cv::erode(free_mask, eroded,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * er + 1, 2 * er + 1)));
  std::vector<cv::Point> free_pts;
  cv::findNonZero(eroded, free_pts);
  if (free_pts.empty()) { fprintf(stderr, "no free space\n"); return 1; }

  FILE * f = fopen(scans_csv.c_str(), "r");
  if (!f) { fprintf(stderr, "cannot open %s\n", scans_csv.c_str()); return 1; }
  std::vector<char> line(1 << 16);
  if (!fgets(line.data(), static_cast<int>(line.size()), f)) return 1;   // header

  printf("cond,trial,kind,d_m,d_deg,wfrac_self,wfrac_other,excess\n");

  while (fgets(line.data(), static_cast<int>(line.size()), f)) {
    char cond[64];
    char * tok = strtok(line.data(), ",");
    if (!tok) continue;
    snprintf(cond, sizeof(cond), "%s", tok);
    const int trial = std::atoi(strtok(nullptr, ","));
    const double gx = std::atof(strtok(nullptr, ","));
    const double gy = std::atof(strtok(nullptr, ","));
    const double gth = std::atof(strtok(nullptr, ","));
    std::vector<Beam> beams(kNumBeams);
    bool ok = true;
    for (int i = 0; i < kNumBeams; i++) {
      char * t = strtok(nullptr, ",");
      if (!t) { ok = false; break; }
      beams[i].range = std::atof(t);
      beams[i].bearing = 2.0 * M_PI * i / kNumBeams;
    }
    if (!ok) continue;

    std::mt19937 rng(2654435761u ^ (std::hash<std::string>{}(cond) * 31u + trial));
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    auto offsetPose = [&](double r_lo, double r_hi, double yaw_deg, bool any_yaw,
        bool need_free, Sample * s) -> bool {
        for (int tries = 0; tries < 50; tries++) {
          const double r = r_lo + (r_hi - r_lo) *
            (r_lo > 0 ? u01(rng) : std::sqrt(u01(rng)));   // 下限 0 なら円内一様
          const double a = 2 * M_PI * u01(rng);
          const double dy = any_yaw ? (2 * u01(rng) - 1) * M_PI
            : (2 * u01(rng) - 1) * yaw_deg * M_PI / 180.0;
          const double x = gx + r * std::cos(a), y = gy + r * std::sin(a);
          if (need_free && !isFree(map_img, map_res, origin_x, origin_y, x, y)) continue;
          *s = {x, y, gth + dy, r, std::fabs(dy) * 180.0 / M_PI};
          return true;
        }
        return false;
      };
    auto kidnapPose = [&](Sample * s) -> bool {
        std::uniform_int_distribution<int> pick(0, static_cast<int>(free_pts.size()) - 1);
        for (int tries = 0; tries < 200; tries++) {
          const cv::Point q = free_pts[pick(rng)];
          const double x = q.x * map_res + origin_x;
          const double y = (map_img.rows - 1 - q.y) * map_res + origin_y;
          const double d = std::hypot(x - gx, y - gy);
          if (d < 3.0) continue;
          const double dy = (2 * u01(rng) - 1) * M_PI;
          *s = {x, y, gth + dy, d, std::fabs(dy) * 180.0 / M_PI};
          return true;
        }
        return false;
      };

    // 基準線: 真姿勢 + OFL の実測精度ぶんの雑音 (docs/simulation.md: 中央 0.04–0.06 m)
    Sample self;
    if (!offsetPose(0.0, 0.10, 3.0, false, false, &self)) continue;
    PoseCandidate sc;
    sc.x = self.x; sc.y = self.y; sc.yaw = self.yaw;
    const double wfrac_self = loc.wallMissFraction(sc, beams);

    struct Cls { const char * name; double lo, hi, yaw; bool any_yaw, free_chk, kidnap; };
    const Cls classes[] = {
      {"healthy", 0.0, 0.15, 5.0, false, false, false},
      {"drifted", 0.15, 1.50, 20.0, false, true, false},
      {"mislock", 1.0, 3.00, 0.0, true, true, false},
      {"kidnap", 0.0, 0.00, 0.0, true, true, true},
    };
    for (const Cls & c : classes) {
      for (int k = 0; k < per_class; k++) {
        Sample s;
        const bool got = c.kidnap ? kidnapPose(&s)
          : offsetPose(c.lo, c.hi, c.yaw, c.any_yaw, c.free_chk, &s);
        if (!got) continue;
        PoseCandidate oc;
        oc.x = s.x; oc.y = s.y; oc.yaw = s.yaw;
        const double w = loc.wallMissFraction(oc, beams);
        // 不一致はノードが実際に比べる 2 つの推定 (self と other) の間で測る
        const double dm = std::hypot(s.x - self.x, s.y - self.y);
        double dd = std::fabs(s.yaw - self.yaw) * 180.0 / M_PI;
        while (dd > 180.0) dd = std::fabs(dd - 360.0);
        printf("%s,%d,%s,%.3f,%.1f,%.4f,%.4f,%.4f\n",
          cond, trial, c.name, dm, dd, wfrac_self, w, w - wfrac_self);
      }
    }
  }
  fclose(f);
  return 0;
}
