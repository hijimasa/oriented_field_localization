// 本手法 (画像空間の向き付き場相関) の評価器。
//
// make_scans が書いた scans.csv を読み、各スキャンを大域探索して
// cond,trial,... の CSV を書き出す。BBS ベースライン (bbs_eval) と同じ
// スキャンを読むので対応比較になる。
//
//   ./ofl_eval scans.csv map.pgm > ofl.csv
//
// 環境変数 (既定は config/params.yaml と一致):
//   OFL_MAP_RES, OFL_MAX_RANGE, OFL_MATCH_RES, OFL_MARGIN_M, OFL_LEVELS,
//   OFL_ANGLE_STEP, OFL_PEAKS, OFL_POOL, OFL_LAMBDA, OFL_WFRAC_MARGIN,
//   OFL_STAGEDUMP=1  段階別の所要時間を stderr へ出す
//   OFL_CANDDUMP=<csv>  候補プールを (score, wfrac, 誤差) ごと書き出す
#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
}  // namespace

int main(int argc, char ** argv)
{
  std::string scans_csv = (argc > 1) ? argv[1] : "out/scans.csv";
  std::string map_path = (argc > 2) ? argv[2] : "out/maps/synthetic.pgm";
  double map_res = envD("OFL_MAP_RES", 0.02);
  const double origin_x = -10.0, origin_y = -23.0;   // 自己完結した評価なので任意

  MatcherParams p;
  p.match_resolution = envD("OFL_MATCH_RES", 0.05);
  p.max_range = envD("OFL_MAX_RANGE", 10.0);
  p.min_range = envD("OFL_MIN_RANGE", 0.0);
  p.margin_pixels = std::max(4, static_cast<int>(std::lround(
        envD("OFL_MARGIN_M", 10.0) / p.match_resolution)));
  p.pyramid_levels = envI("OFL_LEVELS", 3);
  p.coarse_angle_step = envI("OFL_ANGLE_STEP", 6);
  p.peaks_per_angle = envI("OFL_PEAKS", 8);
  p.candidate_pool_size = envI("OFL_POOL", 15);
  p.normal_weight = envD("OFL_LAMBDA", 1.0);
  const double wfrac_margin = envD("OFL_WFRAC_MARGIN", 1.05);
  const bool stage_dump = envI("OFL_STAGEDUMP", 0) != 0;

  FILE * cand_dump = nullptr;
  if (const char * e = getenv("OFL_CANDDUMP")) {
    cand_dump = fopen(e, "w");
    if (!cand_dump) { fprintf(stderr, "canddump open failed: %s\n", e); return 1; }
    fprintf(cand_dump, "cond,trial,k,score,wfrac,pos_err,ang_err\n");
  }

  cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
  if (map_img.empty()) { fprintf(stderr, "map load failed: %s\n", map_path.c_str()); return 1; }

  OrientedFieldLocalizer loc(p);
  auto t_init0 = std::chrono::steady_clock::now();
  loc.setMap(map_img, map_res, origin_x, origin_y);
  auto t_init1 = std::chrono::steady_clock::now();
  fprintf(stderr,
    "match_res %.3f m/px  margin %d px (min %d, recommended %d)  levels %d  "
    "angle step %d deg  peaks %d  pool %d  lambda %.2f  wfrac margin %.2f  "
    "min_range %.2f m\n",
    p.match_resolution, p.margin_pixels, p.minimumMargin(), p.recommendedMargin(),
    p.pyramid_levels, p.coarse_angle_step, p.peaks_per_angle, p.candidate_pool_size,
    p.normal_weight, wfrac_margin, p.min_range);
  fprintf(stderr, "map init: %.2fs\n",
    std::chrono::duration<double>(t_init1 - t_init0).count());

  FILE * f = fopen(scans_csv.c_str(), "r");
  if (!f) { fprintf(stderr, "cannot open %s\n", scans_csv.c_str()); return 1; }
  std::vector<char> line(1 << 16);
  if (!fgets(line.data(), static_cast<int>(line.size()), f)) return 1;   // header

  printf("cond,trial,gt_x,gt_y,gt_th_deg,est_x,est_y,est_th_deg,pos_err,ang_err,"
         "score,margin,pool_hit,time_s,wfrac_gate_pos_err,wfrac_gate_ang_err\n");

  OrientedFieldLocalizer::StageTimes acc;
  int n_scans = 0;

  while (fgets(line.data(), static_cast<int>(line.size()), f)) {
    char cond[64];
    char * tok = strtok(line.data(), ",");
    if (!tok) continue;
    snprintf(cond, sizeof(cond), "%s", tok);
    int trial = std::atoi(strtok(nullptr, ","));
    double gx = std::atof(strtok(nullptr, ","));
    double gy = std::atof(strtok(nullptr, ","));
    double gth = std::atof(strtok(nullptr, ","));
    std::vector<Beam> beams(kNumBeams);
    bool ok = true;
    for (int i = 0; i < kNumBeams; i++) {
      char * t = strtok(nullptr, ",");
      if (!t) { ok = false; break; }
      beams[i].range = std::atof(t);
      beams[i].bearing = 2.0 * M_PI * i / kNumBeams;
    }
    if (!ok) continue;
    double gt_deg = gth * 180.0 / M_PI;
    while (gt_deg < 0) gt_deg += 360.0;

    auto ts = std::chrono::steady_clock::now();
    std::vector<PoseCandidate> cands = loc.localize(beams);
    auto te = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(te - ts).count();

    const auto & st = loc.lastStageTimes();
    acc.repr += st.repr; acc.xform += st.xform; acc.prep += st.prep;
    acc.corr += st.corr; acc.peak += st.peak; acc.fine += st.fine;
    n_scans++;

    if (cands.empty()) {
      printf("%s,%d,%.3f,%.3f,%.1f,nan,nan,nan,nan,nan,nan,nan,0,%.4f,nan,nan\n",
        cond, trial, gx, gy, gt_deg, elapsed);
      fflush(stdout);
      continue;
    }

    auto errOf = [&](const PoseCandidate & c, double & pe, double & ae) {
        pe = std::hypot(c.x - gx, c.y - gy);
        ae = std::fabs(c.yaw * 180.0 / M_PI - gt_deg);
        while (ae > 180) ae = std::fabs(ae - 360);
      };
    double pos_err, ang_err;
    errOf(cands[0], pos_err, ang_err);
    int pool_hit = 0;
    for (const PoseCandidate & c : cands) {
      double pe, ae;
      errOf(c, pe, ae);
      if (pe < 1.0 && ae < 15.0) { pool_hit = 1; break; }
    }
    double margin = (cands.size() > 1 && cands[1].score > 0)
      ? cands[0].score / cands[1].score : 1e9;

    double wf_pe = pos_err, wf_ae = ang_err;
    if (wfrac_margin > 0) {
      int pick = loc.wfracGateSelect(cands, beams, wfrac_margin);
      errOf(cands[pick], wf_pe, wf_ae);
      if (cand_dump) {
        for (size_t ci = 0; ci < cands.size(); ci++) {
          double pe, ae;
          errOf(cands[ci], pe, ae);
          fprintf(cand_dump, "%s,%d,%zu,%.6g,%.6f,%.4f,%.2f\n", cond, trial, ci,
            cands[ci].score, loc.wallMissFraction(cands[ci], beams), pe, ae);
        }
      }
    }

    printf("%s,%d,%.3f,%.3f,%.1f,%.3f,%.3f,%.1f,%.3f,%.1f,%.5f,%.3f,%d,%.4f,%.3f,%.1f\n",
      cond, trial, gx, gy, gt_deg, cands[0].x, cands[0].y,
      cands[0].yaw * 180.0 / M_PI, pos_err, ang_err, cands[0].score, margin,
      pool_hit, elapsed, wf_pe, wf_ae);
    fflush(stdout);
  }
  fclose(f);
  if (cand_dump) fclose(cand_dump);

  if (stage_dump && n_scans > 0) {
    double n = n_scans;
    fprintf(stderr,
      "\n[stage] 1 スキャンあたり [ms] (n=%d)\n"
      "  repr  %7.2f  スキャン -> 壁画像・法線\n"
      "  xform %7.2f  テンプレートの構築\n"
      "  prep  %7.2f  角度ごとの canvas 回転 + 順 DFT\n"
      "  corr  %7.2f  周波数域の積 + 逆 DFT\n"
      "  peak  %7.2f  スコア場の走査・ピーク抽出\n"
      "  fine  %7.2f  細段 refine\n"
      "  合計  %7.2f\n",
      n_scans, acc.repr / n, acc.xform / n, acc.prep / n, acc.corr / n,
      acc.peak / n, acc.fine / n, acc.total() / n);
  }
  return 0;
}
