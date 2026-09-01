// ROS 非依存の不変条件テスト。
// 合成地図を作り、既知姿勢のスキャンをレイキャストで作って探索させる。
#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace oriented_field_localization;

namespace
{
int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string & what)
{
  (ok ? g_pass : g_fail)++;
  printf("  [%s] %s\n", ok ? "ok" : "NG", what.c_str());
}

/// 部屋・斜めの間仕切り・什器を持つ合成地図 (占有 0 / 自由 255)。
/// **点対称にならないように非対称なクラッタを置いてある**: 対称な地図では
/// 180 度回した姿勢が同じ観測を与えるので、top-1 を検査する意味が無くなる。
cv::Mat makeMap(int n = 600, double res = 0.05)
{
  cv::Mat m(n, n, CV_8UC1, cv::Scalar(255));
  const int t = std::max(1, static_cast<int>(std::lround(0.15 / res)));
  cv::rectangle(m, cv::Rect(0, 0, n, n), cv::Scalar(0), t);
  cv::rectangle(m, cv::Rect(n / 6, n / 6, n / 3, n / 3), cv::Scalar(0), t);
  cv::line(m, cv::Point(n / 2, n / 6), cv::Point(n - n / 6, n / 2), cv::Scalar(0), t);
  cv::line(m, cv::Point(n / 6, n - n / 4), cv::Point(n - n / 6, n - n / 4), cv::Scalar(0), t);
  cv::circle(m, cv::Point(n / 2, n - n / 3), 3 * t, cv::Scalar(0), -1);
  cv::circle(m, cv::Point(n / 4, n / 2), 2 * t, cv::Scalar(0), -1);
  // 非対称化: 片側だけに什器と斜め壁を足す
  cv::rectangle(m, cv::Rect(n - n / 4, n - n / 6, n / 8, n / 12), cv::Scalar(0), -1);
  cv::rectangle(m, cv::Rect(n / 12, n / 2 + n / 8, n / 10, n / 16), cv::Scalar(0), -1);
  cv::line(m, cv::Point(n - n / 5, n / 8), cv::Point(n - n / 12, n / 3), cv::Scalar(0), t);
  cv::line(m, cv::Point(n / 3, n - n / 8), cv::Point(n / 2, n - n / 6), cv::Scalar(0), t);
  cv::circle(m, cv::Point(n - n / 3, n / 2 + n / 10), 2 * t, cv::Scalar(0), -1);
  return m;
}

/// 既知姿勢から 360 本のレイキャストを撃つ (地図と同じ座標規約)。
std::vector<Beam> castScan(
  const cv::Mat & map, double res, double ox, double oy,
  double rx, double ry, double yaw, double max_range)
{
  std::vector<Beam> beams;
  const int nb = 360;
  const double step = res * 0.5;
  const int max_steps = static_cast<int>(max_range / step);
  for (int i = 0; i < nb; i++) {
    const double a = 2.0 * M_PI * i / nb;
    const double c = std::cos(a + yaw), s = std::sin(a + yaw);
    double hit = max_range, x = rx, y = ry;
    for (int k = 1; k <= max_steps; k++) {
      x += c * step; y += s * step;
      const int px = static_cast<int>((x - ox) / res);
      const int py = map.rows - 1 - static_cast<int>((y - oy) / res);
      if (px < 0 || px >= map.cols || py < 0 || py >= map.rows) break;
      if (map.at<uint8_t>(py, px) < 89) { hit = k * step; break; }
    }
    beams.push_back({hit, a});
  }
  return beams;
}

}  // namespace

int main()
{
  const double res = 0.05, ox = -5.0, oy = -5.0;
  const cv::Mat map = makeMap(600, res);

  MatcherParams p;
  p.match_resolution = 0.05;
  p.max_range = 8.0;
  p.margin_pixels = p.recommendedMargin();
  p.pyramid_levels = 3;
  p.coarse_angle_step = 6;

  printf("== パラメータの不変条件 ==\n");
  check(p.recommendedMargin() >= p.minimumMargin(),
    "推奨マージンは下限以上");
  check(p.minimumMargin() == static_cast<int>(std::ceil(p.max_range / p.match_resolution)),
    "下限マージン = ceil(max_range / match_resolution)");

  printf("== 地図の設定 ==\n");
  OrientedFieldLocalizer loc(p);
  check(!loc.hasMap(), "地図を入れる前は hasMap() が false");
  check(loc.localize({}).empty(), "地図が無ければ候補を返さない");
  loc.setMap(map, res, ox, oy);
  check(loc.hasMap(), "setMap 後は hasMap() が true");

  printf("== 既知姿勢の復元 ==\n");
  struct GT { double x, y, yaw; };
  const std::vector<GT> gts = {
    {2.0, 2.0, 0.0}, {-1.5, 3.0, 1.2}, {4.0, -2.0, -2.5}, {0.5, -3.5, 3.0},
  };
  int recovered = 0;
  for (const GT & g : gts) {
    const std::vector<Beam> beams =
      castScan(map, res, ox, oy, g.x, g.y, g.yaw, p.max_range);
    const std::vector<PoseCandidate> cands = loc.localize(beams);
    if (cands.empty()) continue;
    const double pe = std::hypot(cands[0].x - g.x, cands[0].y - g.y);
    double ae = std::fabs((cands[0].yaw - g.yaw) * 180.0 / M_PI);
    while (ae > 180) ae = std::fabs(ae - 360);
    if (pe < 0.5 && ae < 10.0) recovered++;
  }
  check(recovered == static_cast<int>(gts.size()),
    "外乱の無いスキャンから全姿勢を 0.5 m / 10 度以内で復元する (" +
    std::to_string(recovered) + "/" + std::to_string(gts.size()) + ")");

  printf("== 候補プールの性質 ==\n");
  {
    const std::vector<Beam> beams = castScan(map, res, ox, oy, 2.0, 2.0, 0.0, p.max_range);
    const std::vector<PoseCandidate> cands = loc.localize(beams);
    check(!cands.empty(), "候補が返る");
    check(static_cast<int>(cands.size()) <= p.candidate_pool_size,
      "候補数は candidate_pool_size 以下");
    bool sorted = true;
    for (size_t i = 1; i < cands.size(); i++) {
      if (cands[i].score > cands[i - 1].score) sorted = false;
    }
    check(sorted, "候補はスコア降順");
    bool separated = true;
    for (size_t i = 0; i < cands.size(); i++) {
      for (size_t j = i + 1; j < cands.size(); j++) {
        double d = std::hypot(cands[i].x - cands[j].x, cands[i].y - cands[j].y);
        double ad = std::fabs((cands[i].yaw - cands[j].yaw) * 180.0 / M_PI);
        while (ad > 180) ad = std::fabs(ad - 360);
        if (d < 0.5 * p.nms_separation_m && ad < p.nms_separation_deg) separated = false;
      }
    }
    check(separated, "NMS が候補を分離している");

    printf("== WFRAC (幾何審判) ==\n");
    const double wf_true = loc.wallMissFraction(cands[0], beams);
    PoseCandidate wrong = cands[0];
    wrong.x += 3.0;
    wrong.y -= 2.5;
    const double wf_wrong = loc.wallMissFraction(wrong, beams);
    check(wf_true >= 0.0 && wf_true <= 1.0, "wfrac は 0..1");
    check(wf_true < wf_wrong, "正解姿勢の wfrac は誤姿勢より小さい");
    check(loc.wfracGateSelect(cands, beams, 0.0) == 0, "しきい値 0 なら top-1 のまま");
    // 拮抗していなければ (margin >= しきい値) 触らない
    if (cands.size() > 1 && cands[1].score > 0 &&
      cands[0].score / cands[1].score >= 1.0)
    {
      check(loc.wfracGateSelect(cands, beams, 1.0) == 0,
        "margin >= しきい値なら top-1 のまま");
    }

    printf("== 段階別の計測 ==\n");
    const auto & st = loc.lastStageTimes();
    check(st.total() > 0, "段階別の所要時間が記録される");
  }

  printf("\n=====================================\n");
  printf("  通過 %d / 失敗 %d\n", g_pass, g_fail);
  printf("=====================================\n");
  return g_fail == 0 ? 0 : 1;
}
