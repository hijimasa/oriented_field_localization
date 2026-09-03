// ROS 非依存の不変条件テスト。
// 合成地図を作り、既知姿勢のスキャンをレイキャストで作って探索させる。
#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <cmath>
#include <cstdio>
#include <future>
#include <stdexcept>
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

struct GT { double x, y, yaw; };

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
  {
    MatcherParams bad = p;
    bad.match_resolution = 0.0;
    bool rejected = false;
    try {OrientedFieldLocalizer invalid(bad);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "ゼロの match_resolution を fail-fast で棄却する");
    bad = p;
    bad.pyramid_levels = 100;
    rejected = false;
    try {OrientedFieldLocalizer invalid(bad);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "極端な pyramid_levels を fail-fast で棄却する");
  }

  printf("== 地図の設定 ==\n");
  OrientedFieldLocalizer loc(p);
  check(!loc.hasMap(), "地図を入れる前は hasMap() が false");
  check(loc.localize({}).empty(), "地図が無ければ候補を返さない");
  loc.setMap(map, res, ox, oy);
  check(loc.hasMap(), "setMap 後は hasMap() が true");
  {
    cv::Mat yaw_map(100, 100, CV_8UC1, cv::Scalar(255));
    // local map (2, 3) の壁は origin yaw=90deg により world (7, 22) へ写る。
    yaw_map.at<uint8_t>(99 - 30, 20) = 0;
    OrientedFieldLocalizer yaw_loc(p);
    yaw_loc.setMap(yaw_map, 0.1, 10.0, 20.0, M_PI / 2.0);
    PoseCandidate origin_pose;
    origin_pose.x = 10.0;
    origin_pose.y = 20.0;
    origin_pose.yaw = M_PI / 2.0;
    const std::vector<Beam> one_wall{{std::hypot(2.0, 3.0), std::atan2(3.0, 2.0)}};
    check(yaw_loc.wallMissFraction(origin_pose, one_wall) == 0.0,
      "origin yaw を world->map 変換へ適用する");
  }

  printf("== 既知姿勢の復元 ==\n");
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
    const auto st = loc.lastStageTimes();
    check(st.total() > 0, "段階別の所要時間が記録される");
  }

  printf("== TRACK (局所探索) ==\n");
  {
    int recovered = 0, in_window = 0;
    for (const GT & g : gts) {
      const std::vector<Beam> beams =
        castScan(map, res, ox, oy, g.x, g.y, g.yaw, p.max_range);
      // 事前姿勢を窓の内側で崩す (オドメトリ誤差の模擬)
      PoseCandidate prior;
      prior.x = g.x + 0.8;
      prior.y = g.y - 0.6;
      prior.yaw = g.yaw + 12.0 * M_PI / 180.0;
      const std::vector<PoseCandidate> cands = loc.track(beams, prior);
      if (cands.empty()) continue;
      const double pe = std::hypot(cands[0].x - g.x, cands[0].y - g.y);
      double ae = std::fabs((cands[0].yaw - g.yaw) * 180.0 / M_PI);
      while (ae > 180) ae = std::fabs(ae - 360);
      if (pe < 0.5 && ae < 10.0) recovered++;
      // 返る解は必ず探索窓の内側にある
      const double dp = std::hypot(cands[0].x - prior.x, cands[0].y - prior.y);
      double da = std::fabs((cands[0].yaw - prior.yaw) * 180.0 / M_PI);
      while (da > 180) da = std::fabs(da - 360);
      if (dp <= p.track_search_m + 0.5 &&
        da <= p.track_angle_window_deg + p.refine_angle_window + 1) { in_window++; }
    }
    check(recovered == static_cast<int>(gts.size()),
      "窓の内側の事前姿勢から全姿勢を復元する (" + std::to_string(recovered) + "/" +
      std::to_string(gts.size()) + ")");
    check(in_window == static_cast<int>(gts.size()),
      "TRACK が返す解は探索窓の内側にある");

    // 誤ロックの検出に使う量: 事前姿勢に依らないこと。
    // (跳び判定は事前姿勢からの相対量なので、誤ロックが自分の事前姿勢と整合すると
    //  検出できない。WFRAC は事前姿勢を一切参照しないのでその役に立つ)
    {
      const std::vector<Beam> b =
        castScan(map, res, ox, oy, gts[0].x, gts[0].y, gts[0].yaw, p.max_range);
      const std::vector<PoseCandidate> c0 = loc.localize(b);
      PoseCandidate wrong = c0[0];
      wrong.x += 4.0;
      wrong.yaw += 1.0;
      const double wf_ok = loc.wallMissFraction(c0[0], b);
      const double wf_ng = loc.wallMissFraction(wrong, b);
      check(wf_ok < 0.35 && wf_ng > 0.35,
        "正解姿勢と誤姿勢が既定のしきい値 0.35 で分離する (" +
        std::to_string(wf_ok).substr(0, 5) + " vs " +
        std::to_string(wf_ng).substr(0, 5) + ")");
    }

    // 地図の外を事前姿勢に与えたら空を返す
    const std::vector<Beam> beams =
      castScan(map, res, ox, oy, gts[0].x, gts[0].y, gts[0].yaw, p.max_range);
    PoseCandidate far_prior;
    far_prior.x = 1e4;
    far_prior.y = 1e4;
    far_prior.yaw = 0.0;
    check(loc.track(beams, far_prior).empty(), "地図の外の事前姿勢では空を返す");
    check(loc.track({}, far_prior).empty(), "ビームが無ければ空を返す");
  }

  printf("== 同一 instance への並行探索 ==\n");
  {
    const GT & g = gts[0];
    const std::vector<Beam> beams =
      castScan(map, res, ox, oy, g.x, g.y, g.yaw, p.max_range);
    PoseCandidate prior;
    prior.x = g.x + 0.4;
    prior.y = g.y - 0.3;
    prior.yaw = g.yaw + 5.0 * M_PI / 180.0;
    auto global_future = std::async(std::launch::async, [&loc, &beams]() {
        return loc.localize(beams);
      });
    auto track_future = std::async(std::launch::async, [&loc, &beams, prior]() {
        return loc.track(beams, prior);
      });
    const auto global = global_future.get();
    const auto tracked = track_future.get();
    check(!global.empty() && !tracked.empty(),
      "localize() と track() を同一 instance に並行呼び出せる");
    check(loc.lastStageTimes().total() > 0.0,
      "並行呼び出し後も完了した探索の計測 snapshot を返す");
  }

  printf("\n=====================================\n");
  printf("  通過 %d / 失敗 %d\n", g_pass, g_fail);
  printf("=====================================\n");
  return g_fail == 0 ? 0 : 1;
}
