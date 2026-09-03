// AMCL 監視の判定 (reseed_policy.hpp) の単体テスト。ROS も OpenCV も要らない。
//
// 数値は docs/nav2_closed_loop.md / docs/simulation.md の実測から取っている:
//   正常時の WFRAC        中央 0.000 (静的) / 0.50 近くまで上がる (動く障害物あり)
//   誤ロック時の WFRAC    0.74
// **絶対値だけでは動く障害物と誤ロックを分離できない**ことがテストの主眼である。
#include "oriented_field_localization/reseed_policy.hpp"

#include <cstdio>
#include <string>

using namespace oriented_field_localization;

namespace
{
int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string & what)
{
  (ok ? g_pass : g_fail)++;
  printf("  [%s] %s\n", ok ? "ok" : "NG", what.c_str());
}

ReseedEvidence ev(double other, double self, double m, double deg = 0.0)
{
  ReseedEvidence e;
  e.wfrac_other = other;
  e.wfrac_self = self;
  e.disagree_m = m;
  e.disagree_deg = deg;
  return e;
}

/// 証拠を n スキャン分入れて、発火した回数を返す。
int feed(ReseedPolicy & pol, const ReseedEvidence & e, int n, double t0 = 0.0,
  double dt = 0.1)
{
  int fired = 0;
  for (int i = 0; i < n; i++) {
    if (pol.update(e, t0 + dt * i)) fired++;
  }
  return fired;
}

}  // namespace

int main()
{
  ReseedPolicyParams p;   // 既定値 (0.35 / 0.15 / 0.5 m / 20 deg / 10 回 / 10 s)

  printf("\n-- 発火する場合 (誤ロック) --\n");
  {
    // kidnap 後の AMCL: 地図と全く整合せず、こちらの解から遠い
    const auto lost = ev(0.74, 0.05, 6.0);
    ReseedPolicy pol(p);
    check(feed(pol, lost, p.after_scans - 1) == 0,
      "連続回数に届くまでは発火しない (誤検出のコストが高いため)");
    check(pol.update(lost, 1.0), "連続回数に達したら発火する");
    check(pol.hits() == 0, "発火したら証拠はリセットされる");
  }

  printf("\n-- 発火しない場合 --\n");
  {
    // 動く障害物: 正常時の WFRAC そのものが 0.50 まで上がるが、**両方に同じだけ
    // 乗る**ので差は出ない。絶対値 0.35 だけで判定すると誤発火する条件である
    const auto crowd = ev(0.50, 0.48, 6.0);
    ReseedPolicy pol(p);
    check(crowd.wfrac_other > p.max_wfrac, "(前提) 絶対値のしきい値は超えている");
    check(feed(pol, crowd, 50) == 0,
      "地図に無い障害物で両方の WFRAC が上がっても発火しない (差で見るため)");
  }
  {
    // 相手の姿勢は地図と整合していないが、こちらと同じ場所を指している。
    // 撒き直しても直るものが無く、跳びだけが残る
    const auto agree = ev(0.74, 0.05, 0.2, 5.0);
    ReseedPolicy pol(p);
    check(feed(pol, agree, 50) == 0, "姿勢が一致しているなら再シードしない");
  }
  {
    const auto healthy = ev(0.10, 0.05, 6.0);
    ReseedPolicy pol(p);
    check(feed(pol, healthy, 50) == 0, "相手が地図と整合していれば離れていても発火しない");
  }

  printf("\n-- 角度だけの不一致 --\n");
  {
    const auto yawed = ev(0.74, 0.05, 0.1, 45.0);
    ReseedPolicy pol(p);
    check(feed(pol, yawed, p.after_scans) == 1,
      "位置が同じでも角度が離れていれば発火する");
  }

  printf("\n-- 証拠の連続性 --\n");
  {
    const auto lost = ev(0.74, 0.05, 6.0);
    const auto healthy = ev(0.10, 0.05, 6.0);
    ReseedPolicy pol(p);
    feed(pol, lost, p.after_scans - 1);
    check(pol.update(healthy, 1.0) == false && pol.hits() == 0,
      "証拠が 1 回途切れたら数え直しになる");
    check(feed(pol, lost, p.after_scans - 1, 2.0) == 0, "途切れた後は再び連続回数が要る");
  }
  {
    const auto lost = ev(0.74, 0.05, 6.0);
    ReseedPolicy pol(p);
    feed(pol, lost, p.after_scans - 1);
    pol.reset();
    check(pol.hits() == 0, "reset() で証拠が消える (自分が棄却したスキャン)");
    check(feed(pol, lost, p.after_scans - 1, 2.0) == 0, "reset() の後も連続回数が要る");
  }

  printf("\n-- 最小間隔 --\n");
  {
    const auto lost = ev(0.74, 0.05, 6.0);
    ReseedPolicy pol(p);
    // 1 回目は t=0 付近で発火する
    check(feed(pol, lost, p.after_scans, 0.0) == 1, "1 回目は発火する");
    // 間隔が明けるまでは、証拠が続いても発火しない
    check(feed(pol, lost, 50, 1.0) == 0, "最小間隔の間は発火しない (撒き直しの連発を防ぐ)");
    // 間隔が明けた最初のスキャンで発火する (証拠を溜めたまま待っているので、
    // 明けてから数え直しにはならない)
    check(pol.update(lost, p.min_interval_s + 1.0), "間隔が明けたら次のスキャンで発火する");
  }
  {
    const auto lost = ev(0.74, 0.05, 6.0);
    ReseedPolicy pol(p);
    pol.noteFired(0.0);      // 起動時の引き継ぎで /initialpose を出した想定
    check(feed(pol, lost, 50, 1.0) == 0, "外部の再シードとも最小間隔を共有する");
  }

  printf("\n-- 判定の分解 --\n");
  {
    check(reseedGeometryBad(ev(0.74, 0.05, 0.0), p), "誤ロックは幾何の判定に掛かる");
    check(!reseedGeometryBad(ev(0.50, 0.48, 0.0), p), "障害物だけでは幾何の判定に掛からない");
    check(!reseedGeometryBad(ev(0.34, 0.00, 0.0), p),
      "差が十分でも絶対値がしきい値以下なら掛からない");
    check(reseedPosesDisagree(ev(0, 0, 0.6, 0.0), p), "位置だけで離れていると判定できる");
    check(!reseedPosesDisagree(ev(0, 0, 0.4, 19.0), p), "どちらも小さければ一致とみなす");
  }

  printf("\n=====================================\n");
  printf("  通過 %d / 失敗 %d\n", g_pass, g_fail);
  printf("=====================================\n");
  return g_fail == 0 ? 0 : 1;
}
