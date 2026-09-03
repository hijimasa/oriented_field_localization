// /initialpose 再送予算 (seed_handoff.hpp) の単体テスト。ROS も OpenCV も要らない。
//
// テストの主眼は 2 つ:
//   1. **購読者が居ない間は予算を消費しない** (出しても失われるだけ。
//      実測で AMCL の activate は初回ロックの 6 秒後だった)
//   2. **受領を確認したら残りを止める** (1 回の再シード判断が repeat 回の
//      撒き直しに増幅されるのを断つ。docs/amcl_supervision.md の限界に
//      挙がっていた項目)
#include "oriented_field_localization/seed_handoff.hpp"

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

/// 1 秒周期のティックを n 回回し、再送した回数を返す。
int tick(SeedHandoff & h, bool has_sub, int n, double t0 = 0.0)
{
  int sent = 0;
  for (int i = 0; i < n; i++) {
    if (h.wantRepeat(has_sub, t0 + i)) sent++;
  }
  return sent;
}

}  // namespace

int main()
{
  SeedHandoffParams p;   // 既定値 (repeat 5 / wait 60 s / 0.5 m / 20 deg)

  printf("\n-- start 前 --\n");
  {
    SeedHandoff h(p);
    check(!h.pending(), "start 前は再送予算が無い");
    check(tick(h, true, 10) == 0, "start 前はティックしても何も出ない");
    check(!h.observe(0.0, 0.0), "start 前の観測は受領にならない");
  }

  printf("\n-- 再送予算 --\n");
  {
    SeedHandoff h(p);
    h.start(0.0);
    check(h.pending(), "start 直後は再送が残っている");
    check(tick(h, true, 100, 1.0) == p.repeat - 1,
      "購読者が居れば初回を含めて repeat 回で打ち止めになる");
    check(!h.pending(), "予算を使い切ったら pending でなくなる");
  }

  printf("\n-- 購読者が居ない間は予算を消費しない --\n");
  {
    SeedHandoff h(p);
    h.start(0.0);
    check(tick(h, false, 30, 1.0) == 0, "購読者が現れるまで再送しない");
    check(h.pending(), "その間、予算は減らない");
    // 実測で AMCL の activate は 6 秒後だった: 遅れて現れたら残り全部を出せる
    check(tick(h, true, 100, 31.0) == p.repeat - 1,
      "wait_s 以内に購読者が現れたら残りの予算をそこから出す");
  }
  {
    SeedHandoff h(p);
    h.start(0.0);
    tick(h, false, static_cast<int>(p.wait_s) + 2, 1.0);
    check(!h.pending(), "wait_s 待っても購読者が現れなければ諦める");
    check(tick(h, true, 10, 100.0) == 0, "諦めた後に購読者が現れても出さない");
  }

  printf("\n-- 受領確認 (増幅の停止) --\n");
  {
    // AMCL がシードを反映した姿勢を出してきた: 残りの再送は撒き直しの
    // 増幅にしかならないので止める
    SeedHandoff h(p);
    h.start(0.0);
    check(h.wantRepeat(true, 1.0), "受領前は再送する");
    check(h.observe(0.1, 3.0), "シード近傍の観測で受領になる");
    check(!h.pending() && tick(h, true, 100, 2.0) == 0,
      "受領したら残りの再送を止める (1 回の判断が repeat 回に増幅されない)");
  }
  {
    // AMCL がまだ壊れたまま (kidnap 位置の姿勢を出し続けている): 再送は続ける
    SeedHandoff h(p);
    h.start(0.0);
    check(!h.observe(6.0, 0.0), "遠い観測は受領にならない");
    check(h.pending(), "受領していないので再送は続く");
  }
  {
    // 位置だけ合っていて角度が大きく違う: こちらのシードを反映していない
    SeedHandoff h(p);
    h.start(0.0);
    check(!h.observe(0.1, 90.0), "角度が違う観測は受領にならない");
  }
  {
    SeedHandoffParams noyaw = p;
    noyaw.ack_deg = 0.0;
    SeedHandoff h(noyaw);
    h.start(0.0);
    check(h.observe(0.1, 179.0), "ack_deg = 0 なら角度は見ない");
  }
  {
    // 予算を使い切った後の観測は何もしない (受領扱いにもならない)
    SeedHandoff h(p);
    h.start(0.0);
    tick(h, true, 100, 1.0);
    check(!h.observe(0.0, 0.0), "予算が尽きた後の観測は受領にならない");
  }

  printf("\n-- 再 start とキャンセル --\n");
  {
    SeedHandoff h(p);
    h.start(0.0);
    check(h.observe(0.1, 0.0), "(前提) 1 本目は受領済み");
    h.start(10.0);
    check(h.pending() && tick(h, true, 100, 11.0) == p.repeat - 1,
      "新しいシードは受領状態を引き継がず、予算が回復する");
  }
  {
    SeedHandoff h(p);
    h.start(0.0);
    h.cancel();
    check(!h.pending() && tick(h, true, 10, 1.0) == 0, "cancel() で残りを捨てる");
  }

  printf("\n=====================================\n");
  printf("  通過 %d / 失敗 %d\n", g_pass, g_fail);
  printf("=====================================\n");
  return g_fail == 0 ? 0 : 1;
}
