// /initialpose の再送予算の管理と、受領確認による打ち切り。
//
// /initialpose を 1 回きり volatile で出すと、購読側 (AMCL) の activate が
// こちらの初回ロックより後の場合に黙って失われる (実測 6 秒後だった。
// docs/simulation.md)。そのため一定回数まで再送するのだが、再送は無料ではない:
// AMCL は受け取るたびにパーティクルを撒き直すので、**1 回の再シード判断が
// repeat 回の撒き直しに増幅される** (実測: 追跡喪失 4 回が撒き直し 24 回。
// docs/amcl_supervision.md)。撒き直しは姿勢の跳びを生み、跳びは経路を無効にする。
//
// そこで **AMCL が受け取ったことを姿勢で確認したら残りの再送を止める**。
// AMCL は initialpose を受けると次のスキャン更新でその近傍の姿勢を出すので、
// シード (をオドメトリで現在まで運んだもの) との不一致がしきい値の中に入った
// 観測が届いた時点で「受領した」とみなせる。位置と角度の両方を要求する:
// 位置だけ合っていて角度が大きく違う姿勢は、こちらのシードを反映していない。
//
// ROS には依存しない。再送の周期駆動と姿勢の伝播はノード側 (ofl_node.cpp)。
#ifndef ORIENTED_FIELD_LOCALIZATION__SEED_HANDOFF_HPP_
#define ORIENTED_FIELD_LOCALIZATION__SEED_HANDOFF_HPP_

namespace oriented_field_localization
{

struct SeedHandoffParams
{
  int repeat = 5;         ///< 送る回数の上限 (初回を含む)
  double wait_s = 60.0;   ///< 購読者が現れるのを待つ上限 [s]
  double ack_m = 0.5;     ///< 受領判定: シードとの距離がこれ以下 [m]
  double ack_deg = 20.0;  ///< 同 [deg]。0 で角度は見ない
};

/// 1 つのシードの再送予算。start() -> wantRepeat() / observe() の順で使う。
class SeedHandoff
{
public:
  SeedHandoff() = default;
  explicit SeedHandoff(const SeedHandoffParams & p)
  : p_(p) {}

  const SeedHandoffParams & params() const {return p_;}

  /// 新しいシードの初回 publish の直後に呼ぶ。前のシードの残りは捨てる。
  void start(double now_s)
  {
    left_ = p_.repeat - 1;
    started_s_ = now_s;
  }

  /// 再送が残っているか。
  bool pending() const {return left_ > 0;}

  /// 周期ティック。今再送すべきなら true を返して予算を 1 消費する。
  /// **購読者が居ない間は予算を消費しない** (出しても失われるだけ)。
  /// ただし wait_s 待っても現れなければ諦める。
  bool wantRepeat(bool has_subscriber, double now_s)
  {
    if (left_ <= 0) return false;
    if (!has_subscriber) {
      if (now_s - started_s_ > p_.wait_s) left_ = 0;
      return false;
    }
    left_--;
    return true;
  }

  /// シード後に観測した相手の姿勢との不一致を入れる。受領とみなせるなら
  /// 残りの再送を捨てて true を返す。
  bool observe(double disagree_m, double disagree_deg)
  {
    if (left_ <= 0) return false;
    if (disagree_m > p_.ack_m) return false;
    if (p_.ack_deg > 0 && disagree_deg > p_.ack_deg) return false;
    left_ = 0;
    return true;
  }

  /// 残りの再送を捨てる。
  void cancel() {left_ = 0;}

private:
  SeedHandoffParams p_;
  int left_ = 0;
  double started_s_ = 0.0;
};

}  // namespace oriented_field_localization

#endif  // ORIENTED_FIELD_LOCALIZATION__SEED_HANDOFF_HPP_
