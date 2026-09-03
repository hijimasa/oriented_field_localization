// 外部の位置推定 (AMCL 等) を監視して、壊れているときだけ再シードを促す判定。
//
// **なぜ WFRAC の絶対値では判定できないか。**スキャン点が地図の壁に載らない割合
// (WFRAC) は事前姿勢に依存しない幾何量なので誤ロックで跳ね上がるが、その正常値は
// 環境に依存する。地図に無い動く障害物が視野に入ると正常時の WFRAC そのものが
// 0.50 近くまで上がり、静的な世界で較正した 0.35 では区別がつかなくなる
// (docs/nav2_closed_loop.md の実測)。
//
// そこで**同じスキャンで測った自分自身の WFRAC を基準線として使い、その差を見る**。
// 障害物や地図のずれは両方の姿勢に同じだけ乗るので、差を取ると相殺される。
// 誤ロックしているのは相手だけなので、差にははっきり出る。
//
// 再シードは無料ではない。/initialpose はパーティクルを撒き直すので、それ自体が
// 姿勢の跳びを生み、局所 costmap が odom フレームにある以上は経路が無効になって
// follow_path が abort する (実測 2.5 秒の停止。docs/nav2_closed_loop.md)。
// **誤発火のコストは検出の遅れより高い**ので、判定は連続回数と最小間隔で鈍らせる。
#ifndef ORIENTED_FIELD_LOCALIZATION__RESEED_POLICY_HPP_
#define ORIENTED_FIELD_LOCALIZATION__RESEED_POLICY_HPP_

#include <limits>

namespace oriented_field_localization
{

struct ReseedPolicyParams
{
  double max_wfrac = 0.35;          ///< 相手の WFRAC がこれ以下なら健全とみなす
  double wfrac_excess = 0.15;       ///< 自分の WFRAC からの超過分の下限
  double min_disagreement_m = 0.5;  ///< 姿勢がこれ以上離れていなければ再シードしない [m]
  double min_disagreement_deg = 20.0;  ///< 同 [deg]。0 で無効
  int after_scans = 10;             ///< 証拠がこの回数連続したら発火する
  double min_interval_s = 10.0;     ///< 再シードの最小間隔 [s]
};

/// 1 スキャンぶんの証拠。
struct ReseedEvidence
{
  double wfrac_other = 0.0;   ///< 相手の姿勢で測った WFRAC
  double wfrac_self = 0.0;    ///< **同じスキャンで**自分の姿勢を測った WFRAC (基準線)
  double disagree_m = 0.0;    ///< 2 つの姿勢の距離 [m]
  double disagree_deg = 0.0;  ///< 2 つの姿勢の角度差 [deg]
};

/// 姿勢が離れているか。**再シードは相手が間違っているだけでは足りず、
/// 自分と違っていることが要る。**同じ場所を指しているなら、撒き直しても
/// 直るものが無く、跳びだけが残る。
inline bool reseedPosesDisagree(const ReseedEvidence & e, const ReseedPolicyParams & p)
{
  return (p.min_disagreement_m > 0 && e.disagree_m > p.min_disagreement_m) ||
         (p.min_disagreement_deg > 0 && e.disagree_deg > p.min_disagreement_deg);
}

/// 相手の姿勢が地図と整合していないか。絶対値と、自分を基準にした超過の両方を要求する。
inline bool reseedGeometryBad(const ReseedEvidence & e, const ReseedPolicyParams & p)
{
  return e.wfrac_other > p.max_wfrac &&
         e.wfrac_other - e.wfrac_self > p.wfrac_excess;
}

/// 連続回数と最小間隔で鈍らせた発火判定。
/// **自分自身が信用できないスキャンでは update() を呼ばず reset() する**こと
/// (自分の姿勢が基準線なので、それが怪しいときの差には意味が無い)。
class ReseedPolicy
{
public:
  ReseedPolicy() = default;
  explicit ReseedPolicy(const ReseedPolicyParams & p)
  : p_(p) {}

  const ReseedPolicyParams & params() const {return p_;}

  /// 証拠を 1 スキャン分入れる。発火すべきときだけ true を返し、内部状態を進める。
  bool update(const ReseedEvidence & e, double now_s)
  {
    if (!(reseedGeometryBad(e, p_) && reseedPosesDisagree(e, p_))) {
      hits_ = 0;
      return false;
    }
    hits_++;
    if (hits_ < p_.after_scans) return false;
    // **証拠は溜めたまま間隔だけ待つ。**ここで hits_ を捨てると、間隔の間に
    // 証拠が消えたことにしてしまい、間隔明けにまた最初から数え直しになる。
    if (now_s - last_fire_s_ < p_.min_interval_s) return false;
    last_fire_s_ = now_s;
    hits_ = 0;
    return true;
  }

  /// 証拠を捨てる (自分が棄却した / 追跡していないスキャン)。
  void reset() {hits_ = 0;}

  /// 外部で再シードしたことを知らせ、最小間隔を共有する。
  void noteFired(double now_s)
  {
    last_fire_s_ = now_s;
    hits_ = 0;
  }

  int hits() const {return hits_;}

private:
  ReseedPolicyParams p_;
  int hits_ = 0;
  double last_fire_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace oriented_field_localization

#endif  // ORIENTED_FIELD_LOCALIZATION__RESEED_POLICY_HPP_
