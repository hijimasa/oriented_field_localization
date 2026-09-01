// 画像空間の向き付き場 (壁質量 + 観測側を向く単位法線) を FFT 相関させて、
// 2D LiDAR スキャンを占有格子地図へ合わせる大域位置推定。
//
// スコア (粗段・細段で共通):
//
//   s(p, alpha) = < f_map , R_alpha f_scan >_p / sqrt(E_scan)
//     f      = ( mass, lam*mass*nx, lam*mass*ny )   画像座標 (y 下向き)
//     E_scan = テンプレート側のエネルギー総和 (全候補で共通)
//
// **分母をテンプレート側だけにしてあるのが要点**である。候補位置ごとの地図側
// エネルギーで割ると (コサイン正規化)、候補ごとに参照信号を作り直すのと同じ
// ことになり、候補間の比較可能性が壊れて順位付けが落ちる (docs/benchmark.md)。
//
// 探索は画像ピラミッドの粗密探索:
//   粗段  全 alpha (coarse_angle_step 刻み) x 全位置を FFT 相関し、
//         角度ごとに peaks_per_angle 個のピークを NMS 付きで保持する
//   細段  候補ごとに ±refine_angle_window 度 x ±refine_search_range px を
//         疎な点列の直接相関で精密化し、NMS で刈る
//
// ROS には依存しない。ノードは src/ofl_node.cpp。
#ifndef ORIENTED_FIELD_LOCALIZATION__ORIENTED_FIELD_MATCHER_HPP_
#define ORIENTED_FIELD_LOCALIZATION__ORIENTED_FIELD_MATCHER_HPP_

#include <opencv2/opencv.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace oriented_field_localization
{

/// 1 本のビーム (ロボット座標)。range >= max_range は無反射として捨てる。
struct Beam
{
  double range;    ///< [m]
  double bearing;  ///< [rad] ロボット x 軸から反時計回り
};

/// 探索が返す候補姿勢 (地図のワールド座標)。
struct PoseCandidate
{
  double x = 0.0;      ///< [m]
  double y = 0.0;      ///< [m]
  double yaw = 0.0;    ///< [rad]
  double score = 0.0;  ///< 相関スコア (候補間で比較可能)
};

struct MatcherParams
{
  double match_resolution = 0.05;    ///< マッチング格子の解像度 [m/px]
  double max_range = 10.0;           ///< テンプレートに使う最大レンジ [m]
  double min_range = 0.0;            ///< 至近レンジゲート [m]。0 で無効
  int margin_pixels = 200;           ///< 地図外周のゼロパディング [px]
  int pyramid_levels = 3;            ///< 粗密探索の段数 (1 で単一スケール)
  int coarse_angle_step = 6;         ///< 粗段の角度刻み [deg]。360 の約数
  int peaks_per_angle = 8;           ///< 粗段で角度ごとに保持するピーク数
  int candidate_pool_size = 15;      ///< 最終的に残す候補数
  int intermediate_pool_size = 8;    ///< 中間段で残す候補数
  int refine_angle_window = 2;       ///< 細段の角度窓 [deg]
  double refine_search_m = 0.2;      ///< 細段の位置窓 [m]
  double nms_separation_m = 2.0;     ///< 候補 NMS の位置間隔 [m]
  int nms_separation_deg = 15;       ///< 候補 NMS の角度間隔 [deg]
  double normal_weight = 1.0;        ///< 法線チャネルの重み lam。0 で壁質量のみ
  double mass_weight = 1.0;          ///< 質量チャネルの重み
  double map_normal_sigma = 3.0;     ///< 地図法線を作る自由空間マスクのぼかし [px]
  double wfrac_tolerance_m = 0.10;   ///< WFRAC の基本許容 [m]

  // ---- TRACK (事前姿勢の周りだけを探す局所探索) ----
  double track_search_m = 3.0;       ///< 事前姿勢からの位置探索半径 [m]
  int track_angle_window_deg = 30;   ///< 事前姿勢からの角度探索幅 [deg]

  /// margin_pixels がテンプレートを切らないための下限。
  int minimumMargin() const;
  /// 相関の外挿が 0 で済む十分条件 (これを満たすと DFT が最小になる)。
  int recommendedMargin() const;
};

/// 地図側の前計算を保持し、スキャンごとの探索を提供する。
/// setMap() は起動時に 1 回だけ呼ぶ (重い)。localize() はスレッドセーフ。
class OrientedFieldLocalizer
{
public:
  explicit OrientedFieldLocalizer(const MatcherParams & params = MatcherParams());

  /// 地図を設定して前計算する。map_img は map_server 規約のグレースケール
  /// (占有 < 89、自由 > 205、行 0 が上端 = 最大 y)。
  void setMap(
    const cv::Mat & map_img, double map_resolution, double origin_x, double origin_y);

  bool hasMap() const;
  const MatcherParams & params() const;

  /// 大域探索。スコア降順の候補を最大 candidate_pool_size 個返す。
  std::vector<PoseCandidate> localize(const std::vector<Beam> & beams) const;

  /// 局所探索 (TRACK)。事前姿勢 `prior` の周り
  /// (±track_search_m、±track_angle_window_deg) だけを探す。
  /// 粗段を FFT ではなく疎な点列の直接相関で走査するので、位置の候補数が
  /// 地図全体の数百分の一になる分そのまま速い。細段は localize() と共通。
  /// 事前姿勢が地図の外なら空を返す。
  std::vector<PoseCandidate> track(
    const std::vector<Beam> & beams, const PoseCandidate & prior) const;

  /// 候補姿勢でスキャン点を地図に重ね、壁に載らない点の割合を返す (0..1)。
  double wallMissFraction(const PoseCandidate & pose, const std::vector<Beam> & beams) const;

  /// 拮抗帯 (score[0]/score[1] < margin_thresh) でだけ WFRAC で選び直す。
  /// 拮抗していなければ 0 (= top-1 のまま) を返す。
  int wfracGateSelect(
    const std::vector<PoseCandidate> & cands, const std::vector<Beam> & beams,
    double margin_thresh) const;

  /// 診断用: 1 スキャンあたりの段階別所要時間 [ms] (localize() が更新する)。
  struct StageTimes
  {
    double repr = 0;   ///< スキャン -> 壁画像・法線
    double xform = 0;  ///< テンプレート canvas / 点列の構築
    double prep = 0;   ///< 角度ごとの canvas 回転 + 順 DFT
    double corr = 0;   ///< 周波数域の積 + 逆 DFT
    double peak = 0;   ///< スコア場の走査・ピーク抽出
    double fine = 0;   ///< 細段 refine
    double total() const { return repr + xform + prep + corr + peak + fine; }
  };
  const StageTimes & lastStageTimes() const;

  ~OrientedFieldLocalizer();
  OrientedFieldLocalizer(OrientedFieldLocalizer &&) noexcept;
  OrientedFieldLocalizer & operator=(OrientedFieldLocalizer &&) noexcept;
  OrientedFieldLocalizer(const OrientedFieldLocalizer &) = delete;
  OrientedFieldLocalizer & operator=(const OrientedFieldLocalizer &) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace oriented_field_localization

#endif  // ORIENTED_FIELD_LOCALIZATION__ORIENTED_FIELD_MATCHER_HPP_
