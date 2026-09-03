#include "oriented_field_localization/oriented_field_matcher.hpp"

#ifdef _OPENMP
#include <omp.h>
#else
static inline int omp_get_max_threads() { return 1; }
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace oriented_field_localization
{
namespace
{

inline double nowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 実画像を fh x fw のゼロ詰め領域の左上へ置いて DFT (CCS パック) を取る。
cv::Mat specOf(const cv::Mat & src, int fw, int fh)
{
  cv::Mat pad = cv::Mat::zeros(fh, fw, CV_32F);
  src.copyTo(pad(cv::Rect(0, 0, src.cols, src.rows)));
  cv::Mat sp;
  cv::dft(pad, sp, 0, src.rows);
  return sp;
}

cv::Mat correlate(const cv::Mat & img_ext, const cv::Mat & tpl)
{
  cv::Mat r;
  cv::matchTemplate(img_ext, tpl, r, cv::TM_CCORR);
  return r;
}

/// 地図法線: 自由空間マスクの勾配 (自由側を向く)。両側が自由な薄壁では勾配が
/// 立たず、そこは法線なし = 質量チャネルだけに退化する。
void mapNormals(
  const cv::Mat & map_img, const cv::Mat & wall_u8, double sigma,
  cv::Mat & nx, cv::Mat & ny)
{
  cv::Mat freem = map_img > 205, f32, gx, gy;
  freem.convertTo(f32, CV_32F, 1.0 / 255.0);
  cv::GaussianBlur(f32, f32, cv::Size(0, 0), sigma);
  cv::Sobel(f32, gx, CV_32F, 1, 0, 3);
  cv::Sobel(f32, gy, CV_32F, 0, 1, 3);
  cv::Mat gxs, gys;
  cv::resize(gx, gxs, wall_u8.size(), 0, 0, cv::INTER_AREA);
  cv::resize(gy, gys, wall_u8.size(), 0, 0, cv::INTER_AREA);
  nx = cv::Mat::zeros(wall_u8.size(), CV_32F);
  ny = cv::Mat::zeros(wall_u8.size(), CV_32F);
  for (int y = 0; y < wall_u8.rows; y++) {
    const uint8_t * wr = wall_u8.ptr<uint8_t>(y);
    const float * ar = gxs.ptr<float>(y);
    const float * br = gys.ptr<float>(y);
    float * ox = nx.ptr<float>(y);
    float * oy = ny.ptr<float>(y);
    for (int x = 0; x < wall_u8.cols; x++) {
      if (!wr[x]) continue;
      float a = ar[x], b = br[x];
      float m = std::sqrt(a * a + b * b);
      if (m < 1e-3f) continue;
      ox[x] = a / m;
      oy[x] = b / m;
    }
  }
}

/// 法線を粗段の格子へ縮約する (壁質量で重み付けた平均。再正規化はしない)。
/// sc == 1 では wf を掛けて wf で割り戻すだけなので恒等写像になる。
void downNormals(
  const cv::Mat & w8, const cv::Mat & nx, const cv::Mat & ny, int sc,
  cv::Mat & nxc, cv::Mat & nyc)
{
  if (sc <= 1) { nxc = nx; nyc = ny; return; }
  cv::Mat wf;
  w8.convertTo(wf, CV_32F, 1.0 / 255.0);
  cv::Mat wnx = nx.mul(wf), wny = ny.mul(wf), wc, wnxc, wnyc;
  cv::resize(wf, wc, cv::Size(), 1.0 / sc, 1.0 / sc, cv::INTER_AREA);
  cv::resize(wnx, wnxc, wc.size(), 0, 0, cv::INTER_AREA);
  cv::resize(wny, wnyc, wc.size(), 0, 0, cv::INTER_AREA);
  nxc = cv::Mat::zeros(wc.size(), CV_32F);
  nyc = cv::Mat::zeros(wc.size(), CV_32F);
  for (int y = 0; y < wc.rows; y++) {
    const float * wr = wc.ptr<float>(y);
    const float * xr = wnxc.ptr<float>(y);
    const float * yr = wnyc.ptr<float>(y);
    float * ox = nxc.ptr<float>(y);
    float * oy = nyc.ptr<float>(y);
    for (int x = 0; x < wc.cols; x++) {
      float ww = wr[x];
      if (ww > 1e-6f) { ox[x] = xr[x] / ww; oy[x] = yr[x] / ww; }
    }
  }
}

}  // namespace

int MatcherParams::minimumMargin() const
{
  return static_cast<int>(std::ceil(max_range / match_resolution));
}

int MatcherParams::recommendedMargin() const
{
  return static_cast<int>(std::ceil(std::sqrt(2.0) * max_range / match_resolution)) + 1;
}

// =============================================================================
// 内部表現
// =============================================================================

/// 地図側の 1 段。
struct Level
{
  int scale = 1;              ///< L0 に対する縮小率
  double res = 0.05;          ///< この段の解像度 [m/px]
  int margin = 0;             ///< この段のゼロパディング [px]
  int map_w = 0, map_h = 0;   ///< パディング前の大きさ
  int w = 0, h = 0;           ///< パディング後の大きさ
  int R = 0, D = 0;           ///< footprint 半径 [px] と canvas 一辺 (2R+1)
  cv::Mat M, X, Y;            ///< パディング後の場 (質量・質量*法線)
  cv::Mat Ea;                 ///< footprint 内の地図側エネルギー和
  int fw = 0, fh = 0;         ///< DFT の大きさ
  int ext = 0;                ///< 相関用に外へ広げた幅 [px]
  int off = 0;                ///< 相関結果の索引ずれ (= R - ext)
  cv::Mat MF, XF, YF;         ///< CCS パックの実 DFT (地図側、起動時に 1 回)
};

/// テンプレート (スキャン) の 1 段。
struct Tpl
{
  cv::Mat M, X, Y;                                  ///< D x D。粗段のみ
  double Eb = 0;                                    ///< テンプレート側エネルギー
  std::vector<float> px, py, m, nx, ny;             ///< 細段用の疎な点列
};

/// 内部の候補 (この段の画素座標)。
struct Cand
{
  int angle = 0;
  int u = 0, v = 0;
  double score = 0;
};

struct OrientedFieldLocalizer::Impl
{
  MatcherParams p;
  bool has_map = false;
  double map_res = 0.05;
  double origin_x = 0.0, origin_y = 0.0, origin_yaw = 0.0;
  int map_rows_l0 = 0;
  cv::Mat map_image;      ///< 原解像度のグレースケール
  cv::Mat wall_dt;        ///< 占有画素への距離変換 [px]
  std::vector<Level> levels;
  StageTimes stage;
  mutable std::mutex search_mutex;  ///< 探索とその計測値を同一 instance 内で直列化

  // ---- スキャン -> 壁二値グリッド (壁 255、背景 0、3x3 dilate) ----
  cv::Mat scanToGrid(const std::vector<Beam> & beams) const
  {
    std::vector<std::pair<double, double>> walls;
    for (const Beam & b : beams) {
      if (b.range < p.min_range) continue;
      if (b.range >= p.max_range - 0.01) continue;
      walls.push_back({b.range * std::cos(b.bearing), b.range * std::sin(b.bearing)});
    }
    if (walls.empty()) return cv::Mat(101, 101, CV_8UC1, cv::Scalar(0));
    double extent = 0;
    for (auto & [x, y] : walls) extent = std::max(extent, std::max(std::abs(x), std::abs(y)));
    int gs = static_cast<int>(2 * extent / p.match_resolution) + 20;
    if (gs % 2 == 0) gs++;
    int c = gs / 2;
    cv::Mat grid(gs, gs, CV_8UC1, cv::Scalar(0));
    for (auto & [x, y] : walls) {
      int gx = c + static_cast<int>(x / p.match_resolution);
      int gy = c - static_cast<int>(y / p.match_resolution);
      if (gx >= 0 && gx < gs && gy >= 0 && gy < gs) grid.at<uint8_t>(gy, gx) = 255;
    }
    cv::Mat dil;
    cv::dilate(grid, dil, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    return dil;
  }

  // ---- ビーム法線 (隣接ビームの接線に垂直、センサ側を向く) ----
  void beamNormals(
    const std::vector<Beam> & beams, const std::vector<int> & vidx,
    std::vector<std::pair<double, double>> & out) const
  {
    out.assign(beams.size(), {0.0, 0.0});
    const int n = static_cast<int>(vidx.size());
    std::vector<double> X(n), Y(n);
    for (int k = 0; k < n; k++) {
      const Beam & b = beams[vidx[k]];
      X[k] = b.range * std::cos(b.bearing);
      Y[k] = b.range * std::sin(b.bearing);
    }
    for (int k = 0; k < n; k++) {
      int i = vidx[k];
      double px = X[k], py = Y[k];
      double ax = px, ay = py, bx = px, by = py;
      if (k >= 1 && (i - vidx[k - 1]) <= 3 &&
        std::hypot(X[k - 1] - px, Y[k - 1] - py) < 0.6) { ax = X[k - 1]; ay = Y[k - 1]; }
      if (k + 1 < n && (vidx[k + 1] - i) <= 3 &&
        std::hypot(X[k + 1] - px, Y[k + 1] - py) < 0.6) { bx = X[k + 1]; by = Y[k + 1]; }
      double dx = bx - ax, dy = by - ay, dn = std::hypot(dx, dy);
      if (dn < 1e-6) continue;
      double nxv = -dy / dn, nyv = dx / dn;
      if (nxv * px + nyv * py > 0) { nxv = -nxv; nyv = -nyv; }
      out[i] = {nxv, nyv};
    }
  }

  // ---- スキャン法線チャネル (壁グリッドと同じ格子、3x3 ブロックで書く) ----
  void scanNormals(
    const std::vector<Beam> & beams, const cv::Mat & grid, cv::Mat & nx, cv::Mat & ny) const
  {
    int gs = grid.cols, c = gs / 2;
    std::vector<int> vidx;
    for (size_t i = 0; i < beams.size(); i++) {
      if (beams[i].range >= p.min_range && beams[i].range < p.max_range - 0.01) {
        vidx.push_back(static_cast<int>(i));
      }
    }
    cv::Mat snx(gs, gs, CV_32F, cv::Scalar(0)), sny(gs, gs, CV_32F, cv::Scalar(0));
    cv::Mat cnt(gs, gs, CV_32F, cv::Scalar(0));
    std::vector<std::pair<double, double>> bn;
    beamNormals(beams, vidx, bn);
    for (int i : vidx) {
      const Beam & b = beams[i];
      double px = b.range * std::cos(b.bearing), py = b.range * std::sin(b.bearing);
      double nxv = bn[i].first, nyv = bn[i].second;
      int gx = c + static_cast<int>(px / p.match_resolution);
      int gy = c - static_cast<int>(py / p.match_resolution);
      for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {
          int Xp = gx + ox, Yp = gy + oy;
          if (Xp < 0 || Xp >= gs || Yp < 0 || Yp >= gs) continue;
          snx.at<float>(Yp, Xp) += static_cast<float>(nxv);
          sny.at<float>(Yp, Xp) += static_cast<float>(-nyv);   // y 下向きへ
          cnt.at<float>(Yp, Xp) += 1;
        }
      }
    }
    nx = cv::Mat::zeros(gs, gs, CV_32F);
    ny = cv::Mat::zeros(gs, gs, CV_32F);
    for (int Y = 0; Y < gs; Y++) {
      const float * cr = cnt.ptr<float>(Y);
      const float * sxr = snx.ptr<float>(Y);
      const float * syr = sny.ptr<float>(Y);
      const uint8_t * gr = grid.ptr<uint8_t>(Y);
      float * oxr = nx.ptr<float>(Y);
      float * oyr = ny.ptr<float>(Y);
      for (int X = 0; X < gs; X++) {
        float n = cr[X];
        if (n > 0 && gr[X]) {
          float a = sxr[X] / n, b = syr[X] / n;
          float m = std::sqrt(a * a + b * b);
          if (m > 1e-3f) { oxr[X] = a / m; oyr[X] = b / m; }
        }
      }
    }
  }

  // ---- 候補の重複除去 ----
  std::vector<Cand> nms(std::vector<Cand> cands, int sep_px, int sep_deg, int keep) const
  {
    std::sort(cands.begin(), cands.end(),
      [](const Cand & a, const Cand & b) { return a.score > b.score; });
    if (sep_px <= 0) {
      if (static_cast<int>(cands.size()) > keep) cands.resize(keep);
      return cands;
    }
    std::vector<Cand> out;
    for (const Cand & c : cands) {
      bool dup = false;
      for (const Cand & o : out) {
        double d = std::hypot(static_cast<double>(c.u - o.u), static_cast<double>(c.v - o.v));
        int ad = std::abs(c.angle - o.angle);
        ad = std::min(ad, 360 - ad);
        if (d < sep_px && ad < sep_deg) { dup = true; break; }
      }
      if (!dup) out.push_back(c);
      if (static_cast<int>(out.size()) >= keep) break;
    }
    return out;
  }

  /// canvas を alpha [deg] 回す。画像座標 (y 下向き) なので正の alpha は
  /// 表示上の反時計回り = ワールドの yaw と同じ向きになる。位置も値も回す。
  static void rotateTpl(
    const Tpl & t, double alpha_deg, cv::Mat & rm, cv::Mat & rx, cv::Mat & ry)
  {
    int D = t.M.cols;
    cv::Mat Rm = cv::getRotationMatrix2D(
      cv::Point2f((D - 1) / 2.f, (D - 1) / 2.f), alpha_deg, 1.0);
    cv::Mat wx, wy;
    cv::warpAffine(t.M, rm, Rm, cv::Size(D, D), cv::INTER_LINEAR,
      cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::warpAffine(t.X, wx, Rm, cv::Size(D, D), cv::INTER_LINEAR,
      cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::warpAffine(t.Y, wy, Rm, cv::Size(D, D), cv::INTER_LINEAR,
      cv::BORDER_CONSTANT, cv::Scalar(0));
    double th = -alpha_deg * CV_PI / 180.0;
    double c = std::cos(th), s = std::sin(th);
    rx = wx * c - wy * s;
    ry = wx * s + wy * c;
  }

  /// テンプレートの点列を alpha [deg] 回した整数格子上の点列にする。
  /// 画像座標 (y 下向き) なので、位置も法線も同じ回転行列で回る。
  static void rotatePoints(
    const Tpl & T, int alpha, std::vector<int> & rpx, std::vector<int> & rpy,
    std::vector<float> & rm, std::vector<float> & rnx, std::vector<float> & rny)
  {
    const size_t np = T.px.size();
    const double th = alpha * CV_PI / 180.0;
    const double ca = std::cos(th), sa = std::sin(th);
    rpx.resize(np); rpy.resize(np); rm.resize(np); rnx.resize(np); rny.resize(np);
    for (size_t i = 0; i < np; i++) {
      rpx[i] = static_cast<int>(std::lround(ca * T.px[i] + sa * T.py[i]));
      rpy[i] = static_cast<int>(std::lround(-sa * T.px[i] + ca * T.py[i]));
      rm[i] = T.m[i];
      rnx[i] = static_cast<float>(ca * T.nx[i] + sa * T.ny[i]);
      rny[i] = static_cast<float>(-sa * T.nx[i] + ca * T.ny[i]);
    }
  }

  /// 回した点列を (u, v) に置いたときの相関 (正規化前)。
  static double numeratorAt(
    const Level & L, const std::vector<int> & rpx, const std::vector<int> & rpy,
    const std::vector<float> & rm, const std::vector<float> & rnx,
    const std::vector<float> & rny, int u, int v, double mass_w, double lam2)
  {
    double num = 0;
    const size_t np = rpx.size();
    for (size_t i = 0; i < np; i++) {
      const int X = u + rpx[i], Y = v + rpy[i];
      if (X < 0 || X >= L.w || Y < 0 || Y >= L.h) continue;
      num += mass_w * L.M.at<float>(Y, X) * rm[i] +
        lam2 * (L.X.at<float>(Y, X) * rnx[i] + L.Y.at<float>(Y, X) * rny[i]);
    }
    return num;
  }

  // ---- 粗段: 全 alpha x 全位置を FFT 相関 ----
  std::vector<Cand> coarseSweep(const Tpl & TC)
  {
    const int kc = static_cast<int>(levels.size()) - 1;
    const Level & LC = levels[kc];
    const double lam2 = p.normal_weight * p.normal_weight;
    std::vector<int> alphas;
    for (int a = 0; a < 360; a += p.coarse_angle_step) alphas.push_back(a);
    const int nms_px_c = p.nms_separation_m > 0.0
      ? std::max(4, static_cast<int>(std::lround(p.nms_separation_m / LC.res))) : 0;
    std::vector<std::vector<Cand>> per_angle(alphas.size());
    const double inv_eb_c = 1.0 / std::sqrt(TC.Eb);
    const int u0 = LC.margin, u1 = LC.margin + LC.map_w;
    const int v0 = LC.margin, v1 = LC.margin + LC.map_h;
    const int sw = u1 - u0, sh = v1 - v0;
    double s_prep = 0, s_corr = 0, s_peak = 0;

#pragma omp parallel reduction(+ : s_prep, s_corr, s_peak)
    {
      // 作業領域はスレッドごとに 1 回だけ確保・ゼロ埋めする (canvas の外は常にゼロ)
      cv::Mat pad = cv::Mat::zeros(LC.fh, LC.fw, CV_32F);
      cv::Mat rm, rx, ry, fm, fx, fy, pm, pxs, pys, acc, num;
      std::vector<float> sm;
      auto specInto = [&](const cv::Mat & src, cv::Mat & dst) {
          src.copyTo(pad(cv::Rect(0, 0, LC.D, LC.D)));
          cv::dft(pad, dst, 0, LC.D);
        };
#pragma omp for schedule(dynamic)
      for (int ai = 0; ai < static_cast<int>(alphas.size()); ai++) {
        const double a0 = nowSec();
        rotateTpl(TC, alphas[ai], rm, rx, ry);
        specInto(rm, fm);
        specInto(rx, fx);
        specInto(ry, fy);
        const double a1 = nowSec();
        cv::mulSpectrums(LC.MF, fm, pm, 0, true);
        cv::mulSpectrums(LC.XF, fx, pxs, 0, true);
        cv::mulSpectrums(LC.YF, fy, pys, 0, true);
        cv::addWeighted(pm, p.mass_weight, pxs, lam2, 0.0, acc);
        cv::scaleAdd(pys, lam2, acc, acc);
        cv::dft(acc, num, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
        const double a2 = nowSec();

        sm.assign(static_cast<size_t>(sw) * sh, -2.0f);
        for (int v = v0; v < v1; v++) {
          const float * nr = num.ptr<float>(v - LC.off);
          const float * er = LC.Ea.ptr<float>(v - LC.off);
          float * dr = &sm[static_cast<size_t>(v - v0) * sw];
          for (int u = u0; u < u1; u++) {
            if (er[u - LC.off] < 1e-6f) continue;   // footprint に地図が無い位置
            dr[u - u0] = static_cast<float>(nr[u - LC.off] * inv_eb_c);
          }
        }
        per_angle[ai] = greedyPeaks(sm, sw, sh, u0, v0, alphas[ai], nms_px_c);
        s_prep += a1 - a0;
        s_corr += a2 - a1;
        s_peak += nowSec() - a2;
      }
    }
    {
      // スレッド合計を並列度で割り、他の段 (壁時計) と同じ尺度へ揃える
      const double par = static_cast<double>(omp_get_max_threads());
      stage.prep = 1000 * s_prep / par;
      stage.corr = 1000 * s_corr / par;
      stage.peak = 1000 * s_peak / par;
    }
    std::vector<Cand> cands;
    for (auto & v : per_angle) cands.insert(cands.end(), v.begin(), v.end());
    return nms(std::move(cands), nms_px_c, p.nms_separation_deg, p.candidate_pool_size);
  }

  // ---- TRACK: 事前姿勢の周りだけを直接相関で走査する ----
  // 粗段の窓 (±track_search_m, ±track_angle_window_deg) を疎な点列で総当たりする。
  // 位置の候補数が地図全体の数百分の一なので、FFT を使うより直接評価が安い。
  std::vector<Cand> localSweep(const Tpl & TC, const PoseCandidate & prior)
  {
    const int kc = static_cast<int>(levels.size()) - 1;
    const Level & LC = levels[kc];
    const double lam2 = p.normal_weight * p.normal_weight;
    const double inv_eb = 1.0 / std::sqrt(TC.Eb);

    // 事前姿勢を粗段の画素へ落とす (toWorld の逆写像)
    const double dx = prior.x - origin_x, dy = prior.y - origin_y;
    const double co = std::cos(origin_yaw), so = std::sin(origin_yaw);
    const double local_x = co * dx + so * dy;
    const double local_y = -so * dx + co * dy;
    const double px = local_x / LC.res;
    const double py = LC.map_h - 1 - local_y / LC.res;
    const int cu = static_cast<int>(std::lround(px)) + LC.margin;
    const int cv_ = static_cast<int>(std::lround(py)) + LC.margin;
    int prior_deg = static_cast<int>(
      std::lround((prior.yaw - origin_yaw) * 180.0 / CV_PI));
    prior_deg = ((prior_deg % 360) + 360) % 360;

    const int SR = std::max(1, static_cast<int>(std::lround(p.track_search_m / LC.res)));
    std::vector<int> alphas;
    for (int d = -p.track_angle_window_deg; d <= p.track_angle_window_deg;
      d += p.coarse_angle_step)
    {
      alphas.push_back(((prior_deg + d) % 360 + 360) % 360);
    }
    const int nms_px = p.nms_separation_m > 0.0
      ? std::max(2, static_cast<int>(std::lround(p.nms_separation_m / LC.res))) : 0;

    const int u0 = std::max(LC.margin, cu - SR);
    const int u1 = std::min(LC.margin + LC.map_w, cu + SR + 1);
    const int v0 = std::max(LC.margin, cv_ - SR);
    const int v1 = std::min(LC.margin + LC.map_h, cv_ + SR + 1);
    if (u1 <= u0 || v1 <= v0) return {};    // 事前姿勢が地図の外
    const int sw = u1 - u0, sh = v1 - v0;

    std::vector<std::vector<Cand>> per_angle(alphas.size());
    const double t0 = nowSec();
#pragma omp parallel for schedule(dynamic)
    for (int ai = 0; ai < static_cast<int>(alphas.size()); ai++) {
      std::vector<int> rpx, rpy;
      std::vector<float> rm, rnx, rny;
      rotatePoints(TC, alphas[ai], rpx, rpy, rm, rnx, rny);
      std::vector<float> sm(static_cast<size_t>(sw) * sh, -2.0f);
      for (int v = v0; v < v1; v++) {
        for (int u = u0; u < u1; u++) {
          sm[static_cast<size_t>(v - v0) * sw + (u - u0)] = static_cast<float>(
            numeratorAt(LC, rpx, rpy, rm, rnx, rny, u, v, p.mass_weight, lam2) * inv_eb);
        }
      }
      per_angle[ai] = greedyPeaks(sm, sw, sh, u0, v0, alphas[ai], nms_px);
    }
    stage.corr = 1000 * (nowSec() - t0);

    std::vector<Cand> cands;
    for (auto & v : per_angle) cands.insert(cands.end(), v.begin(), v.end());
    return nms(std::move(cands), nms_px, p.nms_separation_deg, p.candidate_pool_size);
  }

  /// スコア場から貪欲に argmax + 抑制でピークを拾う。
  std::vector<Cand> greedyPeaks(
    std::vector<float> & sm, int sw, int sh, int u0, int v0, int alpha, int nms_px) const
  {
    std::vector<Cand> peaks;
    for (int k = 0; k < p.peaks_per_angle; k++) {
      size_t bi = 0;
      float bs = -1.5f;
      for (size_t i = 0; i < sm.size(); i++) {
        if (sm[i] > bs) { bs = sm[i]; bi = i; }
      }
      if (bs <= -1.5f) break;
      const int bu = u0 + static_cast<int>(bi % sw), bv = v0 + static_cast<int>(bi / sw);
      peaks.push_back({alpha, bu, bv, bs});
      const int r = std::max(1, nms_px);
      for (int v = std::max(0, bv - v0 - r); v < std::min(sh, bv - v0 + r + 1); v++) {
        for (int u = std::max(0, bu - u0 - r); u < std::min(sw, bu - u0 + r + 1); u++) {
          if (std::hypot(static_cast<double>(u + u0 - bu), static_cast<double>(v + v0 - bv))
            < nms_px)
          {
            sm[static_cast<size_t>(v) * sw + u] = -2.0f;
          }
        }
      }
    }
    return peaks;
  }

  // ---- 細段: 候補ごとに狭い角度・位置窓を直接相関で精密化する ----
  std::vector<Cand> refineDown(std::vector<Cand> cands, const std::vector<Tpl> & tpl)
  {
    const int kc = static_cast<int>(levels.size()) - 1;
    const double lam2 = p.normal_weight * p.normal_weight;
    const double t0 = nowSec();
    const int SR = std::max(1, static_cast<int>(std::lround(
        p.refine_search_m / p.match_resolution)));
    for (int kcur = kc - 1; kcur >= 0; kcur--) {
      const Level & L = levels[kcur];
      const Tpl & T = tpl[kcur];
      const int ratio = levels[kcur + 1].scale / L.scale;
      for (Cand & c : cands) {
        const double up = (c.u - levels[kcur + 1].margin + 0.5) * ratio - 0.5 + L.margin;
        const double vp = (c.v - levels[kcur + 1].margin + 0.5) * ratio - 0.5 + L.margin;
        c.u = static_cast<int>(std::lround(up));
        c.v = static_cast<int>(std::lround(vp));
      }
      const int n_per = 2 * p.refine_angle_window + 1;
      const int n_jobs = static_cast<int>(cands.size()) * n_per;
      std::vector<Cand> ref(n_jobs);
      const double inv_eb = 1.0 / std::sqrt(T.Eb);
#pragma omp parallel for schedule(dynamic)
      for (int job = 0; job < n_jobs; job++) {
        const int ci = job / n_per;
        const int alpha =
          ((cands[ci].angle + (job % n_per) - p.refine_angle_window) % 360 + 360) % 360;
        std::vector<int> rpx, rpy;
        std::vector<float> rm, rnx, rny;
        rotatePoints(T, alpha, rpx, rpy, rm, rnx, rny);
        Cand best{alpha, cands[ci].u, cands[ci].v, -2.0};
        for (int dv = -SR; dv <= SR; dv++) {
          for (int du = -SR; du <= SR; du++) {
            const int u = cands[ci].u + du, v = cands[ci].v + dv;
            if (u < L.margin || u >= L.margin + L.map_w ||
              v < L.margin || v >= L.margin + L.map_h) continue;
            const double s =
              numeratorAt(L, rpx, rpy, rm, rnx, rny, u, v, p.mass_weight, lam2) * inv_eb;
            if (s > best.score) { best.u = u; best.v = v; best.score = s; }
          }
        }
        ref[job] = best;
      }
      const int keep = (kcur == 0) ? p.candidate_pool_size : p.intermediate_pool_size;
      const int nms_px = p.nms_separation_m > 0.0
        ? std::max(4, static_cast<int>(std::lround(p.nms_separation_m / L.res))) : 0;
      cands = nms(std::move(ref), nms_px, p.nms_separation_deg, keep);
    }
    stage.fine = 1000 * (nowSec() - t0);
    return cands;
  }

  PoseCandidate toWorld(const Cand & c) const
  {
    const Level & L0 = levels[0];
    double px = c.u - L0.margin;
    double py = c.v - L0.margin;
    PoseCandidate out;
    const double local_x = px * p.match_resolution;
    const double local_y = (L0.map_h - 1 - py) * p.match_resolution;
    const double co = std::cos(origin_yaw), so = std::sin(origin_yaw);
    out.x = origin_x + co * local_x - so * local_y;
    out.y = origin_y + so * local_x + co * local_y;
    out.yaw = std::remainder(c.angle * CV_PI / 180.0 + origin_yaw, 2.0 * CV_PI);
    out.score = c.score;
    return out;
  }
};

// =============================================================================
// 公開 API
// =============================================================================

void validateMatcherParams(const MatcherParams & params)
{
  const auto finite = [](double v) {return std::isfinite(v);};
  if (!finite(params.match_resolution) || params.match_resolution <= 0.0) {
    throw std::invalid_argument("match_resolution must be finite and > 0");
  }
  if (!finite(params.max_range) || params.max_range <= 0.0) {
    throw std::invalid_argument("max_range must be finite and > 0");
  }
  if (!finite(params.min_range) || params.min_range < 0.0 || params.min_range >= params.max_range) {
    throw std::invalid_argument("min_range must be finite, >= 0, and < max_range");
  }
  if (params.margin_pixels < 0) throw std::invalid_argument("margin_pixels must be >= 0");
  if (params.pyramid_levels < 1 || params.pyramid_levels > 8) {
    throw std::invalid_argument("pyramid_levels must be in [1, 8]");
  }
  if (params.coarse_angle_step < 1 || params.coarse_angle_step > 180 ||
    360 % params.coarse_angle_step != 0)
  {
    throw std::invalid_argument("coarse_angle_step must divide 360 and be in [1, 180]");
  }
  if (params.peaks_per_angle < 1 || params.peaks_per_angle > 10000 ||
    params.candidate_pool_size < 1 || params.candidate_pool_size > 10000 ||
    params.intermediate_pool_size < 1 || params.intermediate_pool_size > 10000)
  {
    throw std::invalid_argument("candidate pool sizes must be in [1, 10000]");
  }
  if (params.refine_angle_window < 0 || params.refine_angle_window > 180 ||
    params.track_angle_window_deg < 0 || params.track_angle_window_deg > 180)
  {
    throw std::invalid_argument("angle windows must be in [0, 180]");
  }
  if (!finite(params.refine_search_m) || params.refine_search_m < 0.0 ||
    !finite(params.nms_separation_m) || params.nms_separation_m < 0.0 ||
    !finite(params.track_search_m) || params.track_search_m < 0.0)
  {
    throw std::invalid_argument("search and NMS distances must be finite and >= 0");
  }
  if (params.nms_separation_deg < 0 || params.nms_separation_deg > 180) {
    throw std::invalid_argument("nms_separation_deg must be in [0, 180]");
  }
  if (!finite(params.normal_weight) || params.normal_weight < 0.0 ||
    !finite(params.mass_weight) || params.mass_weight < 0.0 ||
    (params.normal_weight == 0.0 && params.mass_weight == 0.0))
  {
    throw std::invalid_argument("matcher weights must be finite and at least one must be > 0");
  }
  if (!finite(params.map_normal_sigma) || params.map_normal_sigma <= 0.0 ||
    !finite(params.wfrac_tolerance_m) || params.wfrac_tolerance_m < 0.0)
  {
    throw std::invalid_argument("map_normal_sigma must be > 0 and wfrac_tolerance_m >= 0");
  }
}

OrientedFieldLocalizer::OrientedFieldLocalizer(const MatcherParams & params)
: impl_(new Impl())
{
  validateMatcherParams(params);
  impl_->p = params;
}

OrientedFieldLocalizer::~OrientedFieldLocalizer() = default;
OrientedFieldLocalizer::OrientedFieldLocalizer(OrientedFieldLocalizer &&) noexcept = default;
OrientedFieldLocalizer & OrientedFieldLocalizer::operator=(
  OrientedFieldLocalizer &&) noexcept = default;

bool OrientedFieldLocalizer::hasMap() const { return impl_->has_map; }
const MatcherParams & OrientedFieldLocalizer::params() const { return impl_->p; }
OrientedFieldLocalizer::StageTimes OrientedFieldLocalizer::lastStageTimes() const
{
  std::lock_guard<std::mutex> lock(impl_->search_mutex);
  return impl_->stage;
}

void OrientedFieldLocalizer::setMap(
  const cv::Mat & map_img, double map_resolution, double origin_x, double origin_y,
  double origin_yaw)
{
  if (map_img.empty() || map_img.type() != CV_8UC1) {
    throw std::invalid_argument("map_img must be a non-empty CV_8UC1 image");
  }
  if (!std::isfinite(map_resolution) || map_resolution <= 0.0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) || !std::isfinite(origin_yaw))
  {
    throw std::invalid_argument("map resolution/origin must be finite and resolution > 0");
  }
  Impl & I = *impl_;
  const MatcherParams & p = I.p;
  I.has_map = false;
  I.map_image = map_img.clone();
  I.map_res = map_resolution;
  I.origin_x = origin_x;
  I.origin_y = origin_y;
  I.origin_yaw = origin_yaw;

  // WFRAC 用: 原解像度の占有画素への距離変換
  {
    cv::Mat occ = map_img < 89, freem;
    cv::bitwise_not(occ, freem);
    cv::distanceTransform(freem, I.wall_dt, cv::DIST_L2, 3);
  }

  // 壁二値化 (マッチング解像度、3x3 dilate)
  cv::Mat map_binary;
  double scale = map_resolution / p.match_resolution;
  cv::resize(map_img, map_binary, cv::Size(), scale, scale, cv::INTER_AREA);
  cv::Mat occ_thresh;
  cv::threshold(map_binary, occ_thresh, 89, 255, cv::THRESH_BINARY_INV);
  cv::Mat dil;
  cv::dilate(occ_thresh, dil,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::Mat map_wb(map_binary.size(), CV_8UC1, cv::Scalar(0));
  map_wb.setTo(255, dil > 0);
  I.map_rows_l0 = map_wb.rows;

  cv::Mat mnx, mny;
  mapNormals(map_img, map_wb, p.map_normal_sigma, mnx, mny);

  I.levels.assign(p.pyramid_levels, Level());
  for (int k = 0; k < p.pyramid_levels; k++) {
    int sc = 1;
    for (int s = 0; s < k; s++) sc *= 2;
    Level & L = I.levels[k];
    L.scale = sc;
    L.res = p.match_resolution * sc;
    L.margin = (k == 0) ? p.margin_pixels : std::max(2, p.margin_pixels / sc);

    cv::Mat lw;
    if (sc > 1) {
      cv::resize(map_wb, lw, cv::Size(), 1.0 / sc, 1.0 / sc, cv::INTER_AREA);
      cv::threshold(lw, lw, 0, 255, cv::THRESH_BINARY);
    } else {
      lw = map_wb;
    }
    cv::Mat lnx, lny;
    downNormals(map_wb, mnx, mny, sc, lnx, lny);
    if (lnx.size() != lw.size()) {
      cv::resize(lnx, lnx, lw.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(lny, lny, lw.size(), 0, 0, cv::INTER_NEAREST);
    }
    cv::Mat mass;
    lw.convertTo(mass, CV_32F, 1.0 / 255.0);
    L.map_w = lw.cols;
    L.map_h = lw.rows;
    int m = L.margin;
    cv::copyMakeBorder(mass, L.M, m, m, m, m, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::Mat xx = mass.mul(lnx), yy = mass.mul(lny);
    cv::copyMakeBorder(xx, L.X, m, m, m, m, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::copyMakeBorder(yy, L.Y, m, m, m, m, cv::BORDER_CONSTANT, cv::Scalar(0));
    L.w = L.M.cols;
    L.h = L.M.rows;
    L.R = static_cast<int>(std::ceil(p.max_range / L.res)) + 2;
    L.D = 2 * L.R + 1;

    // 相関用の外挿。センサ位置はパディング前の地図の内側にしか置かないので、
    // テンプレートが触る画素は既存の margin パディングでほぼ覆われている。
    // 追加で要るのは max(0, R - margin) だけ (実機の不変条件
    // margin_pixels >= ceil(max_range / match_resolution) が R-2 なので通常 2 px)。
    // ゼロ領域を削るだけなので相関の値は変わらず、DFT の一辺だけが縮む。
    int e = std::max(0, L.R - L.margin);
    L.ext = e;
    L.off = L.R - e;
    cv::Mat M_ext, X_ext, Y_ext;
    cv::copyMakeBorder(L.M, M_ext, e, e, e, e, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::copyMakeBorder(L.X, X_ext, e, e, e, e, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::copyMakeBorder(L.Y, Y_ext, e, e, e, e, cv::BORDER_CONSTANT, cv::Scalar(0));
    L.fw = cv::getOptimalDFTSize(M_ext.cols);
    L.fh = cv::getOptimalDFTSize(M_ext.rows);
    L.MF = specOf(M_ext, L.fw, L.fh);
    L.XF = specOf(X_ext, L.fw, L.fh);
    L.YF = specOf(Y_ext, L.fw, L.fh);

    // footprint 内の地図エネルギー (alpha にもスキャンにも依らない)。
    // スコアの分母には使わないが、探索範囲の妥当性判定に使う。
    double lam2 = p.normal_weight * p.normal_weight;
    cv::Mat energy = p.mass_weight * L.M.mul(L.M) + lam2 * (L.X.mul(L.X) + L.Y.mul(L.Y));
    cv::Mat e_ext, disc(L.D, L.D, CV_32F, cv::Scalar(0));
    cv::circle(disc, cv::Point(L.R, L.R), L.R, cv::Scalar(1.0), -1);
    cv::copyMakeBorder(energy, e_ext, e, e, e, e, cv::BORDER_CONSTANT, cv::Scalar(0));
    L.Ea = correlate(e_ext, disc);
  }
  I.has_map = true;
}
// ---- テンプレート構築を localize() と track() で共有する ----

namespace
{
/// 段ごとのテンプレート。D x D の canvas は粗段の FFT でしか使わないので、
/// 細段では確保もゼロ埋めもしない (点列だけで足りる)。
bool buildTemplates(
  const MatcherParams & p, const std::vector<Level> & levels, const cv::Mat & swb,
  const cv::Mat & snx, const cv::Mat & sny, std::vector<Tpl> & tpl)
{
  const int n_lv = static_cast<int>(levels.size());
  const int kc = n_lv - 1;
  const double lam2 = p.normal_weight * p.normal_weight;
  tpl.assign(n_lv, Tpl());
  for (int k = 0; k < n_lv; k++) {
    const Level & L = levels[k];
    const int sc = L.scale;
    cv::Mat lw;
    if (sc > 1) {
      cv::resize(swb, lw, cv::Size(), 1.0 / sc, 1.0 / sc, cv::INTER_AREA);
      cv::threshold(lw, lw, 0, 255, cv::THRESH_BINARY);
    } else {
      lw = swb;
    }
    cv::Mat lnx, lny;
    downNormals(swb, snx, sny, sc, lnx, lny);
    if (lnx.size() != lw.size()) {
      cv::resize(lnx, lnx, lw.size(), 0, 0, cv::INTER_NEAREST);
      cv::resize(lny, lny, lw.size(), 0, 0, cv::INTER_NEAREST);
    }
    cv::Mat mass;
    lw.convertTo(mass, CV_32F, 1.0 / 255.0);
    Tpl & T = tpl[k];
    const bool need_canvas = (k == kc);
    if (need_canvas) {
      T.M = cv::Mat::zeros(L.D, L.D, CV_32F);
      T.X = cv::Mat::zeros(L.D, L.D, CV_32F);
      T.Y = cv::Mat::zeros(L.D, L.D, CV_32F);
    }
    const int c = lw.cols / 2;
    const int R = L.R;
    double eb = 0;
    std::vector<cv::Point> pts;
    cv::findNonZero(lw, pts);
    T.px.reserve(pts.size()); T.py.reserve(pts.size());
    T.m.reserve(pts.size()); T.nx.reserve(pts.size()); T.ny.reserve(pts.size());
    for (const cv::Point & q : pts) {
      const int cx = q.x - c + R, cy = q.y - c + R;
      if (cx < 0 || cx >= L.D || cy < 0 || cy >= L.D) continue;
      const float mm = mass.at<float>(q.y, q.x);
      if (mm <= 0) continue;
      const float bx = mm * lnx.at<float>(q.y, q.x);
      const float by = mm * lny.at<float>(q.y, q.x);
      if (need_canvas) {
        T.M.at<float>(cy, cx) = mm;
        T.X.at<float>(cy, cx) = bx;
        T.Y.at<float>(cy, cx) = by;
      }
      T.px.push_back(static_cast<float>(cx - R));
      T.py.push_back(static_cast<float>(cy - R));
      T.m.push_back(mm);
      T.nx.push_back(bx);
      T.ny.push_back(by);
      eb += p.mass_weight * static_cast<double>(mm) * mm +
        lam2 * (static_cast<double>(bx) * bx + static_cast<double>(by) * by);
    }
    T.Eb = eb;
    if (T.Eb < 1e-9 || T.px.empty()) return false;
  }
  return true;
}
}  // namespace

std::vector<PoseCandidate> OrientedFieldLocalizer::localize(
  const std::vector<Beam> & beams) const
{
  Impl & I = *impl_;
  std::lock_guard<std::mutex> lock(I.search_mutex);
  I.stage = StageTimes();
  if (!I.has_map || beams.empty()) return {};
  const int kc = static_cast<int>(I.levels.size()) - 1;

  double t0 = nowSec();
  cv::Mat swb = I.scanToGrid(beams);
  cv::Mat snx, sny;
  I.scanNormals(beams, swb, snx, sny);
  I.stage.repr = 1000 * (nowSec() - t0);

  t0 = nowSec();
  std::vector<Tpl> tpl;
  const bool ok = buildTemplates(I.p, I.levels, swb, snx, sny, tpl);
  I.stage.xform = 1000 * (nowSec() - t0);
  if (!ok) return {};

  std::vector<Cand> cands = I.coarseSweep(tpl[kc]);
  cands = I.refineDown(std::move(cands), tpl);

  std::vector<PoseCandidate> out;
  out.reserve(cands.size());
  for (const Cand & c : cands) out.push_back(I.toWorld(c));
  return out;
}

std::vector<PoseCandidate> OrientedFieldLocalizer::track(
  const std::vector<Beam> & beams, const PoseCandidate & prior) const
{
  Impl & I = *impl_;
  std::lock_guard<std::mutex> lock(I.search_mutex);
  I.stage = StageTimes();
  if (!I.has_map || beams.empty()) return {};
  const int kc = static_cast<int>(I.levels.size()) - 1;

  double t0 = nowSec();
  cv::Mat swb = I.scanToGrid(beams);
  cv::Mat snx, sny;
  I.scanNormals(beams, swb, snx, sny);
  I.stage.repr = 1000 * (nowSec() - t0);

  t0 = nowSec();
  std::vector<Tpl> tpl;
  const bool ok = buildTemplates(I.p, I.levels, swb, snx, sny, tpl);
  I.stage.xform = 1000 * (nowSec() - t0);
  if (!ok) return {};

  std::vector<Cand> cands = I.localSweep(tpl[kc], prior);
  if (cands.empty()) return {};
  cands = I.refineDown(std::move(cands), tpl);

  std::vector<PoseCandidate> out;
  out.reserve(cands.size());
  for (const Cand & c : cands) out.push_back(I.toWorld(c));
  return out;
}


double OrientedFieldLocalizer::wallMissFraction(
  const PoseCandidate & pose, const std::vector<Beam> & beams) const
{
  const Impl & I = *impl_;
  if (!I.has_map || I.wall_dt.empty()) return 1.0;
  const MatcherParams & p = I.p;
  double ca = std::cos(pose.yaw), sa = std::sin(pose.yaw);
  int n = 0, missed = 0;
  for (const Beam & b : beams) {
    if (b.range < p.min_range || b.range >= p.max_range - 0.01) continue;
    double sx = b.range * std::cos(b.bearing), sy = b.range * std::sin(b.bearing);
    double X = pose.x + sx * ca - sy * sa;
    double Y = pose.y + sx * sa + sy * ca;
    const double dx = X - I.origin_x, dy = Y - I.origin_y;
    const double co = std::cos(I.origin_yaw), so = std::sin(I.origin_yaw);
    const double local_x = co * dx + so * dy;
    const double local_y = -so * dx + co * dy;
    int px = static_cast<int>(local_x / I.map_res);
    int py = I.map_image.rows - 1 - static_cast<int>(local_y / I.map_res);
    n++;
    // レンジ比例の許容 (sin 0.5 deg 相当) を足す
    double tol = p.wfrac_tolerance_m + b.range * 0.00873;
    if (px < 0 || px >= I.map_image.cols || py < 0 || py >= I.map_image.rows ||
      I.wall_dt.at<float>(py, px) * I.map_res > tol) {
      missed++;
    }
  }
  return n > 0 ? static_cast<double>(missed) / n : 1.0;
}

int OrientedFieldLocalizer::wfracGateSelect(
  const std::vector<PoseCandidate> & cands, const std::vector<Beam> & beams,
  double margin_thresh) const
{
  if (cands.empty() || margin_thresh <= 0) return 0;
  const double top2 = cands.size() > 1 ? cands[1].score : 0.0;
  const double margin = top2 > 0 ? cands[0].score / top2 : 1e30;
  if (margin >= margin_thresh) return 0;   // 拮抗していなければ触らない
  const double floor_score = cands[0].score / margin_thresh;
  std::vector<double> wf(cands.size(), 1.0);
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < static_cast<int>(cands.size()); i++) {
    wf[i] = wallMissFraction(cands[i], beams);
  }
  int best = 0;
  for (size_t i = 1; i < cands.size(); i++) {
    if (cands[i].score >= floor_score && wf[i] < wf[best]) best = static_cast<int>(i);
  }
  return best;
}

}  // namespace oriented_field_localization
