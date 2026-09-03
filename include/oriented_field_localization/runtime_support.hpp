#ifndef ORIENTED_FIELD_LOCALIZATION__RUNTIME_SUPPORT_HPP_
#define ORIENTED_FIELD_LOCALIZATION__RUNTIME_SUPPORT_HPP_

#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace oriented_field_localization
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

inline double wrapPi(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

inline Pose2D relative(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  const double dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, wrapPi(b.yaw - a.yaw)};
}

inline Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {
    a.x + c * b.x - s * b.y,
    a.y + s * b.x + c * b.y,
    wrapPi(a.yaw + b.yaw)};
}

inline Pose2D inverse(const Pose2D & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-(c * a.x + s * a.y), s * a.x - c * a.y, wrapPi(-a.yaw)};
}

inline Beam transformBeam(const Beam & beam, const Pose2D & target_from_scan)
{
  const double x = beam.range * std::cos(beam.bearing);
  const double y = beam.range * std::sin(beam.bearing);
  const double c = std::cos(target_from_scan.yaw), s = std::sin(target_from_scan.yaw);
  const double tx = target_from_scan.x + c * x - s * y;
  const double ty = target_from_scan.y + s * x + c * y;
  return {std::hypot(tx, ty), std::atan2(ty, tx)};
}

/// Timestamped odometry with interpolation and bounded nearest-sample fallback.
class OdomHistory
{
public:
  explicit OdomHistory(std::size_t capacity = 200) : capacity_(capacity) {}

  void add(double stamp_s, const Pose2D & pose)
  {
    if (!std::isfinite(stamp_s) || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) return;
    Sample sample{stamp_s, pose};
    const auto pos = std::upper_bound(samples_.begin(), samples_.end(), stamp_s,
      [](double stamp, const Sample & other) {return stamp < other.stamp_s;});
    samples_.insert(pos, sample);
    while (samples_.size() > capacity_) samples_.pop_front();
  }

  bool lookup(double stamp_s, double nearest_tolerance_s, Pose2D * out) const
  {
    if (!out || samples_.empty() || !std::isfinite(stamp_s) ||
      !std::isfinite(nearest_tolerance_s) || nearest_tolerance_s < 0.0)
    {
      return false;
    }
    const auto hi = std::lower_bound(samples_.begin(), samples_.end(), stamp_s,
      [](const Sample & sample, double stamp) {return sample.stamp_s < stamp;});
    if (hi != samples_.end() && hi->stamp_s == stamp_s) {
      *out = hi->pose;
      return true;
    }
    if (hi != samples_.begin() && hi != samples_.end()) {
      const auto lo = std::prev(hi);
      const double dt = hi->stamp_s - lo->stamp_s;
      if (dt <= 0.0) return false;
      const double u = (stamp_s - lo->stamp_s) / dt;
      out->x = lo->pose.x + u * (hi->pose.x - lo->pose.x);
      out->y = lo->pose.y + u * (hi->pose.y - lo->pose.y);
      out->yaw = wrapPi(lo->pose.yaw + u * wrapPi(hi->pose.yaw - lo->pose.yaw));
      return true;
    }
    const Sample & nearest = hi == samples_.end() ? samples_.back() : samples_.front();
    if (std::fabs(nearest.stamp_s - stamp_s) > nearest_tolerance_s) return false;
    *out = nearest.pose;
    return true;
  }

private:
  struct Sample
  {
    double stamp_s;
    Pose2D pose;
  };
  std::size_t capacity_;
  std::deque<Sample> samples_;
};

/// Whether a queued scan has a reason to run.
inline bool localizationShouldRun(bool requested, bool auto_localize, bool tracking)
{
  return requested || auto_localize || tracking;
}

/// A service-triggered request completes after one GLOBAL accept when TRACK is
/// disabled, or only after continuous TRACK has taken ownership when enabled.
inline bool localizationRequestCompletes(bool track_enabled, bool now_tracking)
{
  return !track_enabled || now_tracking;
}

}  // namespace oriented_field_localization

#endif  // ORIENTED_FIELD_LOCALIZATION__RUNTIME_SUPPORT_HPP_
