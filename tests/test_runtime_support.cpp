#include "oriented_field_localization/runtime_support.hpp"

#include <cmath>
#include <cstdio>

using oriented_field_localization::OdomHistory;
using oriented_field_localization::Pose2D;
using oriented_field_localization::localizationRequestCompletes;
using oriented_field_localization::localizationShouldRun;
using oriented_field_localization::transformBeam;

namespace
{
int failures = 0;
void check(bool condition, const char * message)
{
  std::printf("  [%s] %s\n", condition ? "ok" : "NG", message);
  if (!condition) failures++;
}
bool near(double a, double b, double eps = 1e-9) {return std::fabs(a - b) < eps;}
}  // namespace

int main()
{
  std::printf("== service request lifecycle ==\n");
  bool requested = true;
  check(localizationShouldRun(requested, false, false), "service request starts GLOBAL");
  if (localizationRequestCompletes(true, false)) requested = false;
  check(requested, "GLOBAL accept before threshold keeps request active");
  if (localizationRequestCompletes(true, true)) requested = false;
  check(!requested, "request clears after transition to TRACK");
  requested = true;
  if (localizationRequestCompletes(false, false)) requested = false;
  check(!requested, "GLOBAL-only mode completes after one accept");

  std::printf("== lidar extrinsic ==\n");
  const auto shifted = transformBeam({2.0, 0.0}, {1.0, 0.0, M_PI / 2.0});
  check(near(shifted.range, std::sqrt(5.0)), "translation changes endpoint range");
  check(near(shifted.bearing, std::atan2(2.0, 1.0)), "translation and yaw change bearing");

  std::printf("== timestamped odometry ==\n");
  OdomHistory history;
  history.add(10.0, {0.0, 0.0, 170.0 * M_PI / 180.0});
  history.add(12.0, {2.0, 4.0, -170.0 * M_PI / 180.0});
  Pose2D out;
  check(history.lookup(11.0, 0.0, &out), "bracketed scan time is available");
  check(near(out.x, 1.0) && near(out.y, 2.0), "position interpolates at scan time");
  check(std::fabs(std::fabs(out.yaw) - M_PI) < 1e-9, "yaw interpolates across wrap boundary");
  check(!history.lookup(12.2, 0.1, &out), "stale nearest odometry is rejected");
  check(history.lookup(12.05, 0.1, &out), "nearby odometry is accepted within tolerance");

  return failures == 0 ? 0 : 1;
}
