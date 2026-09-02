// oriented_field_localization の ROS 2 ノード。
//
// GLOBAL (地図全域の探索) と TRACK (事前姿勢の周りの局所探索) を状態機械で
// 切り替える。
//
//   GLOBAL --- track_after_accepts 回連続で採択 ---> TRACK
//   TRACK  --- max_consecutive_rejects 回連続で棄却 ---> GLOBAL
//
// TRACK の事前姿勢は「最後に採択した地図姿勢 + それ以降のオドメトリ差分」で、
// オドメトリが無い場合は最後の採択姿勢をそのまま使う (スキャン間の移動が
// track_search_m に収まる前提)。
//
// トピック:
//   subscribe  scan   sensor_msgs/LaserScan
//              map    nav_msgs/OccupancyGrid   (map_yaml_path 未指定のとき)
//              odom   nav_msgs/Odometry        (use_odometry のとき)
//   publish    ~/pose                  geometry_msgs/PoseWithCovarianceStamped
//              /initialpose            同上 (GLOBAL からの引き継ぎ時のみ)
//              ~/candidates            geometry_msgs/PoseArray
//   service    ~/global_localization   std_srvs/Empty   (GLOBAL 探索の強制)
//
// TF は REP-105 に従う。位置推定が出すのは map -> odom であって map -> base_link
// ではない (T_map_odom = T_map_base * T_odom_base^-1)。odom を出すノードが居ない
// 構成のために tf_mode: map_to_base も用意してある。
#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using oriented_field_localization::Beam;
using oriented_field_localization::MatcherParams;
using oriented_field_localization::OrientedFieldLocalizer;
using oriented_field_localization::PoseCandidate;

namespace
{
/// 2D 姿勢 (地図 or オドメトリ座標)。
struct Pose2D
{
  double x = 0, y = 0, yaw = 0;
};

double wrapPi(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// b を a の座標系から見た相対姿勢 (a^-1 * b)。
Pose2D relative(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  const double dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, wrapPi(b.yaw - a.yaw)};
}

/// a に相対姿勢 d を掛ける (a * d)。
Pose2D compose(const Pose2D & a, const Pose2D & d)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * d.x - s * d.y, a.y + s * d.x + c * d.y, wrapPi(a.yaw + d.yaw)};
}

/// a^-1 (2D 剛体変換の逆)。
Pose2D inverse(const Pose2D & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), wrapPi(-a.yaw)};
}

geometry_msgs::msg::Quaternion yawQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

enum class TfMode { None, MapToOdom, MapToBase };

}  // namespace

class OflNode : public rclcpp::Node
{
public:
  OflNode()
  : Node("oriented_field_localization")
  {
    MatcherParams p;
    p.match_resolution = declare_parameter("match_resolution", p.match_resolution);
    p.max_range = declare_parameter("max_range", p.max_range);
    p.min_range = declare_parameter("min_range", p.min_range);
    p.margin_pixels = declare_parameter("margin_pixels", p.margin_pixels);
    p.pyramid_levels = declare_parameter("pyramid_levels", p.pyramid_levels);
    p.coarse_angle_step = declare_parameter("coarse_angle_step", p.coarse_angle_step);
    p.peaks_per_angle = declare_parameter("peaks_per_angle", p.peaks_per_angle);
    p.candidate_pool_size = declare_parameter("candidate_pool_size", p.candidate_pool_size);
    p.intermediate_pool_size =
      declare_parameter("intermediate_pool_size", p.intermediate_pool_size);
    p.refine_angle_window = declare_parameter("refine_angle_window", p.refine_angle_window);
    p.refine_search_m = declare_parameter("refine_search_m", p.refine_search_m);
    p.nms_separation_m = declare_parameter("nms_separation_m", p.nms_separation_m);
    p.nms_separation_deg = declare_parameter("nms_separation_deg", p.nms_separation_deg);
    p.normal_weight = declare_parameter("normal_weight", p.normal_weight);
    p.mass_weight = declare_parameter("mass_weight", p.mass_weight);
    p.map_normal_sigma = declare_parameter("map_normal_sigma", p.map_normal_sigma);
    p.wfrac_tolerance_m = declare_parameter("wfrac_tolerance_m", p.wfrac_tolerance_m);
    p.track_search_m = declare_parameter("track_search_m", p.track_search_m);
    p.track_angle_window_deg =
      declare_parameter("track_angle_window_deg", p.track_angle_window_deg);

    wfrac_margin_ = declare_parameter("wfrac_margin", 1.05);
    global_min_margin_ = declare_parameter("global_min_margin", 1.05);
    auto_localize_ = declare_parameter("auto_localize", false);
    enable_track_ = declare_parameter("enable_track", true);
    track_after_accepts_ = declare_parameter("track_after_accepts", 3);
    max_consecutive_rejects_ = declare_parameter("max_consecutive_rejects", 5);
    max_accept_jump_m_ = declare_parameter("max_accept_jump_m", 0.5);
    max_accept_yaw_deg_ = declare_parameter("max_accept_yaw_deg", 20.0);
    // 誤ロックの検出。跳び判定だけでは足りない: 追跡が誤った場所へ乗ると、
    // 以後の事前姿勢もその誤った場所から出るので「跳んでいない」と見えてしまい、
    // 自分自身と整合したまま永久に戻れない (kidnap の実測で確認)。
    // スキャン点が地図の壁に載らない割合 (WFRAC) は事前姿勢に依存しない
    // 幾何量なので、誤ロックではっきり跳ね上がる。
    track_max_wfrac_ = declare_parameter("track_max_wfrac", 0.35);
    use_odometry_ = declare_parameter("use_odometry", true);
    publish_initialpose_ = declare_parameter("publish_initialpose", true);
    initialpose_repeat_ = declare_parameter("initialpose_repeat", 5);
    initialpose_wait_s_ = declare_parameter("initialpose_wait_s", 60);
    update_min_d_ = declare_parameter("update_min_d", 0.2);
    update_min_a_deg_ = declare_parameter("update_min_a_deg", 15.0);
    update_max_interval_s_ = declare_parameter("update_max_interval_s", 1.0);
    smooth_base_m_ = declare_parameter("smooth_base_m", 0.0);
    smooth_gain_ = declare_parameter("smooth_gain", 0.0);
    smooth_rot_lever_m_ = declare_parameter("smooth_rot_lever_m", 0.3);
    smooth_bypass_m_ = declare_parameter("smooth_bypass_m", 0.3);
    smooth_bypass_deg_ = declare_parameter("smooth_bypass_deg", 10.0);
    smooth_saturate_scans_ = declare_parameter("smooth_saturate_scans", 5);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    transform_tolerance_ = declare_parameter("transform_tolerance", 0.2);
    const std::string tf_mode = declare_parameter("tf_mode", std::string("none"));
    tf_mode_ = tf_mode == "map_to_odom" ? TfMode::MapToOdom
      : tf_mode == "map_to_base" ? TfMode::MapToBase : TfMode::None;
    const std::string map_yaml = declare_parameter("map_yaml_path", std::string(""));

    // margin は「テンプレートが地図の縁で切れない」ための下限を満たす必要がある。
    // 推奨値まで上げると相関の外挿が 0 になり、DFT も最小になる。
    if (p.margin_pixels < p.minimumMargin()) {
      RCLCPP_WARN(get_logger(),
        "margin_pixels %d < minimum %d; raising it (poses near the map border would "
        "otherwise be unreachable)", p.margin_pixels, p.minimumMargin());
      p.margin_pixels = p.minimumMargin();
    }
    if (p.margin_pixels < p.recommendedMargin()) {
      RCLCPP_INFO(get_logger(),
        "margin_pixels %d < recommended %d; the correlation pads by %d px extra",
        p.margin_pixels, p.recommendedMargin(), p.recommendedMargin() - p.margin_pixels);
    }
    if (tf_mode_ == TfMode::MapToOdom && !use_odometry_) {
      RCLCPP_WARN(get_logger(),
        "tf_mode=map_to_odom needs use_odometry; falling back to map_to_base");
      tf_mode_ = TfMode::MapToBase;
    }

    localizer_ = std::make_unique<OrientedFieldLocalizer>(p);

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(msg);});
    if (use_odometry_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", sensor_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(msg);});
    }

    if (map_yaml.empty()) {
      auto map_qos = rclcpp::QoS(1).transient_local().reliable();
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onMap(msg);});
      RCLCPP_INFO(get_logger(), "waiting for /map (transient_local)");
    } else {
      loadMapFromYaml(map_yaml);
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    candidates_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/candidates", 10);
    if (tf_mode_ != TfMode::None) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    service_ = create_service<std_srvs::srv::Empty>(
      "~/global_localization",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        // 明示的な要求は追跡状態を捨てて GLOBAL からやり直す (kidnap 対応)
        std::lock_guard<std::mutex> lk(state_mu_);
        has_pose_ = false;
        consecutive_accepts_ = 0;
        consecutive_rejects_ = 0;
        requested_ = true;
        cv_.notify_one();
      });

    if (publish_initialpose_ && initialpose_repeat_ > 1) {
      // /initialpose を 1 回きり volatile で出すと、購読側 (AMCL など) の
      // activate がこちらの初回ロックより後になった場合に黙って失われる。
      // **回数で待つのでは足りない** (実測で AMCL の activate は初回ロックの
      // 6 秒後だった)。購読者が現れるまでは回数を消費せずに待ち、現れてから
      // 出し直す。待ちは initialpose_wait_s_ 秒で打ち切る。
      initialpose_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this] {
          geometry_msgs::msg::PoseWithCovarianceStamped m;
          {
            std::lock_guard<std::mutex> lk(initialpose_mu_);
            if (initialpose_left_ <= 0) return;
            if (initialpose_pub_->get_subscription_count() == 0) {
              if (++initialpose_waited_ > initialpose_wait_s_) initialpose_left_ = 0;
              return;
            }
            initialpose_left_--;
            m = initialpose_msg_;
          }
          m.header.stamp = now();
          initialpose_pub_->publish(m);
        });
    }

    worker_ = std::thread(&OflNode::workerLoop, this);
    RCLCPP_INFO(get_logger(), "ready (%s, TRACK %s, odometry %s)",
      auto_localize_ ? "auto_localize" : "call ~/global_localization",
      enable_track_ ? "on" : "off", use_odometry_ ? "on" : "off");
  }

  ~OflNode() override
  {
    stop_ = true;
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  void loadMapFromYaml(const std::string & yaml_path)
  {
    // map_server 形式の最小パース (image / resolution / origin)
    std::ifstream ifs(yaml_path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "cannot open YAML: %s", yaml_path.c_str());
      return;
    }
    std::string line, image_file;
    double resolution = 0.05, ox = 0, oy = 0;
    while (std::getline(ifs, line)) {
      if (line.find("image:") != std::string::npos) {
        image_file = line.substr(line.find(':') + 2);
      } else if (line.find("resolution:") != std::string::npos) {
        resolution = std::stod(line.substr(line.find(':') + 2));
      } else if (line.find("origin:") != std::string::npos) {
        const auto b = line.find('[');
        const auto c1 = line.find(',', b);
        const auto c2 = line.find(',', c1 + 1);
        ox = std::stod(line.substr(b + 1, c1 - b - 1));
        oy = std::stod(line.substr(c1 + 1, c2 - c1 - 1));
      }
    }
    while (!image_file.empty() && (image_file.back() == ' ' || image_file.back() == '\r')) {
      image_file.pop_back();
    }
    const std::string dir = yaml_path.substr(0, yaml_path.find_last_of('/'));
    cv::Mat img = cv::imread(dir + "/" + image_file, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      RCLCPP_ERROR(get_logger(), "failed to load map image: %s", image_file.c_str());
      return;
    }
    setMap(img, resolution, ox, oy);
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (map_ready_) return;
    const int w = msg->info.width, h = msg->info.height;
    cv::Mat image(h, w, CV_8UC1, cv::Scalar(205));
    for (int i = 0; i < w * h; i++) {
      const int8_t v = msg->data[i];
      if (v == 0) image.data[i] = 255;
      else if (v >= 50) image.data[i] = 0;
    }
    cv::flip(image, image, 0);   // OccupancyGrid は行 0 が最小 y
    setMap(image, msg->info.resolution,
      msg->info.origin.position.x, msg->info.origin.position.y);
  }

  void setMap(const cv::Mat & img, double res, double ox, double oy)
  {
    const auto t0 = now();
    localizer_->setMap(img, res, ox, oy);
    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "map %dx%d @ %.3f m/px loaded in %.2f s",
      img.cols, img.rows, res, (now() - t0).seconds());
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mu_);
    odom_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
      yawOf(msg->pose.pose.orientation)};
    has_odom_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    {
      // 処理中に届いたスキャンは捨てて最新だけを残す (古いスキャンで推定しても
      // 出てくるのは過去の姿勢なので、溜めても意味が無い)
      std::lock_guard<std::mutex> lk(mu_);
      pending_scan_ = msg;
    }
    cv_.notify_one();
  }

  static std::vector<Beam> toBeams(const sensor_msgs::msg::LaserScan & s, double max_range)
  {
    std::vector<Beam> beams;
    beams.reserve(s.ranges.size());
    const double hi = std::min(static_cast<double>(s.range_max), max_range);
    for (size_t i = 0; i < s.ranges.size(); i++) {
      const double r = s.ranges[i];
      if (!std::isfinite(r) || r < s.range_min || r >= hi) continue;
      beams.push_back({r, s.angle_min + s.angle_increment * static_cast<double>(i)});
    }
    return beams;
  }

  void workerLoop()
  {
    while (!stop_) {
      sensor_msgs::msg::LaserScan::SharedPtr scan;
      {
        std::unique_lock<std::mutex> lk(mu_);
        // 未処理のスキャンが 1 枚あり、かつ走る理由があるときだけ起きる。
        // 走る理由 = TRACK 中 / auto_localize / サービス要求。
        cv_.wait(lk, [this] {
            return stop_ || (pending_scan_ && map_ready_ &&
              (requested_ || tracking_ || auto_localize_));
          });
        if (stop_) return;
        scan = pending_scan_;
        pending_scan_.reset();          // 1 枚のスキャンは 1 回だけ処理する
        if (!(auto_localize_ || tracking_)) requested_ = false;
      }
      if (!scan) continue;

      const std::vector<Beam> beams = toBeams(*scan, localizer_->params().max_range);
      if (beams.size() < 8) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "too few valid beams (%zu); skipping", beams.size());
        continue;
      }

      // 事前姿勢: 最後の採択姿勢 + それ以降のオドメトリ差分
      Pose2D prior;
      Pose2D odom_now;
      bool have_prior = false;
      {
        std::lock_guard<std::mutex> lk(state_mu_);
        odom_now = odom_;
        if (has_pose_) {
          prior = last_pose_;
          if (use_odometry_ && has_odom_ && had_odom_at_accept_) {
            prior = compose(last_pose_, relative(odom_at_accept_, odom_));
          }
          have_prior = true;
        }
      }
      const bool use_track = enable_track_ && tracking_ && have_prior;

      // ---- 探索を動いた分だけに間引く (検証は毎スキャン走らせる) ----
      // 高いのは相関探索 (実測 29.9 ms/scan) のほうで、WFRAC はビーム数ぶんの
      // 地図参照だけなので桁が違う。**距離でゲートするのは探索だけにする。**
      // 検証まで間引くと、kidnap 後に停止している間は誤ロックを検出できなくなる。
      if (use_track && update_min_d_ > 0) {
        const Pose2D moved = relative(pose_at_search_, prior);
        const double since = (now() - last_search_).seconds();
        const bool moved_enough =
          std::hypot(moved.x, moved.y) >= update_min_d_ ||
          std::fabs(moved.yaw) * 180.0 / M_PI >= update_min_a_deg_;
        const bool timed_out =
          update_max_interval_s_ > 0 && since >= update_max_interval_s_;
        if (!moved_enough && !timed_out) {
          PoseCandidate pc;
          pc.x = prior.x; pc.y = prior.y; pc.yaw = prior.yaw;
          const double w = localizer_->wallMissFraction(pc, beams);
          if (!(track_max_wfrac_ > 0 && w > track_max_wfrac_)) {
            // 事前姿勢は地図と整合している。探索せず、そのまま出す。
            // map -> odom は前回の採択から変わらない (AMCL と同じ挙動)。
            skipped_++;
            PoseCandidate out;
            out.x = prior.x; out.y = prior.y; out.yaw = prior.yaw; out.score = 0;
            publishPose(out, odom_now, scan->header.stamp,
              /*also_initialpose=*/false, /*snap=*/false, /*have_odom=*/true);
            continue;
          }
          // WFRAC が閾値を超えた: 動いていなくても探索する
        }
        pose_at_search_ = prior;
        last_search_ = now();
      }

      const auto t0 = std::chrono::steady_clock::now();
      std::vector<PoseCandidate> cands;
      if (use_track) {
        PoseCandidate pr;
        pr.x = prior.x; pr.y = prior.y; pr.yaw = prior.yaw;
        cands = localizer_->track(beams, pr);
      } else {
        cands = localizer_->localize(beams);
      }
      const double dt = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

      if (cands.empty()) {
        onReject("no candidate", use_track);
        continue;
      }
      publishCandidates(cands, scan->header.stamp);

      int pick = 0;
      if (wfrac_margin_ > 0) pick = localizer_->wfracGateSelect(cands, beams, wfrac_margin_);
      const PoseCandidate & best = cands[pick];
      const double margin = (cands.size() > 1 && cands[1].score > 0)
        ? cands[0].score / cands[1].score : 1e9;

      double wfrac = -1.0;
      if (use_track) {
        // (1) 事前姿勢からの跳び: TRACK の窓は広いので、窓の中でも遠くへ飛んだ解は
        //     単発の誤マッチとみなして捨てる
        const double dpos = std::hypot(best.x - prior.x, best.y - prior.y);
        const double dyaw = std::fabs(wrapPi(best.yaw - prior.yaw)) * 180.0 / M_PI;
        if ((max_accept_jump_m_ > 0 && dpos > max_accept_jump_m_) ||
          (max_accept_yaw_deg_ > 0 && dyaw > max_accept_yaw_deg_))
        {
          onReject("jump", true, dpos, dyaw);
          continue;
        }
        // (2) 幾何の整合: 誤ロックは跳びでは捕まらないので、事前姿勢に依らない
        //     WFRAC で見る
        if (track_max_wfrac_ > 0) {
          wfrac = localizer_->wallMissFraction(best, beams);
          if (wfrac > track_max_wfrac_) {
            onReject("wfrac", true, wfrac);
            continue;
          }
        }
      } else if (margin < global_min_margin_) {
        onReject("margin", false, margin);
        // マージン不足で捨てたときは、サービス 1 回ぶんの要求を消費しない。
        // 消費すると「曖昧だったので黙って何も返さない」で終わってしまう。
        // 次のスキャンでやり直す (auto_localize なら元から毎スキャン走る)。
        if (!auto_localize_) requested_ = true;
        continue;
      }

      onAccept(best, odom_now, scan->header.stamp, use_track, margin, dt, cands.size(),
        wfrac);
    }
  }

  void onAccept(
    const PoseCandidate & best, const Pose2D & odom_now, const rclcpp::Time & stamp,
    bool was_track, double margin, double dt, size_t n_cands, double wfrac)
  {
    bool became_track = false;
    bool first_lock = false;
    bool have_odom = false;
    {
      std::lock_guard<std::mutex> lk(state_mu_);
      have_odom = has_odom_;
      first_lock = !has_pose_;
      last_pose_ = {best.x, best.y, best.yaw};
      has_pose_ = true;
      odom_at_accept_ = odom_now;
      had_odom_at_accept_ = has_odom_;
      consecutive_rejects_ = 0;
      consecutive_accepts_++;
      if (enable_track_ && !tracking_ && consecutive_accepts_ >= track_after_accepts_) {
        tracking_ = true;
        became_track = true;
      }
    }
    // GLOBAL からの採択 (初回ロック・追跡喪失からの復帰) は平滑化せず即座に反映する
    publishPose(best, odom_now, stamp, /*also_initialpose=*/!was_track && first_lock,
      /*snap=*/!was_track, have_odom);
    RCLCPP_INFO(get_logger(),
      "%s accept (%.2f, %.2f, %.1f deg) score %.4f margin %.2f wfrac %.3f in %.0f ms "
      "(%zu cands, skip %d)%s",
      was_track ? "TRACK" : "GLOBAL", best.x, best.y, best.yaw * 180.0 / M_PI,
      best.score, margin, wfrac, 1e3 * dt, n_cands, skipped_.load(),
      became_track ? " -> TRACK" : "");
  }

  void onReject(const char * why, bool was_track, double a = 0, double b = 0)
  {
    bool back_to_global = false;
    {
      std::lock_guard<std::mutex> lk(state_mu_);
      consecutive_accepts_ = 0;
      consecutive_rejects_++;
      if (tracking_ && consecutive_rejects_ >= max_consecutive_rejects_) {
        tracking_ = false;
        has_pose_ = false;      // 追跡喪失: 事前姿勢を捨てて GLOBAL から取り直す
        consecutive_rejects_ = 0;
        back_to_global = true;
        requested_ = true;      // auto_localize でなくても 1 回は GLOBAL を回す
      }
    }
    RCLCPP_INFO(get_logger(), "%s reject (%s %.2f %.2f)%s",
      was_track ? "TRACK" : "GLOBAL", why, a, b,
      back_to_global ? " -> GLOBAL" : "");
  }

  /// 出力する姿勢だけを鈍らせる。**内部の事前姿勢 (last_pose_) は生のまま**なので、
  /// TRACK の引き込みも WFRAC の判定も kidnap の検出も一切変わらない。
  ///
  /// 平滑化するのは「前回の出力をオドメトリで伝播した予測」からの**補正量**だけで、
  /// 実際の運動は素通りする。したがって遅れは補正量の変化率 (= オドメトリの
  /// ドリフト率) にしか比例しない。
  ///
  /// snap = true (GLOBAL からの採択) と、補正が smooth_bypass_* を超えた場合は
  /// 鈍らせない。kidnap 復帰の 10 m 級の補正を鈍らせると、数秒間まちがった姿勢で
  /// 航行することになり、跳びより悪い。
  Pose2D smoothOutput(
    const Pose2D & raw, const Pose2D & odom_now, bool snap, bool have_odom)
  {
    const bool can_predict = has_out_ && use_odometry_ && have_odom && had_out_odom_;
    const bool smoothing = smooth_base_m_ > 0 || smooth_gain_ > 0;
    if (!smoothing || snap || !can_predict) {
      out_pose_ = raw;
      saturated_ = 0;
    } else {
      const Pose2D od = relative(out_odom_, odom_now);   // このスキャンの運動量
      const Pose2D pred = compose(out_pose_, od);
      const double dx = raw.x - pred.x, dy = raw.y - pred.y;
      const double dyaw = wrapPi(raw.yaw - pred.yaw);
      const double d = std::hypot(dx, dy);
      const double dyaw_deg = std::fabs(dyaw) * 180.0 / M_PI;
      if ((smooth_bypass_m_ > 0 && d > smooth_bypass_m_) ||
        (smooth_bypass_deg_ > 0 && dyaw_deg > smooth_bypass_deg_))
      {
        out_pose_ = raw;                       // 大きい補正は即座に反映する
        saturated_ = 0;
      } else {
        // 1 スキャンあたりの補正量に上限を置く (スルーレート制限)。
        // 一次遅れではなく上限にしてあるのは、オドメトリのドリフトが旋回中に
        // 集中するので、時定数だと遅れがそこだけ膨らんで読みにくいため。
        // **補正権限を運動量に比例させる。**オドメトリの誤差は移動量と回転量に
        // 比例して増える (AMCL の alpha1..alpha4 と同じ形) ので、上限を固定に
        // すると、止まっているときは緩すぎ、激しく動いているときは足りない。
        // 実測: オドメトリ誤差の増加は巡航で 0.21 m/s、kidnap 後の切り返しで
        // 0.60 m/s だった。固定 0.20 m/s では後者に 3 倍足りない。
        const double dtrans = std::hypot(od.x, od.y);
        const double drot = std::fabs(od.yaw);
        const double cap_m =
          smooth_base_m_ + smooth_gain_ * (dtrans + smooth_rot_lever_m_ * drot);
        const double cap_deg =
          (smooth_base_m_ / std::max(smooth_rot_lever_m_, 1e-6) + smooth_gain_ * drot)
          * 180.0 / M_PI;
        const bool sat_pos = d > cap_m;
        const bool sat_yaw = dyaw_deg > cap_deg;
        if (sat_pos || sat_yaw) saturated_++; else saturated_ = 0;
        if (smooth_saturate_scans_ > 0 && saturated_ >= smooth_saturate_scans_) {
          // **上限に張り付いたままなら追随を諦めて飛ばす。**補正の要求が権限を
          // 上回り続けると、bypass に届かない値 (実測 0.15 m) で遅れが安定して
          // しまい、大きさだけ見ている bypass では検出できない。
          out_pose_ = raw;
          saturated_ = 0;
        } else {
          const double ks = sat_pos ? cap_m / d : 1.0;
          const double kyaw = sat_yaw ? (cap_deg * M_PI / 180.0) / std::fabs(dyaw) : 1.0;
          out_pose_ = {pred.x + ks * dx, pred.y + ks * dy, wrapPi(pred.yaw + kyaw * dyaw)};
        }
      }
    }
    out_odom_ = odom_now;
    has_out_ = true;
    had_out_odom_ = have_odom;
    return out_pose_;
  }

  void publishPose(
    const PoseCandidate & c, const Pose2D & odom_now, const rclcpp::Time & stamp,
    bool also_initialpose, bool snap, bool have_odom)
  {
    const Pose2D out = smoothOutput({c.x, c.y, c.yaw}, odom_now, snap, have_odom);
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.pose.pose.position.x = out.x;
    m.pose.pose.position.y = out.y;
    m.pose.pose.orientation = yawQuat(out.yaw);
    m.pose.covariance[0] = 0.25;
    m.pose.covariance[7] = 0.25;
    m.pose.covariance[35] = 0.07;
    pose_pub_->publish(m);
    if (also_initialpose && publish_initialpose_) {
      initialpose_pub_->publish(m);
      std::lock_guard<std::mutex> lk(initialpose_mu_);
      initialpose_msg_ = m;
      initialpose_left_ = initialpose_repeat_ - 1;
      initialpose_waited_ = 0;
    }

    if (!tf_broadcaster_) return;
    const rclcpp::Time tf_stamp =
      stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = tf_stamp;
    if (tf_mode_ == TfMode::MapToOdom) {
      // REP-105: 位置推定が出すのは map -> odom。
      //   T_map_base = T_map_odom * T_odom_base  =>  T_map_odom = T_map_base * T_odom_base^-1
      const Pose2D map_odom = compose(out, inverse(odom_now));
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = map_odom.x;
      tf.transform.translation.y = map_odom.y;
      tf.transform.rotation = yawQuat(map_odom.yaw);
    } else {
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = out.x;
      tf.transform.translation.y = out.y;
      tf.transform.rotation = yawQuat(out.yaw);
    }
    tf_broadcaster_->sendTransform(tf);
  }

  void publishCandidates(
    const std::vector<PoseCandidate> & cands, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseArray arr;
    arr.header.stamp = stamp;
    arr.header.frame_id = map_frame_;
    for (const PoseCandidate & c : cands) {
      geometry_msgs::msg::Pose q;
      q.position.x = c.x;
      q.position.y = c.y;
      q.orientation = yawQuat(c.yaw);
      arr.poses.push_back(q);
    }
    candidates_pub_->publish(arr);
  }

  std::unique_ptr<OrientedFieldLocalizer> localizer_;
  double wfrac_margin_ = 1.05;
  double global_min_margin_ = 1.05;
  bool auto_localize_ = false;
  bool enable_track_ = true;
  int track_after_accepts_ = 3;
  int max_consecutive_rejects_ = 5;
  double max_accept_jump_m_ = 0.5;
  double max_accept_yaw_deg_ = 20.0;
  double track_max_wfrac_ = 0.35;
  bool use_odometry_ = true;
  bool publish_initialpose_ = true;
  int initialpose_repeat_ = 5;
  int initialpose_wait_s_ = 60;
  double update_min_d_ = 0.2;           ///< これだけ動くまで探索しない [m]。0 で無効
  double update_min_a_deg_ = 15.0;      ///< 同 [deg]
  double update_max_interval_s_ = 1.0;  ///< 停止していても この間隔では探索する [s]
  double smooth_base_m_ = 0.0;          ///< 静止時の補正権限 [m/scan]。0 で無効
  double smooth_gain_ = 0.0;            ///< 運動量に対する補正権限の比。0 で無効
  double smooth_rot_lever_m_ = 0.3;     ///< 回転 [rad] を位置誤差 [m] に換算する腕
  double smooth_bypass_m_ = 0.3;
  double smooth_bypass_deg_ = 10.0;
  int smooth_saturate_scans_ = 5;
  double transform_tolerance_ = 0.2;
  TfMode tf_mode_ = TfMode::None;
  std::string map_frame_, odom_frame_, base_frame_;

  std::atomic<bool> map_ready_{false};
  std::atomic<bool> requested_{false};
  std::atomic<bool> stop_{false};
  std::atomic<bool> tracking_{false};

  std::mutex mu_;                     ///< last_scan_ / requested_
  std::condition_variable cv_;
  sensor_msgs::msg::LaserScan::SharedPtr pending_scan_;

  std::mutex state_mu_;               ///< 追跡状態とオドメトリ
  Pose2D last_pose_, odom_, odom_at_accept_;
  bool has_pose_ = false, has_odom_ = false, had_odom_at_accept_ = false;
  Pose2D out_pose_, out_odom_;          ///< 出力用 (平滑化後)。内部の事前姿勢とは別
  bool has_out_ = false, had_out_odom_ = false;
  int saturated_ = 0;                   ///< 補正が上限に張り付いた連続回数
  Pose2D pose_at_search_;               ///< 最後に探索したときの事前姿勢
  rclcpp::Time last_search_{0, 0, RCL_ROS_TIME};
  std::atomic<int> skipped_{0};         ///< 探索を省いたスキャン数
  int consecutive_accepts_ = 0, consecutive_rejects_ = 0;

  std::mutex initialpose_mu_;
  geometry_msgs::msg::PoseWithCovarianceStamped initialpose_msg_;
  int initialpose_left_ = 0;
  int initialpose_waited_ = 0;
  rclcpp::TimerBase::SharedPtr initialpose_timer_;

  std::thread worker_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidates_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OflNode>());
  rclcpp::shutdown();
  return 0;
}
