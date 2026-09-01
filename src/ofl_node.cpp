// oriented_field_localization の ROS 2 ノード。
//
// **大域位置推定 (GLOBAL) だけを行う。**追跡はしない。1 スキャンから地図座標の
// 3-DoF 姿勢を求めて /initialpose へ流し、以降の追跡は AMCL などに任せる、という
// 分担を想定している (CBGL と同じ位置づけ)。オドメトリ・状態機械・ICP 精密化は
// 持たない。
//
// トピック:
//   subscribe  /scan  sensor_msgs/LaserScan
//              /map   nav_msgs/OccupancyGrid   (map_yaml_path 未指定のとき)
//   publish    /initialpose            geometry_msgs/PoseWithCovarianceStamped
//              ~/candidates            geometry_msgs/PoseArray  (可視化・診断)
//   service    ~/global_localization   std_srvs/Empty           (探索の開始)
#include "oriented_field_localization/oriented_field_matcher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
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

    wfrac_margin_ = declare_parameter("wfrac_margin", 1.05);
    global_min_margin_ = declare_parameter("global_min_margin", 1.0);
    auto_localize_ = declare_parameter("auto_localize", false);
    publish_tf_ = declare_parameter("publish_tf", false);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
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

    localizer_ = std::make_unique<OrientedFieldLocalizer>(p);

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(msg);});

    if (map_yaml.empty()) {
      auto map_qos = rclcpp::QoS(1).transient_local().reliable();
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onMap(msg);});
      RCLCPP_INFO(get_logger(), "waiting for /map (transient_local)");
    } else {
      loadMapFromYaml(map_yaml);
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    candidates_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/candidates", 10);
    if (publish_tf_) tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    service_ = create_service<std_srvs::srv::Empty>(
      "~/global_localization",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        requested_ = true;
        cv_.notify_one();
      });

    worker_ = std::thread(&OflNode::workerLoop, this);
    RCLCPP_INFO(get_logger(),
      "ready: call ~/global_localization%s", auto_localize_ ? " (auto_localize on)" : "");
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
        auto b = line.find('[');
        auto c1 = line.find(',', b);
        auto c2 = line.find(',', c1 + 1);
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

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(mu_);
      last_scan_ = msg;
    }
    if (auto_localize_) requested_ = true;
    cv_.notify_one();
  }

  static std::vector<Beam> toBeams(
    const sensor_msgs::msg::LaserScan & s, double max_range)
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
        cv_.wait(lk, [this] {return stop_ || (requested_ && last_scan_ && map_ready_);});
        if (stop_) return;
        scan = last_scan_;
        requested_ = false;
      }
      if (!scan) continue;

      const std::vector<Beam> beams = toBeams(*scan, localizer_->params().max_range);
      if (beams.size() < 8) {
        RCLCPP_WARN(get_logger(), "too few valid beams (%zu); skipping", beams.size());
        continue;
      }
      const auto t0 = now();
      std::vector<PoseCandidate> cands = localizer_->localize(beams);
      const double dt = (now() - t0).seconds();
      if (cands.empty()) {
        RCLCPP_WARN(get_logger(), "no candidate found");
        continue;
      }

      // 拮抗帯だけ WFRAC (スキャン点が地図壁から外れる割合) で選び直す
      int pick = 0;
      if (wfrac_margin_ > 0) pick = localizer_->wfracGateSelect(cands, beams, wfrac_margin_);
      const PoseCandidate & best = cands[pick];
      const double margin = (cands.size() > 1 && cands[1].score > 0)
        ? cands[0].score / cands[1].score : 1e9;

      publishCandidates(cands, scan->header.stamp);
      if (margin < global_min_margin_) {
        RCLCPP_INFO(get_logger(),
          "rejected: top1/top2 margin %.2f < %.2f (%.0f ms, %zu candidates)",
          margin, global_min_margin_, 1e3 * dt, cands.size());
        continue;
      }
      publishPose(best, scan->header.stamp);
      RCLCPP_INFO(get_logger(),
        "accepted (%.2f, %.2f, %.1f deg) score %.4f margin %.2f in %.0f ms",
        best.x, best.y, best.yaw * 180.0 / M_PI, best.score, margin, 1e3 * dt);
    }
  }

  void publishPose(const PoseCandidate & c, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.pose.pose.position.x = c.x;
    m.pose.pose.position.y = c.y;
    m.pose.pose.orientation.z = std::sin(c.yaw / 2.0);
    m.pose.pose.orientation.w = std::cos(c.yaw / 2.0);
    m.pose.covariance[0] = 0.25;
    m.pose.covariance[7] = 0.25;
    m.pose.covariance[35] = 0.07;
    pose_pub_->publish(m);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = c.x;
      tf.transform.translation.y = c.y;
      tf.transform.rotation = m.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
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
      q.orientation.z = std::sin(c.yaw / 2.0);
      q.orientation.w = std::cos(c.yaw / 2.0);
      arr.poses.push_back(q);
    }
    candidates_pub_->publish(arr);
  }

  std::unique_ptr<OrientedFieldLocalizer> localizer_;
  double wfrac_margin_ = 1.05;
  double global_min_margin_ = 1.0;
  bool auto_localize_ = false;
  bool publish_tf_ = false;
  std::string map_frame_, base_frame_;

  std::atomic<bool> map_ready_{false};
  std::atomic<bool> requested_{false};
  std::atomic<bool> stop_{false};
  std::mutex mu_;
  std::condition_variable cv_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::thread worker_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
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
