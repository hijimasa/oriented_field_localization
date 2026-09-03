#include "oriented_field_localization/map_yaml_loader.hpp"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace oriented_field_localization
{
namespace
{
template<typename T>
T required(const YAML::Node & root, const char * key)
{
  if (!root[key]) throw std::runtime_error(std::string("map YAML missing '") + key + "'");
  try {
    return root[key].as<T>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("invalid map YAML '") + key + "': " + e.what());
  }
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}
}  // namespace

LoadedMap loadMapYaml(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot parse map YAML '" + yaml_path + "': " + e.what());
  }
  if (!root.IsMap()) throw std::runtime_error("map YAML root must be a mapping: " + yaml_path);

  LoadedMap out;
  out.resolution = required<double>(root, "resolution");
  if (!std::isfinite(out.resolution) || out.resolution <= 0.0) {
    throw std::runtime_error("map YAML 'resolution' must be finite and > 0");
  }
  const YAML::Node origin = root["origin"];
  if (!origin || !origin.IsSequence() || origin.size() < 3) {
    throw std::runtime_error("map YAML 'origin' must be [x, y, yaw]");
  }
  try {
    out.origin_x = origin[0].as<double>();
    out.origin_y = origin[1].as<double>();
    out.origin_yaw = origin[2].as<double>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("invalid map YAML 'origin': ") + e.what());
  }
  if (!std::isfinite(out.origin_x) || !std::isfinite(out.origin_y) ||
    !std::isfinite(out.origin_yaw))
  {
    throw std::runtime_error("map YAML 'origin' values must be finite");
  }

  std::filesystem::path image_path(required<std::string>(root, "image"));
  if (image_path.is_relative()) image_path = std::filesystem::path(yaml_path).parent_path() / image_path;
  image_path = image_path.lexically_normal();
  out.image_path = image_path.string();
  const cv::Mat source = cv::imread(out.image_path, cv::IMREAD_GRAYSCALE);
  if (source.empty()) throw std::runtime_error("cannot load map image: " + out.image_path);

  const int negate = root["negate"] ? required<int>(root, "negate") : 0;
  if (negate != 0 && negate != 1) throw std::runtime_error("map YAML 'negate' must be 0 or 1");
  const double occupied = root["occupied_thresh"] ? required<double>(root, "occupied_thresh") : 0.65;
  const double free = root["free_thresh"] ? required<double>(root, "free_thresh") : 0.196;
  if (!std::isfinite(occupied) || !std::isfinite(free) || free < 0.0 || occupied > 1.0 ||
    free >= occupied)
  {
    throw std::runtime_error(
            "map YAML thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1");
  }
  const std::string mode = lower(root["mode"] ? required<std::string>(root, "mode") : "trinary");
  if (mode != "trinary" && mode != "scale" && mode != "raw") {
    throw std::runtime_error("map YAML 'mode' must be trinary, scale, or raw");
  }

  out.image.create(source.size(), CV_8UC1);
  for (int y = 0; y < source.rows; ++y) {
    const auto * src = source.ptr<unsigned char>(y);
    auto * dst = out.image.ptr<unsigned char>(y);
    for (int x = 0; x < source.cols; ++x) {
      if (mode == "raw") {
        // Nav2 raw mode stores occupancy directly in [0,100]; other values are unknown.
        dst[x] = src[x] <= 100 ? static_cast<unsigned char>(
          std::lround(255.0 * (1.0 - src[x] / 100.0))) : 205;
        continue;
      }
      const double shade = static_cast<double>(src[x]) / 255.0;
      const double occupancy = negate ? shade : 1.0 - shade;
      if (occupancy > occupied) dst[x] = 0;
      else if (occupancy < free) dst[x] = 255;
      else if (mode == "trinary") dst[x] = 205;
      else dst[x] = static_cast<unsigned char>(std::lround(
          255.0 * (occupied - occupancy) / (occupied - free)));
    }
  }
  return out;
}

}  // namespace oriented_field_localization
