#ifndef ORIENTED_FIELD_LOCALIZATION__MAP_YAML_LOADER_HPP_
#define ORIENTED_FIELD_LOCALIZATION__MAP_YAML_LOADER_HPP_

#include <opencv2/core/mat.hpp>

#include <string>

namespace oriented_field_localization
{

struct LoadedMap
{
  cv::Mat image;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::string image_path;
};

/// Load a Nav2/map_server YAML and normalize its image to 0/205/255-style grayscale.
/// Throws std::runtime_error with the offending field/path on malformed input.
LoadedMap loadMapYaml(const std::string & yaml_path);

}  // namespace oriented_field_localization

#endif  // ORIENTED_FIELD_LOCALIZATION__MAP_YAML_LOADER_HPP_
