#include "oriented_field_localization/map_yaml_loader.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

using oriented_field_localization::loadMapYaml;

namespace
{
int failures = 0;
void check(bool condition, const char * message)
{
  std::printf("  [%s] %s\n", condition ? "ok" : "NG", message);
  if (!condition) failures++;
}
}  // namespace

int main()
{
  const auto dir = std::filesystem::temp_directory_path() / "ofl_map_yaml_test";
  std::filesystem::create_directories(dir);
  const auto image_path = dir / "quoted map.pgm";
  cv::Mat image(1, 3, CV_8UC1);
  image.at<unsigned char>(0, 0) = 0;
  image.at<unsigned char>(0, 1) = 127;
  image.at<unsigned char>(0, 2) = 255;
  cv::imwrite(image_path.string(), image);
  const auto yaml_path = dir / "map.yaml";
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: \"quoted map.pgm\"\nresolution: 0.1\n"
         << "origin: [1.0, -2.0, 0.75]\nnegate: 0\n"
         << "occupied_thresh: 0.65\nfree_thresh: 0.2\nmode: trinary\n";
  }
  const auto map = loadMapYaml(yaml_path.string());
  check(map.image_path == image_path.string(), "quoted relative image path resolves");
  check(std::fabs(map.origin_yaw - 0.75) < 1e-12, "origin yaw is preserved");
  check(map.image.at<unsigned char>(0, 0) == 0 &&
    map.image.at<unsigned char>(0, 1) == 205 &&
    map.image.at<unsigned char>(0, 2) == 255, "thresholds produce occupied/unknown/free");
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: \"" << image_path.string() << "\"\nresolution: 0.1\n"
         << "origin: [1.0, -2.0, 0.75]\nnegate: 0\n"
         << "occupied_thresh: 0.65\nfree_thresh: 0.2\nmode: scale\n";
  }
  check(loadMapYaml(yaml_path.string()).image_path == image_path.string(),
    "absolute image path is accepted");
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: \"quoted map.pgm\"\nresolution: 0.1\n"
         << "origin: [0, 0, 0]\nnegate: 1\n"
         << "occupied_thresh: 0.65\nfree_thresh: 0.2\nmode: trinary\n";
  }
  const auto negated = loadMapYaml(yaml_path.string());
  check(negated.image.at<unsigned char>(0, 0) == 255 &&
    negated.image.at<unsigned char>(0, 2) == 0, "negate reverses occupied and free");
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: \"quoted map.pgm\"\nresolution: 0.1\n"
         << "origin: [0, 0, 0]\nmode: raw\n";
  }
  const auto raw = loadMapYaml(yaml_path.string());
  check(raw.image.at<unsigned char>(0, 0) == 255 &&
    raw.image.at<unsigned char>(0, 1) == 205 &&
    raw.image.at<unsigned char>(0, 2) == 205, "raw mode maps 0..100 and marks larger values unknown");

  bool rejected = false;
  const auto bad_path = dir / "bad.yaml";
  {
    std::ofstream yaml(bad_path);
    yaml << "image: nowhere.pgm\nresolution: nope\norigin: [0, 0, 0]\n";
  }
  try {loadMapYaml(bad_path.string());} catch (const std::runtime_error &) {rejected = true;}
  check(rejected, "malformed numeric field throws a descriptive error");
  std::filesystem::remove_all(dir);
  return failures == 0 ? 0 : 1;
}
