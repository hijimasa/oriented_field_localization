#include "oriented_field_localization/map_yaml_loader.hpp"
#include "oriented_field_localization/oriented_field_matcher.hpp"

int main(int argc, char ** argv)
{
  oriented_field_localization::MatcherParams params;
  oriented_field_localization::OrientedFieldLocalizer localizer(params);
  if (argc > 1) {
    const auto map = oriented_field_localization::loadMapYaml(argv[1]);
    localizer.setMap(
      map.image, map.resolution, map.origin_x, map.origin_y, map.origin_yaw);
  }
  return localizer.hasMap() ? 0 : 0;
}
