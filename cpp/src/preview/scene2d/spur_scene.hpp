#pragma once

#include "core/spur/spur.hpp"
#include "preview/scene2d/scene.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace geargen::preview::spur {

[[nodiscard]] std::vector<core::Point2> space_loop(
    const core::spur::ToothSpaceSection& section, int tip_points = 9);
[[nodiscard]] std::vector<core::Point2> tooth_loop(
    const core::spur::ToothSpaceSection& section, double pitch_step_rad);
[[nodiscard]] Scene2D transverse_scene(const core::spur::SetGeometry& geometry,
                                        const std::string& member,
                                        int neighbours = 1);
[[nodiscard]] Scene2D twist_scene(const core::spur::SetGeometry& geometry,
                                  const std::string& member);
[[nodiscard]] Scene2D blank_scene(const core::spur::SetGeometry& geometry,
                                  const std::string& member);
[[nodiscard]] Scene2D build_scene(const core::spur::SetGeometry& geometry,
                                   const std::string& member,
                                   const std::string& key);
[[nodiscard]] std::vector<std::pair<std::string, std::string>> scene_labels();
[[nodiscard]] std::vector<Row> derived_rows(const core::spur::SetGeometry& geometry);
[[nodiscard]] std::vector<std::string> csv_lines(
    const core::spur::SetGeometry& geometry, const std::string& member);
void write_csv(const std::filesystem::path& path,
               const core::spur::SetGeometry& geometry,
               const std::string& member);

} // namespace geargen::preview::spur
