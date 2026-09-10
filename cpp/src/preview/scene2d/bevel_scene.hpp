#pragma once

#include "core/bevel/bevel.hpp"
#include "preview/scene2d/scene.hpp"

#include <filesystem>
#include <string>

namespace geargen::preview::bevel {

[[nodiscard]] std::vector<core::Point2> space_loop(
    const core::bevel::ToothSpaceSection& section, int tip_points = 9);
[[nodiscard]] std::vector<core::Point2> tooth_loop(
    const core::bevel::ToothSpaceSection& section, double pitch_step_rad);
[[nodiscard]] std::vector<core::Point2> to_axial(
    const std::vector<core::Point2>& points,
    const core::bevel::ToothSpaceSection& section);
[[nodiscard]] Scene2D developed_scene(const core::bevel::SetGeometry& geometry,
                                       const std::string& member,
                                       int neighbours = 1);
[[nodiscard]] Scene2D axial_scene(const core::bevel::SetGeometry& geometry,
                                  const std::string& member);
[[nodiscard]] Scene2D trace_scene(const core::bevel::SetGeometry& geometry,
                                  const std::string& member,
                                  int neighbours = 2);
[[nodiscard]] Scene2D blank_scene(const core::bevel::SetGeometry& geometry,
                                  const std::string& member);
[[nodiscard]] Scene2D build_scene(const core::bevel::SetGeometry& geometry,
                                   const std::string& member,
                                   const std::string& key);
[[nodiscard]] std::vector<std::pair<std::string, std::string>> scene_labels();
[[nodiscard]] std::vector<Row> derived_rows(const core::bevel::SetGeometry& geometry);
[[nodiscard]] std::vector<std::string> csv_lines(
    const core::bevel::SetGeometry& geometry, const std::string& member);
void write_csv(const std::filesystem::path& path,
               const core::bevel::SetGeometry& geometry,
               const std::string& member);

} // namespace geargen::preview::bevel
