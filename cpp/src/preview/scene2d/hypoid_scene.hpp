#pragma once

#include "core/hypoid/hypoid.hpp"
#include "preview/scene2d/scene.hpp"

#include <filesystem>

namespace geargen::preview::hypoid {

[[nodiscard]] Scene2D contact_scene(const core::hypoid::SetGeometry& geometry,
                                    const std::string& member);
[[nodiscard]] Scene2D section_scene(const core::hypoid::SetGeometry& geometry,
                                    const std::string& member);
[[nodiscard]] Scene2D blank_scene(const core::hypoid::SetGeometry& geometry,
                                  const std::string& member);
[[nodiscard]] Scene2D build_scene(const core::hypoid::SetGeometry& geometry,
                                   const std::string& member,
                                   const std::string& key);
[[nodiscard]] std::vector<std::pair<std::string, std::string>> scene_labels();
[[nodiscard]] std::vector<Row> derived_rows(const core::hypoid::SetGeometry& geometry);
[[nodiscard]] std::vector<std::string> csv_lines(
    const core::hypoid::SetGeometry& geometry, const std::string& member);
void write_csv(const std::filesystem::path& path,
               const core::hypoid::SetGeometry& geometry,
               const std::string& member);

} // namespace geargen::preview::hypoid
