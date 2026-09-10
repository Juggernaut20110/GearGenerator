#pragma once

#include "core/planetary/planetary.hpp"
#include "preview/scene2d/scene.hpp"

#include <filesystem>

namespace geargen::preview::planetary {

[[nodiscard]] Scene2D train_scene(const core::planetary::SetGeometry& geometry,
                                  const std::string& member = "sun");
[[nodiscard]] Scene2D transverse_scene(const core::planetary::SetGeometry& geometry,
                                        const std::string& member);
[[nodiscard]] Scene2D blank_scene(const core::planetary::SetGeometry& geometry,
                                  const std::string& member);
[[nodiscard]] Scene2D build_scene(const core::planetary::SetGeometry& geometry,
                                   const std::string& member,
                                   const std::string& key);
[[nodiscard]] std::vector<std::pair<std::string, std::string>> scene_labels();
[[nodiscard]] std::vector<Row> derived_rows(const core::planetary::SetGeometry& geometry);
[[nodiscard]] std::vector<std::string> csv_lines(
    const core::planetary::SetGeometry& geometry, const std::string& member);
void write_csv(const std::filesystem::path& path,
               const core::planetary::SetGeometry& geometry,
               const std::string& member);

} // namespace geargen::preview::planetary
