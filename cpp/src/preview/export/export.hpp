#pragma once

#include "preview/scene2d/scene.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace geargen::preview::exporter {

[[nodiscard]] std::vector<std::string> dxf_lines(const Scene2D& scene);
void write_dxf(const std::filesystem::path& path, const Scene2D& scene);

} // namespace geargen::preview::exporter
