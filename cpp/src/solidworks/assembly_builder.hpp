#pragma once

#include "core/bevel/bevel.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/planetary/planetary.hpp"
#include "core/spur/spur.hpp"
#include "solidworks/part_builder.hpp"

namespace geargen::solidworks::detail {

[[nodiscard]] BuildResult build_spur_set(
    const Session& session, const core::spur::SetGeometry& geometry,
    const std::filesystem::path& output_directory, const BuildOptions& options);
[[nodiscard]] BuildResult build_bevel_set(
    const Session& session, const core::bevel::SetGeometry& geometry,
    const std::filesystem::path& output_directory, const BuildOptions& options);
[[nodiscard]] BuildResult build_hypoid_set(
    const Session& session, const core::hypoid::SetGeometry& geometry,
    const std::filesystem::path& output_directory, const BuildOptions& options);
[[nodiscard]] BuildResult build_planetary_set(
    const Session& session, const core::planetary::SetGeometry& geometry,
    const std::filesystem::path& output_directory, const BuildOptions& options);

} // namespace geargen::solidworks::detail
