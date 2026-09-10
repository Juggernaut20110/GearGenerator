#pragma once

#include "core/bevel/bevel.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/planetary/planetary.hpp"
#include "core/spur/spur.hpp"
#include "solidworks/builder_common.hpp"

namespace geargen::solidworks::detail {

[[nodiscard]] PartArtifact build_spur_part(
    const Session& session, const core::spur::SetGeometry& geometry,
    std::string_view member, const std::filesystem::path& save_path);
[[nodiscard]] PartArtifact build_bevel_part(
    const Session& session, const core::bevel::SetGeometry& geometry,
    std::string_view member, const std::filesystem::path& save_path);
[[nodiscard]] PartArtifact build_hypoid_part(
    const Session& session, const core::hypoid::SetGeometry& geometry,
    std::string_view member, const std::filesystem::path& save_path);

} // namespace geargen::solidworks::detail
