#pragma once

#include "core/bevel/bevel.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/planetary/planetary.hpp"
#include "core/spur/spur.hpp"
#include "preview/scene3d/scene.hpp"

namespace geargen::preview::scene3d {

[[nodiscard]] Scene3D build_scene(const core::spur::SetGeometry& geometry,
                                  double mesh_position_rad = 0.0);
[[nodiscard]] Scene3D build_scene(const core::bevel::SetGeometry& geometry,
                                  double mesh_position_rad = 0.0);
[[nodiscard]] Scene3D build_scene(const core::hypoid::SetGeometry& geometry,
                                  double mesh_position_rad = 0.0);
[[nodiscard]] Scene3D build_scene(const core::planetary::SetGeometry& geometry,
                                  double mesh_position_rad = 0.0);

} // namespace geargen::preview::scene3d
