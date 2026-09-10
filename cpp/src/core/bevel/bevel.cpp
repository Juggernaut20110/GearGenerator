#include "core/bevel/bevel.hpp"

#include "core/common/geometry_types.hpp"

#include <algorithm>
#include <cmath>

namespace geargen::core::bevel {

BevelParameters default_parameters(double module_mm, int z1, int z2, bool zerol)
{
    BevelParameters result;
    result.module_mm = module_mm;
    result.z1 = z1;
    result.z2 = z2;
    const double sigma = result.shaft_angle_deg * kPi / 180.0;
    const double delta1 = std::atan2(std::sin(sigma),
                                     static_cast<double>(z2) / z1 + std::cos(sigma));
    const double outer_cone_distance =
        module_mm * static_cast<double>(z1) / (2.0 * std::sin(delta1));
    const bool curved = zerol;
    result.face_width_mm = std::min((curved ? 0.30 : 1.0 / 3.0) * outer_cone_distance,
                                    10.0 * module_mm);
    result.bore_mm = std::max(0.25 * module_mm * static_cast<double>(z1), 4.0);
    result.hub_thickness_mm = 2.5 * module_mm;
    result.min_root_thickness_mm = 0.25 * module_mm;
    if (curved) {
        result.cutter_radius_mm = outer_cone_distance - result.face_width_mm / 2.0;
    }
    return result;
}

} // namespace geargen::core::bevel
