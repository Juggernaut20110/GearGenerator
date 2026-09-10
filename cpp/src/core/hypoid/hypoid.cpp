#include "core/hypoid/hypoid.hpp"

#include <algorithm>

namespace geargen::core::hypoid {

HypoidParameters default_parameters(double module_mm, int z1, int z2)
{
    HypoidParameters result;
    result.module_mm = module_mm;
    result.z1 = z1;
    result.z2 = z2;
    result.face_width_mm = std::min(0.18 * module_mm * static_cast<double>(z2),
                                    10.0 * module_mm);
    result.bore_mm = std::max(0.25 * module_mm * static_cast<double>(z1), 4.0);
    result.hub_thickness_mm = 2.5 * module_mm;
    return result;
}

} // namespace geargen::core::hypoid
