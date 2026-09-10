#include "core/hypoid/hypoid.hpp"

namespace geargen::core::hypoid {

HypoidParameters default_parameters(double module_mm, int z1, int z2)
{
    return HypoidParameters::with_defaults(module_mm, z1, z2);
}

SetGeometry derive(const HypoidParameters& p)
{
    // This is the stable common-data projection used by the first regression
    // stage. The full non-zero-offset Method 1 solver remains a later port;
    // exposing these fields now makes that boundary explicit instead of
    // fabricating a complete hypoid solution.
    const double delta1 = std::atan2(std::sin(p.sigma()),
                                     p.ratio() + std::cos(p.sigma()));
    const double delta2 = p.sigma() - delta1;
    const double outer = p.wheel_outer_radius() / std::sin(delta2);
    const double mean = outer - p.face_width / 2.0;
    const double inner = outer - p.face_width;
    return {p,
            {p.z1, delta1, outer},
            {p.z2, delta2, outer},
            p.offset,
            outer,
            mean,
            inner,
            p.module / std::cos(p.psi1()),
    };
}

} // namespace geargen::core::hypoid
