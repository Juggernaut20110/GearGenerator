#include "core/bevel/bevel.hpp"

#include "core/common/geometry_types.hpp"

namespace geargen::core::bevel {

BevelParameters default_parameters(double module_mm, int z1, int z2, bool zerol)
{
    return BevelParameters::with_defaults(module_mm, z1, z2, zerol);
}

SetGeometry derive(const BevelParameters& p)
{
    const double delta1 = std::atan2(std::sin(p.sigma()),
                                     p.ratio() + std::cos(p.sigma()));
    const double delta2 = p.sigma() - delta1;
    const double outer = p.module * p.z1 / (2.0 * std::sin(delta1));
    const double inner = outer - p.face_width;
    const double whole_depth = 2.188 * p.module;
    const double working_depth = 2.0 * p.module;
    const double clearance = 0.188 * p.module;
    const auto member = [&](int teeth, double angle) {
    return MemberGeometry{
            teeth,
            angle,
            outer,
            inner,
            static_cast<double>(teeth) / std::cos(angle),
        };
    };
    return {p,
            outer,
            outer - p.face_width / 2.0,
            inner,
            inner / outer,
            working_depth,
            whole_depth,
            clearance,
            kPi * p.module,
            member(p.z1, delta1),
            member(p.z2, delta2),
    };
}

} // namespace geargen::core::bevel
