#include "core/spur/spur.hpp"

#include <algorithm>
#include <cmath>

namespace {

double involute(double angle)
{
    return std::tan(angle) - angle;
}

double inverse_involute(double target)
{
    double lower = 1e-10;
    double upper = geargen::core::kPi / 2.0 - 1e-8;
    for (int i = 0; i < 80; ++i) {
        const double middle = (lower + upper) / 2.0;
        if (involute(middle) < target) {
            lower = middle;
        } else {
            upper = middle;
        }
    }
    return (lower + upper) / 2.0;
}

} // namespace

namespace geargen::core::spur {

SpurParameters default_parameters(double module_mm, int z1, int z2)
{
    return SpurParameters::with_defaults(module_mm, z1, z2);
}

SetGeometry derive(const SpurParameters& p)
{
    const double alpha_t = p.alpha_t();
    const double reference_distance = p.reference_centre_distance();
    const double shift = p.profile_shift_combination();
    double working_alpha = alpha_t;
    if (std::abs(shift) > 1e-14) {
        const double count = p.internal ? p.z2 - p.z1 : p.z1 + p.z2;
        working_alpha = inverse_involute(
            involute(alpha_t) + 2.0 * shift * std::tan(p.alpha_n()) / count);
    }
    double working_distance = p.working_centre_distance.value_or(0.0);
    if (!p.working_centre_distance.has_value()) {
        const double base_1 = p.transverse_module() * p.z1 / 2.0 *
                              std::cos(alpha_t);
        const double base_2 = p.transverse_module() * p.z2 / 2.0 *
                              std::cos(alpha_t);
        const double working_1 = base_1 / std::cos(working_alpha);
        const double working_2 = base_2 / std::cos(working_alpha);
        working_distance = p.internal ? working_2 - working_1
                                      : working_1 + working_2;
    }
    double tip_alteration = 0.0;
    if (p.tip_alteration_mode == "explicit") {
        tip_alteration = p.tip_alteration_coefficient.value_or(0.0);
    } else if (p.tip_alteration_mode == "iso_clearance") {
        const double delta = working_distance - reference_distance;
        tip_alteration = p.internal ? -delta / p.module + shift
                                    : delta / p.module - shift;
    }

    const auto member = [&](int teeth, double profile_shift,
                            bool internal, double beta) {
        MemberGeometry result;
        result.teeth = teeth;
        result.beta_rad = beta;
        result.internal = internal;
        result.reference_radius_mm = p.transverse_module() * teeth / 2.0;
        result.base_radius_mm = result.reference_radius_mm * std::cos(alpha_t);
        result.working_radius_mm =
            result.base_radius_mm / std::cos(working_alpha);
        const double addendum = internal
            ? p.module * (p.basic_rack_addendum_factor - profile_shift +
                          tip_alteration)
            : p.module * (p.basic_rack_addendum_factor + profile_shift +
                          tip_alteration);
        const double dedendum = internal
            ? p.module * (p.basic_rack_dedendum_factor() + profile_shift)
            : p.module * (p.basic_rack_dedendum_factor() - profile_shift);
        result.tip_radius_mm = internal
            ? result.reference_radius_mm - addendum
            : result.reference_radius_mm + addendum;
        result.root_radius_mm = internal
            ? result.reference_radius_mm + dedendum
            : result.reference_radius_mm - dedendum;
        result.twist_rad = p.face_width * std::tan(beta) /
                           result.reference_radius_mm;
        return result;
    };

    const double first_beta = p.beta();
    const double second_beta = p.internal ? first_beta : -first_beta;
    return {
        p,
        p.transverse_module(),
        alpha_t,
        working_alpha,
        working_distance,
        kPi * p.transverse_module(),
        (p.basic_rack_addendum_factor +
         p.basic_rack_dedendum_factor()) * p.module,
        member(p.z1, p.profile_shift_1, false, first_beta),
        member(p.z2, p.profile_shift_2, p.internal, second_beta),
    };
}

} // namespace geargen::core::spur
