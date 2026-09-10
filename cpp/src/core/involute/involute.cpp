#include "core/involute/involute.hpp"

#include <cmath>

namespace geargen::core::involute {

double involute_function(double pressure_angle_rad) noexcept
{
    return std::tan(pressure_angle_rad) - pressure_angle_rad;
}

ProfilePoint external_point(double base_radius_mm, double roll_parameter,
                            double base_space_angle_rad)
{
    const double radius = base_radius_mm *
                          std::sqrt(1.0 + roll_parameter * roll_parameter);
    const double angle = base_space_angle_rad + roll_parameter -
                         std::atan(roll_parameter);
    return {{radius * std::cos(angle), radius * std::sin(angle)},
            roll_parameter};
}

} // namespace geargen::core::involute
