#pragma once

#include <algorithm>
#include <cmath>

namespace geargen::core {

// Core lengths are millimetres.  User-facing parameter records retain the
// Python application's degree-valued angle inputs; all geometry-facing angle
// accessors below return radians.  SOLIDWORKS performs the separate mm->m
// conversion at its adapter boundary.
inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTau = 2.0 * kPi;

[[nodiscard]] constexpr double degrees_to_radians(double degrees) noexcept
{
    return degrees * kPi / 180.0;
}

[[nodiscard]] constexpr double radians_to_degrees(double radians) noexcept
{
    return radians * 180.0 / kPi;
}

struct Tolerance {
    double absolute{1e-9};
    double relative{1e-12};
};

// This is the comparison contract for scalar reference snapshots.  The
// absolute component is appropriate for millimetres near the origin; the
// relative component prevents large derived dimensions from being rejected
// merely because their decimal representation differs in the last bits.
inline constexpr Tolerance kReferenceTolerance{1e-9, 1e-12};
inline constexpr double kGeometryTolerance = 1e-9;
inline constexpr double kClearanceWarningTolerance = 1e-6;

[[nodiscard]] inline bool finite(double value) noexcept
{
    return std::isfinite(value);
}

[[nodiscard]] inline bool approximately_equal(double lhs, double rhs,
                                              Tolerance tolerance =
                                                  kReferenceTolerance) noexcept
{
    if (!finite(lhs) || !finite(rhs)) {
        return lhs == rhs;
    }
    const double difference = std::abs(lhs - rhs);
    const double scale = std::max(std::abs(lhs), std::abs(rhs));
    return difference <=
           std::max(tolerance.absolute, tolerance.relative * scale);
}

[[nodiscard]] inline double signed_angle(double magnitude_degrees,
                                         bool right_hand) noexcept
{
    const double magnitude = degrees_to_radians(magnitude_degrees);
    return right_hand ? magnitude : -magnitude;
}

} // namespace geargen::core
