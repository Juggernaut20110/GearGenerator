#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace geargen::core {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTau = 2.0 * kPi;

struct Point2 {
    double x{};
    double y{};
};

struct Point3 {
    double x{};
    double y{};
    double z{};
};

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct Transform3 {
    Matrix3 rotation{};
    Point3 translation{};
    double scale{1.0};
};

constexpr Matrix3 identity_matrix() noexcept
{
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

inline bool nearly_equal(double lhs, double rhs, double tolerance) noexcept
{
    return std::abs(lhs - rhs) <= tolerance;
}

} // namespace geargen::core
