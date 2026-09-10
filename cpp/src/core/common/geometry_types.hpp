#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "core/common/numerics.hpp"

namespace geargen::core {

struct Point2 {
    double x{};
    double y{};
};

struct Point3 {
    double x{};
    double y{};
    double z{};
};

using Vector2 = Point2;
using Vector3 = Point3;

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

[[nodiscard]] inline Point2 operator+(Point2 lhs, Point2 rhs) noexcept
{
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] inline Point2 operator-(Point2 lhs, Point2 rhs) noexcept
{
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] inline Point2 operator*(Point2 value, double scale) noexcept
{
    return {value.x * scale, value.y * scale};
}

[[nodiscard]] inline Point3 operator+(Point3 lhs, Point3 rhs) noexcept
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

[[nodiscard]] inline Point3 operator-(Point3 lhs, Point3 rhs) noexcept
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

[[nodiscard]] inline Point3 operator*(Point3 value, double scale) noexcept
{
    return {value.x * scale, value.y * scale, value.z * scale};
}

[[nodiscard]] inline double dot(Point2 lhs, Point2 rhs) noexcept
{
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] inline double dot(Point3 lhs, Point3 rhs) noexcept
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

[[nodiscard]] inline double norm(Point2 value) noexcept
{
    return std::hypot(value.x, value.y);
}

[[nodiscard]] inline double norm(Point3 value) noexcept
{
    return std::sqrt(dot(value, value));
}

[[nodiscard]] inline Point2 rotate(Point2 point, double angle_rad) noexcept
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {c * point.x - s * point.y, s * point.x + c * point.y};
}

} // namespace geargen::core
