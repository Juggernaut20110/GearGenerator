#include "core/placement/placement.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geargen::core::placement {

double gear_clocking(int teeth) noexcept
{
    if (teeth <= 0) {
        return 0.0;
    }
    const double pitch = kTau / static_cast<double>(teeth);
    const double nearest = std::round(kPi / pitch - 0.5);
    const double residual = kPi - (nearest + 0.5) * pitch;
    if (std::abs(residual) < 1e-9) {
        return 0.0;
    }
    const double wrapped = std::fmod(residual, pitch);
    return wrapped < 0.0 ? wrapped + pitch : wrapped;
}

double angular_velocity_ratio(int z1, int z2, bool internal) noexcept
{
    if (z1 == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (internal ? 1.0 : -1.0) * static_cast<double>(z2) /
           static_cast<double>(z1);
}

Matrix3 rot_y(double angle_rad) noexcept
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

Matrix3 rot_z(double angle_rad) noexcept
{
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Matrix3 matmul(const Matrix3& lhs, const Matrix3& rhs) noexcept
{
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t k = 0; k < 3; ++k) {
                result[row][column] += lhs[row][k] * rhs[k][column];
            }
        }
    }
    return result;
}

Point3 apply(const Matrix3& matrix, Point3 vector) noexcept
{
    return {
        matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
        matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
        matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
    };
}

std::array<double, 16> to_solidworks_array(const Matrix3& matrix,
                                            Point3 translation_m,
                                            double scale) noexcept
{
    // SOLIDWORKS expects the basis vectors in columns, followed by translation
    // in metres and the scale slot. This is intentionally not a row-major
    // Eigen/Qt transform conversion.
    return {
        matrix[0][0], matrix[1][0], matrix[2][0],
        matrix[0][1], matrix[1][1], matrix[2][1],
        matrix[0][2], matrix[1][2], matrix[2][2],
        translation_m.x, translation_m.y, translation_m.z,
        scale, 0.0, 0.0, 0.0,
    };
}

Point3 axis_of(const Matrix3& matrix) noexcept
{
    return apply(matrix, {0.0, 0.0, 1.0});
}

double angle_between(Point3 lhs, Point3 rhs) noexcept
{
    const double dot = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    const double lhs_norm = std::sqrt(lhs.x * lhs.x + lhs.y * lhs.y + lhs.z * lhs.z);
    const double rhs_norm = std::sqrt(rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z);
    if (lhs_norm == 0.0 || rhs_norm == 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::acos(std::clamp(dot / (lhs_norm * rhs_norm), -1.0, 1.0));
}

} // namespace geargen::core::placement
