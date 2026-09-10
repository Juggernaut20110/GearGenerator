#include "preview/scene3d/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geargen::preview {

std::array<double, 6> Scene3D::bounds() const noexcept
{
    double xmin = std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double zmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    double zmax = -std::numeric_limits<double>::infinity();
    for (const auto& line : polylines) {
        for (const auto& point : line.points) {
            xmin = std::min(xmin, point.x);
            ymin = std::min(ymin, point.y);
            zmin = std::min(zmin, point.z);
            xmax = std::max(xmax, point.x);
            ymax = std::max(ymax, point.y);
            zmax = std::max(zmax, point.z);
        }
    }
    if (!std::isfinite(xmin)) {
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    return {xmin, ymin, zmin, xmax, ymax, zmax};
}

core::Point2 OrthographicCamera::project(core::Point3 point) const noexcept
{
    return {point.x * scale_, point.y * scale_};
}

} // namespace geargen::preview
