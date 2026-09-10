#include "preview/scene2d/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geargen::preview {

std::array<double, 4> Scene2D::bounds() const noexcept
{
    double xmin = std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto& line : polylines) {
        for (const auto& point : line.points) {
            xmin = std::min(xmin, point.x);
            ymin = std::min(ymin, point.y);
            xmax = std::max(xmax, point.x);
            ymax = std::max(ymax, point.y);
        }
    }
    if (!std::isfinite(xmin)) {
        return {0.0, 0.0, 0.0, 0.0};
    }
    return {xmin, ymin, xmax, ymax};
}

} // namespace geargen::preview
