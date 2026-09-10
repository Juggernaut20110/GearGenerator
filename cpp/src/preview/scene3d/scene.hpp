#pragma once

#include "core/common/geometry_types.hpp"

#include <array>
#include <string>
#include <vector>

namespace geargen::preview {

struct Polyline3D {
    std::vector<core::Point3> points;
    std::string style;
    bool closed{false};
};

struct Scene3D {
    std::string title;
    std::vector<Polyline3D> polylines;

    [[nodiscard]] std::array<double, 6> bounds() const noexcept;
};

class OrthographicCamera {
public:
    [[nodiscard]] core::Point2 project(core::Point3 point) const noexcept;
    void set_scale(double scale) noexcept { scale_ = scale; }

private:
    double scale_{1.0};
};

} // namespace geargen::preview
