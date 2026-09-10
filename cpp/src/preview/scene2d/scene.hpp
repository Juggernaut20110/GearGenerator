#pragma once

#include "core/common/geometry_types.hpp"

#include <array>
#include <string>
#include <vector>

namespace geargen::preview {

struct Style {
    std::string name;
    std::string colour{"#333333"};
    double width{1.0};
    std::vector<int> dash;
};

struct Polyline2D {
    std::vector<core::Point2> points;
    std::string style;
    bool closed{false};
};

struct Scene2D {
    std::string key;
    std::string title;
    std::vector<Polyline2D> polylines;

    [[nodiscard]] std::array<double, 4> bounds() const noexcept;
};

} // namespace geargen::preview
