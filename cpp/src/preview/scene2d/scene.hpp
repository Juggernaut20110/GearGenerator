#pragma once

#include "core/common/geometry_types.hpp"
#include "core/involute/involute.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace geargen::preview {

struct Style {
    std::string name;
    std::string colour{"#333333"};
    double width{1.0};
    std::vector<int> dash;
};

using LegendEntry = std::pair<std::string, std::string>;

struct Polyline2D {
    std::vector<core::Point2> points;
    std::string style;
    bool closed{false};
};

struct Scene2D {
    std::string key;
    std::string title;
    std::vector<Polyline2D> polylines;
    std::vector<LegendEntry> legend;

    [[nodiscard]] std::array<double, 4> bounds() const noexcept;
};

[[nodiscard]] const Style& style_for(const std::string& name) noexcept;

[[nodiscard]] std::vector<core::Point2> drawn_points(
    const Polyline2D& polyline);

[[nodiscard]] core::Point2 polar(double radius, double angle_rad) noexcept;
[[nodiscard]] std::vector<core::Point2> arc(double radius, double start_rad,
                                             double end_rad,
                                             int points = 64);
[[nodiscard]] std::vector<core::Point2> circle(double radius,
                                                int points = 181);
[[nodiscard]] std::vector<core::Point2> rotate(
    const std::vector<core::Point2>& points, double angle_rad);
[[nodiscard]] std::vector<core::Point2> join_pieces(
    const std::vector<std::vector<core::Point2>>& pieces,
    double tolerance = 1e-9);

// These two boundaries are shared by every involute-based gear family. The
// core supplies named tooth-space segments; preview decides whether to show
// the cut boundary or the material tooth left between adjacent spaces.
[[nodiscard]] std::vector<core::Point2> space_boundary(
    const std::vector<core::involute::NamedSegment>& segments,
    int tip_points = 9);
[[nodiscard]] std::vector<core::Point2> tooth_boundary(
    const std::vector<core::involute::NamedSegment>& segments,
    double pitch_step_rad, int arc_points = 9);

[[nodiscard]] double nice_length(double pixels_per_mm,
                                  double target_pixels = 110.0) noexcept;

// Canvas coordinates are pixels; world coordinates are engineering mm. The
// only axis convention here is the view boundary: +Y in engineering space
// maps to decreasing canvas Y. Pan is in pixels and zoom is dimensionless.
struct View2D {
    double scale{1.0};
    double origin_x{};
    double origin_y{};

    [[nodiscard]] core::Point2 to_canvas(core::Point2 point) const noexcept;
    [[nodiscard]] core::Point2 from_canvas(core::Point2 point) const noexcept;
    [[nodiscard]] std::vector<double> flatten(
        const std::vector<core::Point2>& points) const;
    [[nodiscard]] static View2D fit(const std::array<double, 4>& bounds,
                                     double width, double height,
                                     double margin = 20.0, double zoom = 1.0,
                                     core::Point2 pan = {});
};

struct Row {
    std::string label;
    std::string pinion;
    std::string gear;
    std::string unit;
    bool header{false};
    std::string third;
};

[[nodiscard]] std::string format_number(double value, int places = 4);

} // namespace geargen::preview
