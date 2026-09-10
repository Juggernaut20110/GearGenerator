#include "preview/scene2d/scene.hpp"

#include "core/common/numerics.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

namespace geargen::preview {

namespace {
const Style kDefault{"default", "#333333", 1.0, {}};
const std::vector<Style> kStyles{
    {"outer", "#1b6fd4", 2.0, {}}, {"inner", "#d4611b", 1.6, {}},
    {"neighbour", "#9db4c8", 1.0, {}}, {"cut", "#b9a0d0", 1.0, {3, 3}},
    {"pitch", "#2f8f4f", 1.2, {7, 3, 2, 3}},
    {"working", "#b56b1f", 1.2, {2, 2}},
    {"reference", "#9a9a9a", 1.0, {4, 3}},
    {"axis", "#7a7a7a", 1.0, {8, 3, 2, 3}},
    {"blank", "#1b6fd4", 2.0, {}}, {"cone", "#c0392b", 1.0, {6, 3}},
    {"scale", "#4a4a4a", 1.4, {}},
};

const std::vector<core::Point2>* segment(
    const std::vector<core::involute::NamedSegment>& segments,
    const std::string& name)
{
    for (const auto& item : segments) {
        if (item.name == name) return &item.points;
    }
    return nullptr;
}

std::vector<core::Point2> reversed(const std::vector<core::Point2>& points)
{
    return {points.rbegin(), points.rend()};
}
} // namespace

const Style& style_for(const std::string& name) noexcept
{
    for (const auto& style : kStyles) {
        if (style.name == name) return style;
    }
    return kDefault;
}

std::vector<core::Point2> drawn_points(const Polyline2D& polyline)
{
    auto result = polyline.points;
    if (polyline.closed && result.size() > 2U) result.push_back(result.front());
    return result;
}

std::array<double, 4> Scene2D::bounds() const noexcept
{
    double xmin = std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto& line : polylines) {
        for (const auto& point : line.points) {
            xmin = std::min(xmin, point.x); ymin = std::min(ymin, point.y);
            xmax = std::max(xmax, point.x); ymax = std::max(ymax, point.y);
        }
    }
    if (!std::isfinite(xmin)) return {-1.0, -1.0, 1.0, 1.0};
    return {xmin, ymin, xmax, ymax};
}

core::Point2 polar(double radius, double angle_rad) noexcept
{
    return {radius * std::cos(angle_rad), radius * std::sin(angle_rad)};
}

std::vector<core::Point2> arc(double radius, double start_rad, double end_rad,
                              int points)
{
    points = std::max(2, points);
    std::vector<core::Point2> result;
    result.reserve(static_cast<std::size_t>(points));
    for (int i = 0; i < points; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(points - 1);
        result.push_back(polar(radius, start_rad + t * (end_rad - start_rad)));
    }
    return result;
}

std::vector<core::Point2> circle(double radius, int points)
{
    return arc(radius, 0.0, core::kTau, points);
}

std::vector<core::Point2> rotate(const std::vector<core::Point2>& points,
                                 double angle_rad)
{
    std::vector<core::Point2> result;
    result.reserve(points.size());
    for (const auto point : points) result.push_back(core::rotate(point, angle_rad));
    return result;
}

std::vector<core::Point2> join_pieces(
    const std::vector<std::vector<core::Point2>>& pieces, double tolerance)
{
    std::vector<core::Point2> result;
    for (const auto& piece : pieces) {
        for (const auto point : piece) {
            if (result.empty() || core::distance(result.back(), point) > tolerance)
                result.push_back(point);
        }
    }
    if (result.size() > 1U && core::distance(result.front(), result.back()) < tolerance)
        result.pop_back();
    return result;
}

std::vector<core::Point2> space_boundary(
    const std::vector<core::involute::NamedSegment>& segments, int tip_points)
{
    const auto* flank_pos = segment(segments, "flank_pos");
    if (flank_pos == nullptr || flank_pos->empty()) return {};
    const auto tip = flank_pos->front();
    const double tip_angle = std::atan2(tip.y, tip.x);
    const auto get = [&segments](const char* name) {
        const auto* value = segment(segments, name);
        return value == nullptr ? std::vector<core::Point2>{} : *value;
    };
    return join_pieces({get("fillet_neg"), get("flank_neg"),
                        arc(core::norm(tip), -tip_angle, tip_angle, tip_points),
                        *flank_pos, get("fillet_pos"), get("root")});
}

std::vector<core::Point2> tooth_boundary(
    const std::vector<core::involute::NamedSegment>& segments,
    double pitch_step_rad, int arc_points)
{
    const auto* flank_pos = segment(segments, "flank_pos");
    const auto* root = segment(segments, "root");
    if (flank_pos == nullptr || flank_pos->empty() || root == nullptr || root->empty())
        return {};
    const auto tip = flank_pos->front();
    const auto root_point = root->front();
    const double tip_radius = core::norm(tip);
    const double tip_angle = std::atan2(tip.y, tip.x);
    const double root_radius = core::norm(root_point);
    const double root_angle = std::atan2(root_point.y, root_point.x);
    const auto turn = [pitch_step_rad](const auto* points) {
        if (points == nullptr) return std::vector<core::Point2>{};
        return rotate(reversed(*points), pitch_step_rad);
    };
    const auto empty = std::vector<core::Point2>{};
    const auto* fillet_pos = segment(segments, "fillet_pos");
    const auto* flank_neg = segment(segments, "flank_neg");
    const auto* fillet_neg = segment(segments, "fillet_neg");
    return join_pieces({fillet_pos == nullptr ? empty : reversed(*fillet_pos),
                        reversed(*flank_pos),
                        arc(tip_radius, tip_angle, pitch_step_rad - tip_angle, arc_points),
                        turn(flank_neg), turn(fillet_neg),
                        arc(root_radius, pitch_step_rad - root_angle, root_angle, arc_points)});
}

double nice_length(double pixels_per_mm, double target_pixels) noexcept
{
    const double raw = target_pixels / std::max(pixels_per_mm, 1e-9);
    const double exponent = raw > 0.0 ? std::floor(std::log10(raw)) : 0.0;
    for (const double multiplier : {1.0, 2.0, 5.0}) {
        const double candidate = multiplier * std::pow(10.0, exponent);
        if (candidate >= raw) return candidate;
    }
    return std::pow(10.0, exponent + 1.0);
}

core::Point2 View2D::to_canvas(core::Point2 point) const noexcept
{
    return {origin_x + point.x * scale, origin_y - point.y * scale};
}

core::Point2 View2D::from_canvas(core::Point2 point) const noexcept
{
    return {(point.x - origin_x) / scale, (origin_y - point.y) / scale};
}

std::vector<double> View2D::flatten(const std::vector<core::Point2>& points) const
{
    std::vector<double> result;
    result.reserve(points.size() * 2U);
    for (const auto point : points) {
        const auto canvas = to_canvas(point);
        result.push_back(canvas.x); result.push_back(canvas.y);
    }
    return result;
}

View2D View2D::fit(const std::array<double, 4>& bounds, double width,
                   double height, double margin, double zoom, core::Point2 pan)
{
    const double span_x = std::max(bounds[2] - bounds[0], 1e-6);
    const double span_y = std::max(bounds[3] - bounds[1], 1e-6);
    const double avail_x = std::max(width - 2.0 * margin, 1.0);
    const double avail_y = std::max(height - 2.0 * margin, 1.0);
    const double scale = std::min(avail_x / span_x, avail_y / span_y) *
                         std::max(zoom, 1e-6);
    const double centre_x = 0.5 * (bounds[0] + bounds[2]);
    const double centre_y = 0.5 * (bounds[1] + bounds[3]);
    return {scale, 0.5 * width - centre_x * scale + pan.x,
            0.5 * height + centre_y * scale + pan.y};
}

std::string format_number(double value, int places)
{
    if (std::isinf(value)) return value < 0.0 ? "-inf" : "inf";
    if (std::isnan(value)) return "nan";
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(places) << value;
    return stream.str();
}

} // namespace geargen::preview
