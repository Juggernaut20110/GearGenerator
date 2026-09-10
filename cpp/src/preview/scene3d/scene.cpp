#include "preview/scene3d/scene.hpp"

#include "core/common/numerics.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace geargen::preview {

namespace {
const Style3D kDefault{"default", "#333333", 1.0, {}};
const std::vector<Style3D> kStyles{
    {"pinion", "#1769aa", 1.8, {}}, {"gear", "#c45a16", 1.8, {}},
    {"sun", "#1769aa", 1.8, {}}, {"planet", "#6f8391", 1.6, {}},
    {"ring", "#b53d3d", 1.8, {}}, {"reference", "#777777", 1.0, {5, 3}},
    {"axis", "#555555", 1.0, {8, 3, 2, 3}},
    {"pitch", "#2f8f4f", 1.0, {7, 3, 2, 3}},
};
} // namespace

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
            xmin = std::min(xmin, point.x); ymin = std::min(ymin, point.y);
            zmin = std::min(zmin, point.z); xmax = std::max(xmax, point.x);
            ymax = std::max(ymax, point.y); zmax = std::max(zmax, point.z);
        }
    }
    if (!std::isfinite(xmin)) return {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0};
    return {xmin, ymin, zmin, xmax, ymax, zmax};
}

std::vector<core::Point3> drawn_points(const Polyline3D& polyline)
{
    auto result = polyline.points;
    if (polyline.closed && result.size() > 2U) result.push_back(result.front());
    return result;
}

const Style3D& style_for_3d(const std::string& name) noexcept
{
    for (const auto& style : kStyles) if (style.name == name) return style;
    return kDefault;
}

std::array<double, 3> OrthographicCamera::camera_coordinates(
    core::Point3 point) const noexcept
{
    const double x = point.x - target_.x;
    const double y = point.y - target_.y;
    const double z = point.z - target_.z;
    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    const double right = cy * x + sy * z;
    double depth = -sy * x + cy * z;
    const double cp = std::cos(pitch_);
    const double sp = std::sin(pitch_);
    const double up = cp * y - sp * depth;
    depth = sp * y + cp * depth;
    return {right, up, depth};
}

core::Point2 OrthographicCamera::project(core::Point3 point, double width,
                                         double height) const noexcept
{
    const auto coordinates = camera_coordinates(point);
    const double scale = fit_scale_ * zoom_;
    return {0.5 * width + pan_x_ + coordinates[0] * scale,
            0.5 * height + pan_y_ - coordinates[1] * scale};
}

void OrthographicCamera::orbit(double dx, double dy, double sensitivity) noexcept
{
    yaw_ += dx * sensitivity;
    pitch_ = std::clamp(pitch_ - dy * sensitivity,
                        -core::kPi / 2.0 + 0.02, core::kPi / 2.0 - 0.02);
}

void OrthographicCamera::pan(double dx, double dy) noexcept
{
    pan_x_ += dx; pan_y_ += dy;
}

void OrthographicCamera::zoom_by(double factor) noexcept
{
    if (factor <= 0.0 || !std::isfinite(factor)) return;
    zoom_ = std::clamp(zoom_ * factor, 0.05, 100.0);
}

void OrthographicCamera::zoom_at(double factor, double x, double y,
                                 double width, double height) noexcept
{
    const double old_zoom = zoom_;
    const double old_pan_x = pan_x_;
    const double old_pan_y = pan_y_;
    zoom_by(factor);
    const double actual = old_zoom == 0.0 ? 1.0 : zoom_ / old_zoom;
    const double cx = 0.5 * width;
    const double cy = 0.5 * height;
    pan_x_ = (x - cx) - actual * (x - cx - old_pan_x);
    pan_y_ = (y - cy) - actual * (y - cy - old_pan_y);
}

void OrthographicCamera::set_view(const std::string& name)
{
    std::string key = name;
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (key == "front") { yaw_ = 0.0; pitch_ = 0.0; }
    else if (key == "rear") { yaw_ = core::kPi; pitch_ = 0.0; }
    else if (key == "top") { yaw_ = 0.0; pitch_ = core::kPi / 2.0; }
    else if (key == "bottom") { yaw_ = 0.0; pitch_ = -core::kPi / 2.0; }
    else if (key == "right") { yaw_ = core::kPi / 2.0; pitch_ = 0.0; }
    else if (key == "left") { yaw_ = -core::kPi / 2.0; pitch_ = 0.0; }
    else if (key == "iso" || key == "isometric") { yaw_ = core::degrees_to_radians(45.0); pitch_ = core::degrees_to_radians(30.0); }
    else throw std::invalid_argument("unknown camera view '" + name + "'");
}

void OrthographicCamera::fit(const std::array<double, 6>& bounds, double width,
                             double height, double margin) noexcept
{
    target_ = {0.5 * (bounds[0] + bounds[3]), 0.5 * (bounds[1] + bounds[4]),
               0.5 * (bounds[2] + bounds[5])};
    std::array<double, 2> mins{std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()};
    std::array<double, 2> maxs{-std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()};
    for (const double x : {bounds[0], bounds[3]})
        for (const double y : {bounds[1], bounds[4]})
            for (const double z : {bounds[2], bounds[5]}) {
                const auto projected = camera_coordinates({x, y, z});
                mins[0] = std::min(mins[0], projected[0]);
                mins[1] = std::min(mins[1], projected[1]);
                maxs[0] = std::max(maxs[0], projected[0]);
                maxs[1] = std::max(maxs[1], projected[1]);
            }
    const double span_x = std::max(maxs[0] - mins[0], 1e-9);
    const double span_y = std::max(maxs[1] - mins[1], 1e-9);
    fit_scale_ = std::min(std::max(width - 2.0 * margin, 1.0) / span_x,
                          std::max(height - 2.0 * margin, 1.0) / span_y);
    zoom_ = 1.0; pan_x_ = pan_y_ = 0.0;
}

} // namespace geargen::preview
