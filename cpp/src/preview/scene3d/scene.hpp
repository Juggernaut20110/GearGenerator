#pragma once

#include "core/common/geometry_types.hpp"
#include "preview/scene2d/scene.hpp"

#include <array>
#include <string>
#include <vector>

namespace geargen::preview {

struct Polyline3D {
    std::vector<core::Point3> points;
    std::string style{"outer"};
    bool closed{false};
    std::string member;
};

struct Scene3D {
    std::vector<Polyline3D> polylines;
    std::string title;
    std::string key{"assembly"};
    std::vector<LegendEntry> legend;

    [[nodiscard]] std::array<double, 6> bounds() const noexcept;
};

[[nodiscard]] std::vector<core::Point3> drawn_points(
    const Polyline3D& polyline);

struct Style3D {
    std::string name;
    std::string colour{"#333333"};
    double width{1.0};
    std::vector<int> dash;
};

[[nodiscard]] const Style3D& style_for_3d(const std::string& name) noexcept;

class OrthographicCamera {
public:
    [[nodiscard]] core::Point2 project(core::Point3 point,
                                       double width = 0.0,
                                       double height = 0.0) const noexcept;
    [[nodiscard]] core::Point2 project_point(core::Point3 point,
                                             double width = 0.0,
                                             double height = 0.0) const noexcept
    { return project(point, width, height); }
    void set_scale(double scale) noexcept { fit_scale_ = scale; }
    void orbit(double dx, double dy, double sensitivity = 0.01) noexcept;
    void pan(double dx, double dy) noexcept;
    void zoom_by(double factor) noexcept;
    void zoom_at(double factor, double x, double y, double width,
                 double height) noexcept;
    void set_view(const std::string& name);
    void front() { set_view("front"); }
    void top() { set_view("top"); }
    void right() { set_view("right"); }
    void iso() { set_view("iso"); }
    void fit(const std::array<double, 6>& bounds, double width, double height,
             double margin = 20.0) noexcept;
    void fit_scene(const Scene3D& scene, double width, double height,
                   double margin = 20.0) noexcept
    { fit(scene.bounds(), width, height, margin); }

    [[nodiscard]] double yaw() const noexcept { return yaw_; }
    [[nodiscard]] double pitch() const noexcept { return pitch_; }
    [[nodiscard]] double zoom() const noexcept { return zoom_; }

private:
    [[nodiscard]] std::array<double, 3> camera_coordinates(
        core::Point3 point) const noexcept;
    core::Point3 target_{};
    double yaw_{};
    double pitch_{};
    double zoom_{1.0};
    double pan_x_{};
    double pan_y_{};
    double fit_scale_{1.0};
};

using Camera = OrthographicCamera;

} // namespace geargen::preview
