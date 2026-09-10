#pragma once

#include "core/common/geometry_types.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace geargen::core::involute {

inline constexpr int kFlankPoints = 40;
inline constexpr double kMinTopLandFactor = 0.05;
inline constexpr double kCutOvershootFactor = 0.5;

struct ProfilePoint {
    Point2 point{};
    double roll_parameter{};
};

struct StartOfInvolute {
    double start_of_involute_r{};
    double start_of_involute_angle{};
    double involute_roll_parameter{};
    double trochoid_parameter{};
    bool undercut{};
    Point2 point{};
};

struct RackGeneratedRoot {
    double reference_r{};
    double r_base{};
    double r_root{};
    double psi0{};
    double half_pitch{};
    double cutter_tip_depth{};
    double rack_root_radius{};
    double alpha{};
    double pitch_space_angle{};
    double phi_root{};
    double phi_flank_transition{};
    double v_centre{};
    double u_centre{};

    [[nodiscard]] double generated_root_r() const noexcept
    {
        return reference_r - cutter_tip_depth;
    }
    [[nodiscard]] bool undercut() const noexcept;
    [[nodiscard]] double base_space_angle() const noexcept
    {
        return half_pitch - psi0;
    }
    [[nodiscard]] Point2 point(double phi) const;
    [[nodiscard]] std::pair<double, double> polar(double phi) const;
    [[nodiscard]] Point2 nominal_involute_point(double roll) const;
    [[nodiscard]] std::pair<Point2, double> nominal_involute_at_radius(
        double radius) const;
    [[nodiscard]] std::vector<Point2> sample_to(double phi_end, int n) const;
};

[[nodiscard]] double involute_function(double pressure_angle_rad) noexcept;
[[nodiscard]] double inv(double angle_rad) noexcept;
[[nodiscard]] Point2 polar(double radius, double angle_rad) noexcept;
[[nodiscard]] ProfilePoint external_point(double base_radius_mm,
                                          double roll_parameter,
                                          double base_space_angle_rad);

[[nodiscard]] double top_land(double radius, double base_radius, double psi0);
[[nodiscard]] double max_tip_radius(double base_radius, double psi0,
                                    double min_land);
[[nodiscard]] double internal_space_width(double radius, double base_radius,
                                          double psi0);
[[nodiscard]] double internal_tooth_width(double radius, double base_radius,
                                          double psi0, double half_pitch);
[[nodiscard]] double min_internal_tip_radius(double base_radius, double psi0,
                                              double half_pitch,
                                              double min_land);

[[nodiscard]] std::vector<Point2> flank_points(double base_radius,
                                               double root_radius,
                                               double tip_radius, double psi0,
                                               double half_pitch, int count);
[[nodiscard]] std::vector<Point2> internal_flank_points(double base_radius,
                                                         double root_radius,
                                                         double tip_radius,
                                                         double psi0,
                                                         int count);

[[nodiscard]] std::optional<RackGeneratedRoot> rack_generated_root(
    double reference_radius, double base_radius, double root_radius,
    double psi0, double half_pitch, double cutter_tip_depth,
    double rack_root_radius);
[[nodiscard]] std::optional<StartOfInvolute> solve_start_of_involute(
    const RackGeneratedRoot& root);

struct RackRootEnvelope {
    std::vector<Point2> points;
    double transition_radius{};
};

[[nodiscard]] std::optional<RackRootEnvelope> rack_root_envelope(
    double reference_radius, double base_radius, double root_radius,
    double psi0, double half_pitch, double cutter_tip_depth,
    double rack_root_radius, int count = 24);

struct FilletResult {
    std::vector<Point2> flank;
    std::vector<Point2> arc;
};

[[nodiscard]] std::optional<FilletResult> root_fillet(
    const std::vector<Point2>& flank, double root_radius, double fillet_radius,
    int arc_points = 9, bool internal = false);

struct NamedSegment {
    std::string name;
    std::vector<Point2> points;
};

struct ToothSpaceLoop {
    std::vector<NamedSegment> segments;
    std::vector<Point2> loop;
    bool filleted{};

    [[nodiscard]] const std::vector<Point2>* find_segment(
        const std::string& name) const noexcept;
};

[[nodiscard]] ToothSpaceLoop tooth_space_loop(
    double base_radius, double root_radius, double tip_radius,
    double cap_radius, double psi0, double half_pitch, double fillet_radius,
    int flank_points = kFlankPoints, bool split_cap = false,
    bool internal = false,
    const std::optional<RackRootEnvelope>& rack_root = std::nullopt);

} // namespace geargen::core::involute
