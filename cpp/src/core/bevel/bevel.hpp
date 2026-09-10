#pragma once

#include "core/common/geometry_types.hpp"
#include "core/common/parameters.hpp"
#include "core/involute/involute.hpp"

#include <optional>
#include <string>
#include <vector>

namespace geargen::core::bevel {

struct CrownTrace {
    double cutter_radius_mm{};
    double centre_distance_mm{};
    double mean_cone_dist_mm{};
    double sign{1.0};

    [[nodiscard]] static CrownTrace for_set(double psi_m_rad,
                                             double cutter_radius_mm,
                                             double mean_cone_dist_mm,
                                             double sign = 1.0);
    [[nodiscard]] double spiral_angle_at(double cone_dist_mm) const;
    [[nodiscard]] double theta_at(double cone_dist_mm) const;
    [[nodiscard]] bool reaches(double inner_mm, double outer_mm) const noexcept;

private:
    [[nodiscard]] double theta_raw(double cone_dist_mm) const;
};

struct MemberGeometry {
    std::string name;
    int z{};
    double pitch_angle_rad{};
    double pitch_dia_mm{};
    double addendum_mm{};
    double dedendum_mm{};
    double addendum_angle_rad{};
    double dedendum_angle_rad{};
    double face_angle_rad{};
    double root_angle_rad{};
    double virtual_teeth{};
    double virtual_pitch_r_mm{};
    double virtual_base_r_mm{};
    double virtual_tip_r_mm{};
    double virtual_root_r_mm{};
    double virtual_tip_r_inner_mm{};
    double outside_dia_mm{};
    double crown_to_apex_mm{};
    double mounting_distance_mm{};
    double outer_root_radius_mm{};
    double root_to_apex_mm{};

    [[nodiscard]] double pitch_angle_deg() const noexcept;
    [[nodiscard]] double face_angle_deg() const noexcept;
    [[nodiscard]] double root_angle_deg() const noexcept;
    [[nodiscard]] double angular_pitch_rad() const noexcept;
};

struct SetGeometry {
    BevelSetParams parameters;
    double outer_cone_dist_mm{};
    double mean_cone_dist_mm{};
    double inner_cone_dist_mm{};
    double section_scale{};
    double working_depth_mm{};
    double whole_depth_mm{};
    double clearance_mm{};
    double circular_pitch_mm{};
    MemberGeometry pinion;
    MemberGeometry gear;
    std::optional<CrownTrace> trace;

    [[nodiscard]] const MemberGeometry& member(const std::string& which) const;
    [[nodiscard]] double face_contact_ratio() const noexcept;
};

struct ToothSpaceSection {
    std::string member;
    std::string end;
    double cone_apex_z_mm{};
    double pitch_angle_rad{};
    double r_root_mm{};
    double r_tip_mm{};
    double r_cap_mm{};
    bool filleted{};
    double cone_dist_mm{};
    double phase_rad{};
    involute::ToothSpaceLoop profile;

    [[nodiscard]] const std::vector<Point2>& loop_2d() const noexcept
    {
        return profile.loop;
    }
    [[nodiscard]] std::vector<Point3> loop_3d() const;
};

[[nodiscard]] BevelSetParams default_parameters(double module_mm, int z1, int z2,
                                                  bool zerol = false);
[[nodiscard]] SetGeometry derive(const BevelSetParams& parameters);
[[nodiscard]] double phase_at_cone_distance(const SetGeometry& geo,
                                             const std::string& member,
                                             double cone_dist_mm);
[[nodiscard]] double tip_radius_at_cone_distance(double cone_dist_mm,
                                                  double pitch_angle_rad,
                                                  double face_angle_rad,
                                                  double crown_radius_mm,
                                                  double crown_to_apex_mm);
[[nodiscard]] Point3 to_cone_3d(double x, double y, double delta_rad,
                                double cone_apex_z_mm,
                                double phase_rad = 0.0) noexcept;
[[nodiscard]] ToothSpaceSection tooth_space_section(
    const SetGeometry& geo, const std::string& member,
    const std::string& end = "outer", int flank_points = involute::kFlankPoints,
    double overshoot_mm = 0.0,
    std::optional<double> cone_dist_mm = std::nullopt,
    bool split_cap = false);
[[nodiscard]] std::pair<double, double> section_span(const SetGeometry& geo,
                                                     const std::string& member);
[[nodiscard]] double end_overshoot(const SetGeometry& geo,
                                   const std::string& member,
                                   std::optional<double> margin_mm = std::nullopt);
[[nodiscard]] std::vector<double> section_cone_distances(
    const SetGeometry& geo, const std::string& member,
    double max_sagitta_mm = 0.02);
[[nodiscard]] int section_count(const SetGeometry& geo, const std::string& member,
                                double max_sagitta_mm = 0.02);
[[nodiscard]] std::vector<Point3> guide_spiral(const SetGeometry& geo,
                                               const std::string& member,
                                               double a_hi_mm, double a_lo_mm,
                                               int points = 61);
[[nodiscard]] std::vector<Point2> blank_outline(const SetGeometry& geo,
                                                const std::string& member);

} // namespace geargen::core::bevel
