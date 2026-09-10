#pragma once

#include "core/common/geometry_types.hpp"
#include "core/common/parameters.hpp"
#include "core/involute/involute.hpp"

#include <optional>
#include <string>
#include <vector>

namespace geargen::core::hypoid {

inline constexpr int kMethod1MaxIterations = 80;
inline constexpr double kMethod1CurvatureToleranceMm = 1e-9;

struct ThicknessGeometry {
    double mean_normal_pressure_angle_rad{};
    double theoretical_thickness_modification{};
    double backlash_thickness_modification{};
    double pinion_thickness_modification{};
    double gear_thickness_modification{};
    double outer_transverse_backlash_mm{};
    double mean_transverse_backlash_mm{};
    double mean_normal_backlash_mm{};
};

struct Method1Geometry {
    double gear_ratio{};
    double desired_pinion_spiral_angle_rad{};
    double shaft_angle_departure_rad{};
    double preliminary_wheel_pitch_angle_rad{};
    double preliminary_wheel_mean_radius_mm{};
    double preliminary_pinion_offset_angle_rad{};
    double preliminary_dimension_factor{};
    double preliminary_pinion_mean_radius_mm{};
    double wheel_offset_angle_axial_rad{};
    double intermediate_pinion_offset_angle_axial_rad{};
    double intermediate_pinion_pitch_angle_rad{};
    double intermediate_pinion_offset_angle_pitch_rad{};
    double intermediate_pinion_spiral_angle_rad{};
    double dimension_factor_increment{};
    double pinion_mean_radius_increment_mm{};
    double pinion_offset_angle_axial_rad{};
    double pinion_offset_angle_pitch_rad{};
    double pinion_spiral_angle_rad{};
    double wheel_spiral_angle_rad{};
    double pinion_pitch_angle_rad{};
    double wheel_pitch_angle_rad{};
    double pinion_mean_radius_mm{};
    double wheel_mean_radius_mm{};
    double pinion_mean_cone_distance_mm{};
    double wheel_mean_cone_distance_mm{};
    double pitch_plane_offset_mm{};
    double limit_pressure_angle_rad{};
    double generated_drive_normal_pressure_angle_rad{};
    double generated_coast_normal_pressure_angle_rad{};
    std::optional<double> limit_radius_of_curvature_mm;
    std::optional<double> mean_tooth_curvature_mm;
    std::optional<double> curvature_residual_mm;
    int iterations{};

    std::optional<double> wheel_face_width_factor;
    std::optional<double> wheel_outer_face_width_mm;
    std::optional<double> wheel_inner_face_width_mm;
    std::optional<double> pinion_face_width_mm;
    std::optional<double> pinion_face_width_increment_along_axis_mm;
    std::optional<double> pinion_outer_face_width_mm;
    std::optional<double> pinion_inner_face_width_mm;
    std::optional<double> pinion_boundary_wheel_outer_cone_distance_mm;
    std::optional<double> pinion_boundary_wheel_inner_cone_distance_mm;
    std::optional<double> pinion_inner_spiral_angle_rad;
    std::optional<double> pinion_outer_spiral_angle_rad;
    std::optional<double> wheel_inner_spiral_angle_rad;
    std::optional<double> wheel_outer_spiral_angle_rad;
    std::optional<double> crossing_to_wheel_mean_z_mm;
    std::optional<double> crossing_to_pinion_mean_z_mm;
    std::optional<double> wheel_pitch_apex_z_mm;
    std::optional<double> pinion_pitch_apex_z_mm;
    std::optional<double> wheel_face_apex_z_mm;
    std::optional<double> wheel_root_apex_z_mm;
    std::optional<double> pinion_face_apex_z_mm;
    std::optional<double> pinion_root_apex_z_mm;
    std::optional<double> pinion_root_plane_offset_angle_rad;
    std::optional<double> pinion_face_plane_offset_angle_rad;
    std::optional<double> pinion_face_width_auxiliary_angle_rad;
};

struct MemberGeometry {
    std::string name;
    int z{};
    double pitch_angle_rad{};
    double pitch_radius_mm{};
    double cone_distance_mm{};
    double outer_cone_distance_mm{};
    double tooth_face_inner_cone_distance_mm{};
    double tooth_face_outer_cone_distance_mm{};
    double mean_spiral_angle_rad{};
    double inner_spiral_angle_rad{};
    double outer_spiral_angle_rad{};
    double generated_drive_normal_pressure_angle_rad{};
    double generated_coast_normal_pressure_angle_rad{};
    double addendum_mm{};
    double dedendum_mm{};
    double working_depth_mm{};
    double clearance_mm{};
    double whole_depth_mm{};
    double addendum_angle_rad{};
    double dedendum_angle_rad{};
    double face_angle_rad{};
    double root_angle_rad{};
    double face_width_mm{};
    double face_width_along_pitch_cone_mm{};
    double outer_face_width_mm{};
    double inner_face_width_mm{};
    double inner_cone_distance_mm{};
    double pitch_apex_z_mm{};
    double mean_pitch_z_mm{};
    double face_apex_z_mm{};
    double root_apex_z_mm{};
    double inner_tip_z_mm{};
    double outer_tip_z_mm{};
    double inner_root_z_mm{};
    double outer_root_z_mm{};
    double outer_pitch_diameter_mm{};
    double inner_pitch_diameter_mm{};
    double outer_tip_diameter_mm{};
    double inner_tip_diameter_mm{};
    double outer_addendum_mm{};
    double inner_addendum_mm{};
    double outer_dedendum_mm{};
    double inner_dedendum_mm{};
    double outer_whole_depth_mm{};
    double inner_whole_depth_mm{};
    double mean_tip_radius_mm{};
    double mean_root_radius_mm{};
    double inner_tip_radius_mm{};
    double outer_tip_radius_mm{};
    double inner_root_radius_mm{};
    double outer_root_radius_mm{};
    double outer_root_diameter_mm{};
    double inner_root_diameter_mm{};
    double virtual_teeth{};
    double virtual_pitch_r_mm{};
    double virtual_base_r_mm{};
    double virtual_tip_r_mm{};
    double virtual_root_r_mm{};
    double thickness_modification_coefficient{};
    double mean_normal_tooth_thickness_mm{};
    double mean_transverse_tooth_thickness_mm{};
    double tredgold_tip_radius_mm{};
    double tredgold_mean_root_radius_mm{};
    double tredgold_outer_root_radius_mm{};

    [[nodiscard]] double angular_pitch_rad() const noexcept;
    [[nodiscard]] double mean_transverse_module_mm() const noexcept;
    [[nodiscard]] double outer_transverse_module_mm() const noexcept;
    [[nodiscard]] double tip_r_mm() const noexcept { return tredgold_tip_radius_mm; }
    [[nodiscard]] double root_r_mm() const noexcept { return tredgold_mean_root_radius_mm; }
};

struct SetGeometry {
    HypoidSetParams parameters;
    double outer_cone_dist_mm{};
    double mean_cone_dist_mm{};
    double inner_cone_dist_mm{};
    double mean_normal_module_mm{};
    double basic_addendum_factor{};
    double basic_dedendum_factor{};
    double method1_profile_shift_coefficient{};
    double mean_working_depth_mm{};
    double mean_clearance_mm{};
    double mean_whole_depth_mm{};
    double offset_angle_rad{};
    double pitch_plane_offset_mm{};
    Method1Geometry method1;
    ThicknessGeometry thickness;
    MemberGeometry pinion;
    MemberGeometry gear;

    [[nodiscard]] const MemberGeometry& member(const std::string& which) const;
    [[nodiscard]] double circular_pitch_mm() const noexcept;
    [[nodiscard]] double working_depth_mm() const noexcept { return mean_working_depth_mm; }
    [[nodiscard]] double clearance_mm() const noexcept { return mean_clearance_mm; }
    [[nodiscard]] double whole_depth_mm() const noexcept { return mean_whole_depth_mm; }
};

struct Section {
    std::string member;
    double cone_dist_mm{};
    double phase_rad{};
    double pitch_angle_rad{};
    double cone_apex_z_mm{};
    double r_root_mm{};
    double r_tip_mm{};
    double r_cap_mm{};
    double spiral_angle_rad{};
    double normal_tooth_thickness_mm{};
    double transverse_tooth_thickness_mm{};
    double drive_transverse_pressure_angle_rad{};
    double coast_transverse_pressure_angle_rad{};
    double drive_base_radius_mm{};
    double coast_base_radius_mm{};
    double root_fillet_radius_mm{};
    bool filleted{};
    std::vector<Point2> drive_flank;
    std::vector<Point2> coast_flank;
    std::vector<involute::NamedSegment> segments;
    std::vector<Point2> loop;

    [[nodiscard]] std::vector<Point3> loop_3d() const;
};

struct SectionBounds {
    double calculation_point_mm{};
    double tooth_face_inner_mm{};
    double tooth_face_outer_mm{};
    double loft_inner_mm{};
    double loft_outer_mm{};
};

[[nodiscard]] HypoidSetParams default_parameters(double module_mm, int z1, int z2);
[[nodiscard]] SetGeometry derive(const HypoidSetParams& parameters);
[[nodiscard]] double normal_to_transverse_pressure_angle(
    double normal_pressure_angle_rad, double spiral_angle_rad);
[[nodiscard]] std::pair<double, double> contact_azimuths(const SetGeometry& geo);
[[nodiscard]] double phase(const SetGeometry& geo, const std::string& member,
                           double cone_dist_mm, bool construction_only = false);
[[nodiscard]] Section tooth_space_section(
    const SetGeometry& geo, const std::string& member,
    std::optional<double> cone_dist_mm = std::nullopt,
    bool split_cap = false, int flank_points = involute::kFlankPoints);
[[nodiscard]] SectionBounds section_cone_bounds(const SetGeometry& geo,
                                                const std::string& member);
[[nodiscard]] std::vector<double> section_cone_distances(
    const SetGeometry& geo, const std::string& member, int count = 8);
[[nodiscard]] int section_count(const SetGeometry& geo, const std::string& member);
[[nodiscard]] std::vector<Point2> blank_outline(const SetGeometry& geo,
                                                const std::string& member);

} // namespace geargen::core::hypoid
