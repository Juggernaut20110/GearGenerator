#pragma once

#include "core/common/parameters.hpp"
#include "core/involute/involute.hpp"

#include <optional>
#include <string>
#include <vector>

namespace geargen::core::spur {

struct MemberGeometry {
    std::string name;
    int teeth{};
    double beta_rad{};
    double reference_radius_mm{};
    double base_radius_mm{};
    double working_radius_mm{};
    double tip_radius_mm{};
    double root_radius_mm{};
    double tip_form_radius_mm{};
    double addendum_mm{};
    double dedendum_mm{};
    double geometric_tooth_thickness_mm{};
    double reference_tooth_thickness_mm{};
    double normal_geometric_tooth_thickness_mm{};
    double normal_tooth_thickness_mm{};
    double reference_tooth_thickness_allowance_mm{};
    double working_tooth_thickness_mm{};
    double working_normal_tooth_thickness_mm{};
    double working_tooth_thickness_allowance_mm{};
    double virtual_teeth{};
    double twist_rad{};
    double psi0_rad{};
    double half_pitch_rad{};
    bool internal{};
    double profile_shift{};
    std::optional<double> generated_root_radius_mm;
    std::optional<double> root_form_radius_mm;
    std::optional<double> start_of_involute_angle_rad;
    std::optional<double> involute_roll_parameter;
    std::optional<bool> undercut;
    std::optional<double> start_active_profile_radius_mm;
    std::optional<double> active_tip_radius_mm;

    [[nodiscard]] double pitch_radius_mm() const noexcept
    {
        return reference_radius_mm;
    }
    [[nodiscard]] double tooth_thickness_mm() const noexcept
    {
        return reference_tooth_thickness_mm;
    }
    [[nodiscard]] double angular_pitch_rad() const noexcept
    {
        return kTau / static_cast<double>(teeth);
    }
    [[nodiscard]] double outside_radius_mm() const noexcept
    {
        return internal ? root_radius_mm : tip_radius_mm;
    }
};

struct SetGeometry {
    SpurParameters parameters;
    double reference_centre_distance_mm{};
    double working_centre_distance_mm{};
    double reference_pressure_angle_rad{};
    double working_pressure_angle_rad{};
    double transverse_module_mm{};
    double circular_pitch_mm{};
    double axial_pitch_mm{};
    double whole_depth_mm{};
    double tip_alteration_coefficient{};
    double working_depth_mm{};
    double tip_clearance_1_mm{};
    double tip_clearance_2_mm{};
    double minimum_tip_clearance_mm{};
    double path_of_contact_mm{};
    std::string contact_ratio_basis;
    double transverse_contact_ratio{};
    double axial_contact_ratio{};
    MemberGeometry pinion;
    MemberGeometry gear;

    [[nodiscard]] double centre_distance_mm() const noexcept
    {
        return working_centre_distance_mm;
    }
    [[nodiscard]] double total_contact_ratio() const noexcept
    {
        return transverse_contact_ratio + axial_contact_ratio;
    }
    [[nodiscard]] double working_circular_pitch() const noexcept
    {
        return kTau * pinion.working_radius_mm /
               static_cast<double>(pinion.teeth);
    }
    [[nodiscard]] double centre_distance_modification() const noexcept
    {
        return (working_centre_distance_mm - reference_centre_distance_mm) /
               parameters.module;
    }
    [[nodiscard]] const MemberGeometry& member(const std::string& which) const;
    [[nodiscard]] MemberGeometry& member(const std::string& which);
};

struct ToothSpaceSection {
    std::string member_name;
    double z_mm{};
    double phase_rad{};
    double root_radius_mm{};
    double tip_radius_mm{};
    double cap_radius_mm{};
    bool filleted{};
    std::optional<double> generated_root_radius_mm;
    std::optional<double> root_form_radius_mm;
    std::optional<double> start_of_involute_angle_rad;
    std::optional<double> involute_roll_parameter;
    std::optional<bool> undercut;
    std::vector<involute::NamedSegment> segments;
    std::vector<Point2> loop_2d;

    [[nodiscard]] bool rack_generated() const noexcept;
    [[nodiscard]] std::vector<Point3> loop_3d() const;
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<Point3>>>
    segments_3d() const;
};

inline constexpr double kAddendumFactor = 1.0;
inline constexpr double kDedendumFactor = 1.25;
inline constexpr double kWholeDepthFactor = 2.25;
inline constexpr double kMaxSectionSagittaMm = 0.02;

[[nodiscard]] SpurParameters default_parameters(double module_mm, int z1,
                                                int z2);
[[nodiscard]] SetGeometry compute_set(const SpurSetParams& parameters);
[[nodiscard]] SetGeometry derive(const SpurSetParams& parameters);

[[nodiscard]] double resolve_tip_alteration(const SpurSetParams& parameters,
                                            double working_centre_distance_mm);
[[nodiscard]] double pair_working_depth(const MemberGeometry& pinion,
                                        const MemberGeometry& gear,
                                        double working_centre_distance_mm,
                                        bool internal);
[[nodiscard]] std::pair<double, double> pair_tip_clearances(
    const MemberGeometry& pinion, const MemberGeometry& gear,
    double working_centre_distance_mm, bool internal);
[[nodiscard]] double transverse_contact_ratio(
    const MemberGeometry& pinion, const MemberGeometry& gear,
    double centre_distance_mm, double alpha_t_rad, double circular_pitch_mm,
    std::optional<double> base_pitch_angle_rad = std::nullopt);
[[nodiscard]] double min_internal_teeth(double alpha_t_rad,
                                        double addendum_factor = kAddendumFactor);
[[nodiscard]] double undercut_limit(double alpha_t_rad, double beta_rad,
                                    double profile_shift = 0.0,
                                    double addendum_factor = kAddendumFactor,
                                    std::optional<double> dedendum_factor =
                                        std::nullopt,
                                    double root_radius_factor = 0.0);

[[nodiscard]] double end_overshoot(const SetGeometry& geometry);
[[nodiscard]] int section_count(
    const SetGeometry& geometry, const std::string& member,
    double max_sagitta = kMaxSectionSagittaMm);
[[nodiscard]] std::vector<double> section_heights(
    const SetGeometry& geometry, const std::string& member,
    double max_sagitta = kMaxSectionSagittaMm);
[[nodiscard]] Point3 to_axial_3d(double x, double y, double phase_rad,
                                 double z_mm) noexcept;
[[nodiscard]] double phase_at(const SetGeometry& geometry,
                              const std::string& member, double z_mm);
[[nodiscard]] ToothSpaceSection tooth_space_section(
    const SetGeometry& geometry, const std::string& member, double z_mm = 0.0,
    int flank_points = involute::kFlankPoints, bool split_cap = false);
[[nodiscard]] std::vector<Point3> guide_helix(
    const SetGeometry& geometry, const std::string& member, double z_lo,
    double z_hi, int points = 61);
[[nodiscard]] double rim_radius(const SetGeometry& geometry,
                                const std::string& member);
[[nodiscard]] std::vector<Point2> blank_outline(const SetGeometry& geometry,
                                                const std::string& member);

} // namespace geargen::core::spur
