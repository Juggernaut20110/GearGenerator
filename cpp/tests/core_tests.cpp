#include "core/bevel/bevel.hpp"
#include "core/bevel/mesh.hpp"
#include "core/common/geometry_types.hpp"
#include "core/common/serialization.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/hypoid/mesh.hpp"
#include "core/involute/involute.hpp"
#include "core/placement/placement.hpp"
#include "core/planetary/planetary.hpp"
#include "core/planetary/mesh.hpp"
#include "core/spur/spur.hpp"
#include "core/spur/mesh.hpp"
#include "core/validation/validation.hpp"
#include "preview/export/export.hpp"
#include "preview/scene2d/scene.hpp"
#include "solidworks/solidworks.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

} // namespace

int main()
{
    using namespace geargen::core;
    using namespace geargen::core::placement;

    bool ok = true;
    const auto spur = spur::default_parameters(2.0, 17, 43);
    ok &= check(spur.face_width > 0.0 && spur.bore > 0.0,
                "spur defaults are positive");
    const auto planetary = planetary::default_parameters(2.0, 24, 18);
    ok &= check(planetary.z_ring() == 60, "planetary ring count is derived");
    const auto bevel = bevel::default_parameters(2.0, 17, 43);
    ok &= check(bevel.face_width > 0.0, "bevel defaults are positive");
    const auto hypoid = hypoid::default_parameters(170.0 / 42.0, 13, 42);
    ok &= check(hypoid.face_width > 0.0, "hypoid defaults are positive");

    ValidationResult validation;
    validate_pair_basics(2.0, 17, 43, 20.0, validation);
    ok &= check(validation.ok(), "valid anchor parameters pass basic validation");
    ValidationResult invalid;
    validate_pair_basics(-1.0, 3, 43, 30.0, invalid);
    ok &= check(!invalid.ok() && invalid.errors.size() == 3,
                "basic validation accumulates independent errors");

    ok &= check(std::abs(gear_clocking(17)) < 1e-12,
                "odd-tooth gear needs no clocking");
    ok &= check(std::abs(gear_clocking(60) - kPi / 60.0) < 1e-12,
                "even-tooth gear receives half-pitch clocking");
    ok &= check(angular_velocity_ratio(17, 43) < 0.0 &&
                    angular_velocity_ratio(17, 43, true) > 0.0,
                "external and internal mesh senses differ");

    const auto packed = to_solidworks_array(rot_z(kPi / 2.0));
    ok &= check(std::abs(packed[0]) < 1e-12 && std::abs(packed[1] - 1.0) < 1e-12,
                "SOLIDWORKS transform packing is column-major");
    ok &= check(std::abs(involute::involute_function(0.0)) < 1e-12,
                "involute primitive is available in core");
    const auto involute_point = involute::external_point(10.0, 0.5, 0.1);
    const double expected_involute_radius = 10.0 * std::sqrt(1.25);
    const double expected_involute_angle = 0.1 + 0.5 - std::atan(0.5);
    ok &= check(std::abs(norm(involute_point.point) - expected_involute_radius) <
                    1e-12 &&
                    std::abs(std::atan2(involute_point.point.y,
                                        involute_point.point.x) -
                             expected_involute_angle) < 1e-12,
                "involute point uses the analytical roll equation");

    const auto standard_geometry = spur::compute_set(spur);
    SpurDefaultOverrides shifted_overrides;
    shifted_overrides.profile_shift_1 = 0.3;
    const auto shifted_params = SpurSetParams::with_defaults(
        2.0, 20, 40, shifted_overrides);
    const auto shifted_geometry = spur::compute_set(shifted_params);
    ok &= check(shifted_geometry.working_centre_distance_mm >
                    shifted_geometry.reference_centre_distance_mm &&
                    shifted_geometry.pinion.tip_radius_mm >
                        standard_geometry.pinion.tip_radius_mm,
                "profile shift changes working distance and addendum");

    SpurDefaultOverrides backlash_overrides;
    backlash_overrides.backlash = 0.2;
    backlash_overrides.backlash_mode = "working_circumferential";
    const auto backlash_geometry = spur::compute_set(
        SpurSetParams::with_defaults(2.0, 20, 40, backlash_overrides));
    ok &= check(std::abs(backlash_geometry.working_circular_pitch() -
                             (backlash_geometry.pinion.working_tooth_thickness_mm +
                              backlash_geometry.gear.working_tooth_thickness_mm) -
                             0.2) <
                    1e-12,
                "working circumferential backlash is preserved");

    SpurDefaultOverrides internal_overrides;
    internal_overrides.internal = true;
    const auto internal_geometry = spur::compute_set(
        SpurSetParams::with_defaults(2.0, 18, 60, internal_overrides));
    const auto internal_section = spur::tooth_space_section(
        internal_geometry, "gear", 0.0, 8, true);
    ok &= check(internal_geometry.gear.internal &&
                    internal_section.loop_2d.size() > 10 &&
                    internal_section.rack_generated() == false &&
                    spur::mesh::gear_translation(internal_geometry).x < 0.0 &&
                    std::abs(spur::mesh::clocking_for(internal_geometry) -
                             kPi / 60.0) < 1e-12,
                "internal tooth space and placement use ring conventions");

    SpurDefaultOverrides helical_overrides;
    helical_overrides.helix_angle = 15.0;
    const auto helical_geometry = spur::compute_set(
        SpurSetParams::with_defaults(2.0, 17, 43, helical_overrides));
    ok &= check(helical_geometry.reference_pressure_angle_rad >
                    helical_geometry.parameters.alpha_n() &&
                    helical_geometry.gear.beta_rad < 0.0 &&
                    std::isfinite(helical_geometry.axial_pitch_mm) &&
                    spur::section_count(helical_geometry, "pinion") > 2 &&
                    std::abs(spur::phase_at(
                                 helical_geometry, "pinion",
                                 helical_geometry.parameters.face_width / 2.0) -
                             helical_geometry.pinion.twist_rad / 2.0) <
                        1e-12,
                "helical conversion and section phase follow the reference");

    SpurDefaultOverrides generated_overrides;
    generated_overrides.root_geometry = "rack_generated";
    const auto generated_geometry = spur::compute_set(
        SpurSetParams::with_defaults(2.0, 12, 43, generated_overrides));
    const auto generated_section = spur::tooth_space_section(
        generated_geometry, "pinion", 0.0, 8, true);
    ok &= check(generated_geometry.pinion.generated_root_radius_mm.has_value() &&
                    generated_geometry.pinion.root_form_radius_mm.has_value() &&
                    generated_geometry.pinion.undercut.value_or(false) &&
                    generated_section.rack_generated() &&
                    generated_section.filleted,
                "rack-generated root retains the analytical undercut envelope");

    const auto planetary_geometry = planetary::compute_set(planetary);
    ok &= check(std::abs(planetary_geometry.centre_distance_mm -
                             planetary::centre_distance_from_ring(planetary)) <
                    1e-12 &&
                    std::abs(planetary::mesh::carrier_angle(planetary_geometry, 1) -
                             kTau / 3.0) <
                        1e-12 &&
                    planetary::mesh::planet_translation(planetary_geometry, 1).y >
                        0.0 &&
                    planetary_geometry.ring.internal,
                "planetary members share centre distance and orbit conventions");

    const auto bevel_straight = bevel::derive(
        BevelSetParams::with_defaults(2.0, 25, 40));
    const auto bevel_straight_section = bevel::tooth_space_section(
        bevel_straight, "pinion", "outer", 8, 0.0, std::nullopt, true);
    ok &= check(!bevel_straight.trace.has_value() &&
                    std::abs(bevel::phase_at_cone_distance(
                        bevel_straight, "pinion", bevel_straight.mean_cone_dist_mm)) <
                        1e-12 &&
                    bevel::section_count(bevel_straight, "pinion") == 2 &&
                    bevel_straight_section.loop_2d().size() > 10,
                "straight bevel uses the zero-phase two-section construction");

    BevelDefaultOverrides spiral_overrides;
    spiral_overrides.spiral_angle = 35.0;
    const auto bevel_spiral = bevel::derive(
        BevelSetParams::with_defaults(2.0, 25, 40, false, spiral_overrides));
    const double spiral_outer = bevel::phase_at_cone_distance(
        bevel_spiral, "pinion", bevel_spiral.outer_cone_dist_mm);
    const double spiral_inner = bevel::phase_at_cone_distance(
        bevel_spiral, "pinion", bevel_spiral.inner_cone_dist_mm);
    ok &= check(bevel_spiral.trace.has_value() &&
                    std::abs(bevel_spiral.trace->spiral_angle_at(
                        bevel_spiral.mean_cone_dist_mm) -
                               bevel_spiral.parameters.psi_m()) <
                        1e-12 &&
                    spiral_outer != spiral_inner &&
                    bevel::section_count(bevel_spiral, "pinion") > 2 &&
                    bevel::mesh::gear_clocking(40) > 0.0,
                "spiral bevel retains crown trace, phase sweep, and clocking");

    const auto bevel_zerol = bevel::derive(
        BevelSetParams::with_defaults(2.0, 25, 40, true));
    ok &= check(bevel_zerol.trace.has_value() &&
                    std::abs(bevel_zerol.parameters.psi_m()) < 1e-12 &&
                    bevel::section_count(bevel_zerol, "pinion") > 2,
                "Zerol bevel remains curved even at zero mean spiral angle");

    const auto cone_point = bevel::to_cone_3d(
        10.0, 0.0, bevel_straight.pinion.pitch_angle_rad,
        bevel_straight.outer_cone_dist_mm /
            std::cos(bevel_straight.pinion.pitch_angle_rad));
    ok &= check(std::abs(norm(cone_point) - 10.0) > 0.0 &&
                    std::isfinite(cone_point.z),
                "bevel sections map from the developed plane into 3D");

    HypoidDefaultOverrides hypoid_overrides;
    hypoid_overrides.offset = 15.0;
    hypoid_overrides.face_width = 30.0;
    hypoid_overrides.spiral_angle = 50.0;
    hypoid_overrides.cutter_radius = 63.5;
    const auto hypoid_offset = HypoidSetParams::with_defaults(
        170.0 / 42.0, 13, 42, hypoid_overrides);
    const auto hypoid_geometry = hypoid::derive(hypoid_offset);
    const auto hypoid_distances = hypoid::section_cone_distances(
        hypoid_geometry, "pinion", 4);
    const auto hypoid_section = hypoid::tooth_space_section(
        hypoid_geometry, "pinion", hypoid_geometry.pinion.cone_distance_mm,
        true, 8);
    ok &= check(hypoid_geometry.method1.iterations > 0 &&
                    hypoid_geometry.method1.curvature_residual_mm.has_value() &&
                    std::abs(*hypoid_geometry.method1.curvature_residual_mm) < 1e-8 &&
                    std::abs(hypoid_geometry.pitch_plane_offset_mm) > 0.0 &&
                    hypoid_distances.size() >= 4 &&
                    hypoid_section.loop.size() > 10 &&
                    std::isfinite(hypoid::phase(
                        hypoid_geometry, "pinion",
                        hypoid_geometry.pinion.cone_distance_mm)),
                "hypoid Method 1 converges and builds a Tredgold section");
    const auto hypoid_zero = hypoid::derive(
        HypoidSetParams::with_defaults(2.0, 13, 42));
    ok &= check(std::abs(hypoid_zero.pitch_plane_offset_mm) < 1e-12 &&
                    hypoid_zero.method1.iterations == 0 &&
                    std::isfinite(hypoid::mesh::gear_clocking(hypoid_zero)) &&
                    std::isfinite(hypoid::mesh::skew_axis_distance(
                        {}, {0.0, 0.0, 1.0}, {},
                        {std::sin(hypoid_zero.parameters.sigma()), 0.0,
                         std::cos(hypoid_zero.parameters.sigma())})),
                "zero-offset hypoid retains the external cone and mesh path");
    ok &= check(std::abs(degrees_to_radians(180.0) - kPi) < 1e-15 &&
                    std::abs(radians_to_degrees(kPi) - 180.0) < 1e-12,
                "angle boundary helpers use radians internally");
    ok &= check(approximately_equal(1000.0, 1000.0 + 5e-10),
                "reference tolerance combines absolute and relative terms");
    ok &= check(parameters_json(spur).find("\"type\":\"spur\"") !=
                    std::string::npos &&
                    validation_json(validation).find("\"errors\"") !=
                        std::string::npos,
                "parameter and validation JSON helpers emit stable fields");

    geargen::preview::Scene2D scene;
    scene.polylines.push_back({{{0.0, 1.0}, {2.0, 3.0}}, "test", false});
    const auto bounds = scene.bounds();
    ok &= check(bounds[0] == 0.0 && bounds[3] == 3.0,
                "2D preview scene computes bounds");
    ok &= check(geargen::preview::exporter::dxf_lines(scene).size() > 4,
                "DXF export has a complete section");
    ok &= check(std::abs(geargen::solidworks::millimeters_to_meters(2.0) - 0.002) < 1e-12,
                "SOLIDWORKS unit boundary converts millimetres");

    return ok ? 0 : 1;
}
