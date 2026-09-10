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
#include "preview/scene2d/bevel_scene.hpp"
#include "preview/scene2d/hypoid_scene.hpp"
#include "preview/scene2d/planetary_scene.hpp"
#include "preview/scene2d/scene.hpp"
#include "preview/scene2d/spur_scene.hpp"
#include "preview/scene3d/assembly.hpp"
#include "preview/scene3d/scene.hpp"
#include "solidworks/solidworks.hpp"

#include <cmath>
#include <algorithm>
#include <filesystem>
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
    const auto view = geargen::preview::View2D::fit({0.0, 0.0, 10.0, 5.0},
                                                     200.0, 100.0);
    ok &= check(std::abs(view.scale - 12.0) < 1e-12 &&
                    distance(view.to_canvas({0.0, 0.0}), {40.0, 80.0}) < 1e-12 &&
                    distance(
                        view.from_canvas(view.to_canvas({5.5, 1.25})),
                        {5.5, 1.25}) < 1e-12,
                "2D view fit, Y inversion, and inverse conversion match Python");

    const auto spur_transverse = geargen::preview::spur::build_scene(
        standard_geometry, "pinion", "transverse");
    const auto spur_twist = geargen::preview::spur::build_scene(
        standard_geometry, "gear", "twist");
    const auto spur_blank = geargen::preview::spur::build_scene(
        standard_geometry, "pinion", "blank");
    ok &= check(spur_transverse.polylines.size() > 5 &&
                    spur_twist.polylines.size() > 10 &&
                    spur_blank.polylines.size() == 6,
                "spur preview exposes transverse, twist, and blank scenes");
    const auto bevel_developed = geargen::preview::bevel::build_scene(
        bevel_straight, "pinion", "developed");
    const auto bevel_axial = geargen::preview::bevel::build_scene(
        bevel_straight, "gear", "axial");
    const auto bevel_blank = geargen::preview::bevel::build_scene(
        bevel_straight, "pinion", "blank");
    ok &= check(!bevel_developed.polylines.empty() &&
                    !bevel_axial.polylines.empty() &&
                    bevel_blank.polylines.size() == 7,
                "bevel preview exposes developed, axial, and blank geometry");
    const auto hypoid_contact = geargen::preview::hypoid::build_scene(
        hypoid_geometry, "pinion", "contact");
    const auto hypoid_section_scene = geargen::preview::hypoid::build_scene(
        hypoid_geometry, "gear", "section");
    const auto hypoid_blank = geargen::preview::hypoid::build_scene(
        hypoid_geometry, "gear", "blank");
    ok &= check(!hypoid_contact.polylines.empty() &&
                    !hypoid_section_scene.polylines.empty() &&
                    !hypoid_blank.polylines.empty(),
                "hypoid preview exposes contact, section, and blank scenes");
    const auto train = geargen::preview::planetary::build_scene(
        planetary_geometry, "sun", "train");
    ok &= check(train.polylines.size() >
                    static_cast<std::size_t>(planetary.n_planets),
                "planetary preview places sun, planets, ring, and references");
    const auto scene3d_spur = geargen::preview::scene3d::build_scene(standard_geometry);
    const auto scene3d_bevel = geargen::preview::scene3d::build_scene(bevel_straight);
    const auto scene3d_hypoid = geargen::preview::scene3d::build_scene(hypoid_geometry);
    const auto scene3d_planetary = geargen::preview::scene3d::build_scene(planetary_geometry);
    ok &= check(!scene3d_spur.polylines.empty() && !scene3d_bevel.polylines.empty() &&
                    !scene3d_hypoid.polylines.empty() && !scene3d_planetary.polylines.empty() &&
                    scene3d_spur.bounds()[3] > scene3d_spur.bounds()[0],
                "3D assembly scenes are available for every gear family");
    geargen::preview::Camera camera;
    camera.front();
    ok &= check(distance(camera.project({1.0, 2.0, 3.0}, 100.0, 80.0),
                         {51.0, 38.0}) < 1e-12,
                "3D camera front projection matches Python convention");

    const auto r12 = geargen::preview::exporter::dxf_lines(spur_transverse);
    ok &= check(std::find(r12.begin(), r12.end(), "AC1009") != r12.end() &&
                    std::find(r12.begin(), r12.end(), "POLYLINE") != r12.end() &&
                    std::find(r12.begin(), r12.end(), "LWPOLYLINE") == r12.end(),
                "DXF export uses the Python-compatible R12 polyline format");
    ok &= check(geargen::preview::spur::csv_lines(helical_geometry, "pinion").front() ==
                    "member,section,index,x,y,z" &&
                    geargen::preview::bevel::csv_lines(bevel_straight, "pinion").front() ==
                    "member,end,loop,index,x_dev,y_dev,x,y,z" &&
                    geargen::preview::hypoid::csv_lines(hypoid_geometry, "pinion").front() ==
                    "member,section_kind,index,x_dev,y_dev,x,y,z" &&
                    geargen::preview::planetary::csv_lines(planetary_geometry, "sun").front() ==
                    "member,index,x,y,z",
                "CSV exporters preserve each Python header and coordinate contract");

    const auto preset_path = std::filesystem::temp_directory_path() /
                             "geargen_native_preset_test.json";
    save_preset(preset_path, shifted_params);
    const auto loaded_spur = load_spur_preset(preset_path);
    ok &= check(loaded_spur.z1 == shifted_params.z1 &&
                    std::abs(loaded_spur.profile_shift_1 - shifted_params.profile_shift_1) < 1e-12,
                "JSON preset save/load preserves typed spur overrides");
    std::filesystem::remove(preset_path);
    ok &= check(std::abs(geargen::solidworks::millimeters_to_meters(2.0) - 0.002) < 1e-12,
                "SOLIDWORKS unit boundary converts millimetres");

    return ok ? 0 : 1;
}
