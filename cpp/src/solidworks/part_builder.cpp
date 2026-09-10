#include "solidworks/part_builder.hpp"

#ifdef _WIN32

#include "core/common/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace geargen::solidworks::detail {

namespace {

using Segment = core::involute::NamedSegment;

const std::vector<Segment>& section_segments(const core::spur::ToothSpaceSection& section)
{
    return section.segments;
}

const std::vector<Segment>& section_segments(const core::bevel::ToothSpaceSection& section)
{
    return section.profile.segments;
}

const std::vector<Segment>& section_segments(const core::hypoid::Section& section)
{
    return section.segments;
}

const std::vector<core::Point2>& segment_points(const std::vector<Segment>& segments,
                                                std::string_view name)
{
    for (const auto& segment : segments)
        if (segment.name == name) return segment.points;
    throw SwError("section has no named segment " + std::string(name));
}

std::vector<core::Point2> join(const std::vector<core::Point2>& first,
                               const std::vector<core::Point2>& second)
{
    if (first.empty()) return second;
    if (second.empty()) return first;
    std::vector<core::Point2> result = first;
    result.insert(result.end(), second.begin() + 1, second.end());
    return result;
}

template <typename Section, typename Convert>
std::vector<std::pair<std::string, std::vector<core::Point3>>>
section_curves(const Section& section, Convert convert)
{
    const auto& segments = section_segments(section);
    const auto flank_neg = join(segment_points(segments, "fillet_neg"),
                                segment_points(segments, "flank_neg"));
    const auto flank_pos = join(segment_points(segments, "flank_pos"),
                                segment_points(segments, "fillet_pos"));
    const bool split_cap = std::any_of(
        segments.begin(), segments.end(), [](const auto& item) { return item.name == "cap_neg"; });
    std::vector<std::pair<std::string, std::vector<core::Point2>>> ordered{
        {"flank_neg", flank_neg},
        {"riser_neg", segment_points(segments, "riser_neg")},
    };
    if (split_cap) {
        ordered.emplace_back("cap_neg", segment_points(segments, "cap_neg"));
        ordered.emplace_back("cap_pos", segment_points(segments, "cap_pos"));
    } else {
        ordered.emplace_back("cap", segment_points(segments, "cap"));
    }
    ordered.emplace_back("riser_pos", segment_points(segments, "riser_pos"));
    ordered.emplace_back("flank_pos", flank_pos);
    ordered.emplace_back("root", segment_points(segments, "root"));

    std::vector<std::pair<std::string, std::vector<core::Point3>>> result;
    result.reserve(ordered.size());
    for (const auto& [name, points] : ordered) {
        std::vector<core::Point3> converted;
        converted.reserve(points.size());
        for (const auto& point : points) converted.push_back(convert(point));
        result.emplace_back(name, std::move(converted));
    }
    return result;
}

std::vector<BlankDimension> dimension_spur_blank(
    const Session& session, const com::Dispatch& model,
    const core::spur::SetGeometry& geometry, std::string_view member,
    const BlankSketch& sketch, const std::vector<core::Point2>& outline)
{
    const auto& params = geometry.parameters;
    const auto& gear = geometry.member(std::string(member));
    const auto datum = axis_datum(sketch.axis);
    const bool has_hub = outline.size() > 4;
    const double inner_radius = outline.at(0).x;
    const double outer_radius = outline.at(1).x;
    const double back_z = outline.at(2).y;

    if (sketch.lines.size() < 4) throw SwError("spur blank has too few sketch lines");
    const auto front_face = sketch.lines.front();
    const auto outside = sketch.lines.at(1);
    const auto back_face = sketch.lines.at(2);
    const auto inner = sketch.lines.back();
    select_entities(model, {datum, front_face}, "place spur front face");
    call(model, "SketchAddConstraints", {com::Variant(kRelationCoincident)},
         "place spur front face through datum");
    call(model, "ClearSelection2", {com::Variant(true)}, "clear front-face selection");

    const bool internal = gear.internal;
    const std::string inner_what = internal ? "tip radius" : "bore radius";
    const std::string inner_name = internal ? "TipRadius" : "BoreRadius";
    const std::string outer_what = internal ? "rim radius" : "tip radius";
    const std::string outer_name = internal ? "RimRadius" : "TipRadius";
    struct Plan {
        std::vector<com::Dispatch> entities;
        core::Point3 position;
        double value;
        std::string unit;
        std::string what;
        std::string name;
        std::optional<std::string> definition;
    };
    std::vector<Plan> plan{
        {{sketch.axis, inner}, radial_position(inner_radius, 0.5 * back_z),
         inner_radius, "mm", inner_what, inner_name, std::nullopt},
        {{sketch.axis, outside}, radial_position(outer_radius, 0.5 * back_z),
         outer_radius, "mm", outer_what, outer_name, std::nullopt},
        {{datum, back_face}, axial_position(outer_radius, 0.5 * back_z),
         back_z, "mm", "face width", "FaceWidth", std::nullopt},
    };
    std::vector<BlankVariable> variables;
    if (has_hub) {
        const double hub_radius = outline.at(3).x;
        const double hub_back_z = outline.at(4).y;
        variables.push_back({"HubThickness", equation_number(params.hub_thickness) + "mm",
                             "hub thickness"});
        plan.push_back({{sketch.axis, sketch.lines.at(3)},
                        radial_position(hub_radius, 0.5 * (back_z + hub_back_z)),
                        hub_radius, "mm", "hub radius", "HubRadius", std::nullopt});
        plan.push_back({{datum, sketch.lines.at(4)},
                        axial_position(1.5 * outer_radius, 0.5 * hub_back_z),
                        hub_back_z, "mm", "hub back face", "HubBackFace",
                        std::string("\"FaceWidth\" + \"HubThickness\"")});
    }

    std::vector<BlankDimension> dimensions;
    DimensionFlags flags(session.application());
    for (const auto& item : plan) {
        add_dimension(model, item.entities, item.position,
                      millimeters_to_meters(item.value), item.what, item.name);
        dimensions.push_back({item.name, item.value, item.unit,
                              millimeters_to_meters(item.value), item.what,
                              item.definition});
    }
    return dimensions;
}

template <typename Geometry>
std::vector<BlankDimension> dimension_cone_blank(
    const Session& session, const com::Dispatch& model, const Geometry& geometry,
    std::string_view member, const BlankSketch& sketch,
    const std::vector<core::Point2>& outline)
{
    const auto& params = geometry.parameters;
    const auto& gear = geometry.member(std::string(member));
    const auto apex = axis_datum(sketch.axis);
    const bool has_rim = params.min_root_thickness > 0.0;
    const std::size_t back_index = has_rim ? 4U : 3U;
    const auto& first = outline.at(0);
    const auto& crown = outline.at(2);
    const auto& root = outline.at(3);
    const double back_z = outline.at(back_index).y;
    const auto& front_face = sketch.lines.at(0);
    const auto& face_cone = sketch.lines.at(1);
    const auto& back_cone = sketch.lines.at(2);
    const auto& flat_back = sketch.lines.at(back_index);
    const auto& bore = sketch.lines.back();

    struct Plan {
        std::vector<com::Dispatch> entities;
        core::Point3 position;
        double value;
        std::string unit;
        std::string what;
        std::string name;
        std::optional<std::string> definition;
    };
    std::vector<Plan> plan{
        {{sketch.axis, bore}, radial_position(first.x, 0.5 * (first.y + back_z)),
         first.x, "mm", "bore radius", "BoreRadius", std::nullopt},
        {{apex, front_face}, axial_position(crown.x, 0.5 * first.y),
         first.y, "mm", "front face to apex", "FrontFaceToApex", std::nullopt},
    };
    if (has_rim) {
        plan.push_back({{endpoint(back_cone, false), flat_back},
                        axial_position(1.5 * crown.x, 0.5 * (root.y + back_z)),
                        back_z - root.y, "mm", "minimum root thickness",
                        "MinRootThickness", std::nullopt});
    }
    plan.push_back({{apex, flat_back}, axial_position(crown.x, 0.5 * back_z),
                    back_z, "mm", "back face to apex", "BackFaceToApex",
                    has_rim ? std::optional<std::string>("\"OuterRootToApex\" + \"MinRootThickness\"")
                            : std::nullopt});
    plan.push_back({{sketch.axis, endpoint(back_cone, true)}, radial_position(crown.x, crown.y),
                    crown.x, "mm", "crown radius", "CrownRadius", std::nullopt});
    plan.push_back({{sketch.axis, endpoint(back_cone, false)}, radial_position(root.x, back_z),
                    root.x, "mm", "outer root radius", "OuterRootRadius", std::nullopt});
    if (params.hub_thickness > 0.0) {
        const double hub_radius = outline.at(back_index + 1).x;
        const double hub_back_z = outline.at(back_index + 2).y;
        plan.push_back({{sketch.axis, sketch.lines.at(back_index + 1)},
                        radial_position(hub_radius, 0.5 * (back_z + hub_back_z)),
                        hub_radius, "mm", "hub radius", "HubRadius", std::nullopt});
        plan.push_back({{apex, sketch.lines.at(back_index + 2)},
                        axial_position(crown.x, 0.5 * hub_back_z), hub_back_z,
                        "mm", "hub back to apex", "HubBackToApex", std::nullopt});
    }
    const double face_angle = gear.face_angle_rad;
    const double back_angle = core::kPi / 2.0 - gear.pitch_angle_rad;
    auto angle_position = [](core::Point2 a, core::Point2 b) {
        const double apex_z = a.y - a.x * (b.y - a.y) / (b.x - a.x);
        const double d1 = std::hypot(a.x, a.y - apex_z);
        const double d2 = std::hypot(b.x, b.y - apex_z);
        const auto far_point = d1 >= d2 ? a : b;
        const double d = std::max(d1, d2);
        const double ur = far_point.x / d;
        const double uz = (far_point.y - apex_z) / d;
        const double br = ur;
        const double bz = uz + (uz >= 0.0 ? 1.0 : -1.0);
        const double norm = std::hypot(br, bz);
        const double reach = 0.5 * std::min(d1, d2);
        return core::Point3{millimeters_to_meters(reach * br / norm), 0.0,
                            millimeters_to_meters(apex_z + reach * bz / norm)};
    };
    plan.push_back({{face_cone, sketch.axis}, angle_position(outline.at(1), crown),
                    core::radians_to_degrees(face_angle), "deg", "face cone angle",
                    "FaceConeAngle", std::nullopt});
    plan.push_back({{back_cone, sketch.axis}, angle_position(crown, root),
                    core::radians_to_degrees(back_angle), "deg", "back cone angle",
                    "BackConeAngle", std::nullopt});

    std::vector<BlankDimension> dimensions;
    DimensionFlags flags(session.application());
    for (const auto& item : plan) {
        const double si = item.unit == "mm" ? millimeters_to_meters(item.value)
                                              : core::degrees_to_radians(item.value);
        add_dimension(model, item.entities, item.position, si, item.what, item.name);
        dimensions.push_back({item.name, item.value, item.unit, si, item.what,
                              item.definition});
    }
    return dimensions;
}

} // namespace

PartArtifact build_spur_part(const Session& session,
                             const core::spur::SetGeometry& geometry,
                             std::string_view member,
                             const std::filesystem::path& save_path)
{
    const auto& m = geometry.member(std::string(member));
    const auto model = session.new_part();
    const auto axis = create_axis(model);
    const auto outline = core::spur::blank_outline(geometry, std::string(member));
    const auto sketch = sketch_blank_outline(model, outline);
    constrain_blank(model, sketch, outline);
    const auto dimensions = dimension_spur_blank(session, model, geometry, member,
                                                 sketch, outline);
    std::vector<BlankVariable> variables;
    if (outline.size() > 4)
        variables.push_back({"HubThickness", equation_number(geometry.parameters.hub_thickness) + "mm",
                             "hub thickness"});
    close_and_revolve_blank(model, sketch, dimensions, variables);

    const bool helical = std::abs(m.twist_rad) > kStraightTwistToleranceRad;
    const auto heights = core::spur::section_heights(geometry, std::string(member));
    std::vector<com::Dispatch> sections;
    for (const double z : heights) {
        const auto profile = core::spur::tooth_space_section(
            geometry, std::string(member), z, core::involute::kFlankPoints, helical);
        const auto curves = section_curves(profile, [&](core::Point2 point) {
            return core::spur::to_axial_3d(point.x, point.y, profile.phase_rad, profile.z_mm);
        });
        sections.push_back(draw_curves_3d(model, curves,
                                          "spur section at z=" + std::to_string(z)));
    }
    std::vector<com::Dispatch> guides;
    if (helical) {
        const auto points = core::spur::guide_helix(geometry, std::string(member),
                                                    heights.front(), heights.back());
        guides.push_back(draw_curves_3d(model, {{"guide helix", points}}, "spur loft guide"));
    }
    const auto cut = loft_cut(model, sections, guides, kMarkLoftGuide);
    drop_offcut(model);
    pattern_teeth(model, cut, axis, m.teeth);
    const double expected = m.internal ? core::spur::rim_radius(geometry, std::string(member))
                                       : m.tip_radius_mm;
    return measure_part(session, model, std::string(member), m.teeth, expected, save_path);
}

template <typename Geometry>
PartArtifact build_cone_part(const Session& session, const Geometry& geometry,
                             std::string_view member,
                             const std::filesystem::path& save_path)
{
    const auto& m = geometry.member(std::string(member));
    const auto model = session.new_part();
    const auto axis = create_axis(model);
    std::vector<core::Point2> outline;
    if constexpr (std::is_same_v<Geometry, core::hypoid::SetGeometry>)
        outline = core::hypoid::blank_outline(geometry, std::string(member));
    else
        outline = core::bevel::blank_outline(geometry, std::string(member));
    const auto sketch = sketch_blank_outline(model, outline);
    constrain_blank(model, sketch, outline);
    const auto dimensions = dimension_cone_blank(session, model, geometry, member, sketch, outline);
    std::vector<BlankVariable> variables;
    if (geometry.parameters.min_root_thickness > 0.0)
        variables.push_back({"OuterRootToApex", equation_number(outline.at(3).y) + "mm",
                             "outer root point to apex"});
    close_and_revolve_blank(model, sketch, dimensions, variables);

    std::vector<double> distances;
    bool curved = false;
    if constexpr (std::is_same_v<Geometry, core::hypoid::SetGeometry>) {
        distances = core::hypoid::section_cone_distances(geometry, std::string(member));
        curved = true;
    } else {
        distances = core::bevel::section_cone_distances(geometry, std::string(member));
        curved = geometry.trace.has_value();
    }
    std::vector<com::Dispatch> sections;
    for (const double distance : distances) {
        if constexpr (std::is_same_v<Geometry, core::hypoid::SetGeometry>) {
            const auto section = core::hypoid::tooth_space_section(
                geometry, std::string(member), distance, true);
            const auto curves = section_curves(section, [&](core::Point2 point) {
                return core::bevel::to_cone_3d(point.x, point.y, section.pitch_angle_rad,
                                               section.cone_apex_z_mm, section.phase_rad);
            });
            sections.push_back(draw_curves_3d(model, curves,
                                              "hypoid section at A=" + std::to_string(distance)));
        } else {
            const auto section = core::bevel::tooth_space_section(
                geometry, std::string(member), "outer", core::involute::kFlankPoints,
                0.0, distance, curved);
            const auto curves = section_curves(section, [&](core::Point2 point) {
                return core::bevel::to_cone_3d(point.x, point.y, section.pitch_angle_rad,
                                               section.cone_apex_z_mm, section.phase_rad);
            });
            sections.push_back(draw_curves_3d(model, curves,
                                              "bevel section at A=" + std::to_string(distance)));
        }
    }
    std::vector<com::Dispatch> guides;
    if (curved) {
        if constexpr (std::is_same_v<Geometry, core::hypoid::SetGeometry>) {
            std::vector<core::Point3> points;
            for (std::size_t interval = 0; interval + 1 < distances.size(); ++interval) {
                const double a = distances[interval];
                const double b = distances[interval + 1];
                for (int i = 0; i <= 8; ++i) {
                    if (interval != 0 && i == 0) continue;
                    const double d = a + (b - a) * static_cast<double>(i) / 8.0;
                    const auto section = core::hypoid::tooth_space_section(
                        geometry, std::string(member), d, true);
                    points.push_back(core::bevel::to_cone_3d(
                        section.r_cap_mm, 0.0, section.pitch_angle_rad,
                        section.cone_apex_z_mm, section.phase_rad));
                }
            }
            guides.push_back(draw_curves_3d(model, {{"hypoid cap guide", points}},
                                            "hypoid loft guide"));
        } else {
            guides.push_back(draw_curves_3d(
                model, {{"guide spiral", core::bevel::guide_spiral(
                           geometry, std::string(member), distances.front(), distances.back())}},
                "bevel loft guide"));
        }
    }
    const auto cut = loft_cut(model, sections, guides, kMarkLoftGuide);
    drop_offcut(model);
    pattern_teeth(model, cut, axis, m.z);
    if constexpr (std::is_same_v<Geometry, core::hypoid::SetGeometry>)
        return measure_part(session, model, std::string(member), m.z,
                            m.outer_tip_radius_mm, save_path);
    else
        return measure_part(session, model, std::string(member), m.z,
                            m.outside_dia_mm / 2.0, save_path);
}

PartArtifact build_bevel_part(const Session& session,
                              const core::bevel::SetGeometry& geometry,
                              std::string_view member,
                              const std::filesystem::path& save_path)
{
    return build_cone_part(session, geometry, member, save_path);
}

PartArtifact build_hypoid_part(const Session& session,
                               const core::hypoid::SetGeometry& geometry,
                               std::string_view member,
                               const std::filesystem::path& save_path)
{
    return build_cone_part(session, geometry, member, save_path);
}

} // namespace geargen::solidworks::detail

#endif
