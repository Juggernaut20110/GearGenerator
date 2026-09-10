#include "preview/scene3d/assembly.hpp"

#include "core/bevel/mesh.hpp"
#include "core/hypoid/mesh.hpp"
#include "core/placement/placement.hpp"
#include "core/planetary/mesh.hpp"
#include "core/spur/mesh.hpp"
#include "preview/scene2d/bevel_scene.hpp"
#include "preview/scene2d/hypoid_scene.hpp"
#include "preview/scene2d/spur_scene.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace geargen::preview::scene3d {

namespace {
using core::Matrix3;
using core::Point2;
using core::Point3;
using core::placement::apply;
using core::placement::matmul;
using core::placement::rot_z;

Point3 transformed(Point3 point, const Matrix3& rotation, Point3 translation)
{
    return apply(rotation, point) + translation;
}

std::vector<double> lerp(double start, double end, int count)
{
    count = std::max(2, count);
    std::vector<double> result;
    result.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
        result.push_back(start + (end - start) * static_cast<double>(i) /
                                   static_cast<double>(count - 1));
    return result;
}

std::vector<Point3> decimate(const std::vector<Point3>& points, int maximum = 56)
{
    if (static_cast<int>(points.size()) <= maximum) return points;
    std::vector<Point3> result;
    result.reserve(static_cast<std::size_t>(maximum));
    for (int i = 0; i < maximum; ++i) {
        const auto index = static_cast<std::size_t>(std::llround(
            static_cast<double>(i) * static_cast<double>(points.size() - 1) /
            static_cast<double>(maximum - 1)));
        result.push_back(points[index]);
    }
    return result;
}

void add_member_wireframe(std::vector<Polyline3D>& lines,
                          const std::vector<std::vector<Point3>>& sections,
                          int teeth, double angular_pitch, const Matrix3& rotation,
                          Point3 translation, const std::string& style,
                          const std::string& member, int connector_stride = 20)
{
    for (int tooth = 0; tooth < teeth; ++tooth) {
        const double tooth_angle = static_cast<double>(tooth) * angular_pitch;
        std::vector<std::vector<Point3>> turned;
        turned.reserve(sections.size());
        for (const auto& section : sections) {
            std::vector<Point3> points;
            points.reserve(section.size());
            for (const auto point : section)
                points.push_back(transformed(point, rot_z(tooth_angle), {}));
            turned.push_back(std::move(points));
        }
        for (const auto& section : turned) {
            std::vector<Point3> world;
            world.reserve(section.size());
            for (const auto point : section) world.push_back(transformed(point, rotation, translation));
            lines.push_back({std::move(world), style, true, member});
        }
        for (std::size_t section = 1; section < turned.size(); ++section) {
            const auto count = std::min(turned[section - 1].size(), turned[section].size());
            for (std::size_t index = 0; index < count; index += static_cast<std::size_t>(std::max(1, connector_stride))) {
                lines.push_back({
                    {transformed(turned[section - 1][index], rotation, translation),
                     transformed(turned[section][index], rotation, translation)},
                    style, false, member});
            }
        }
    }
}

void add_axis(std::vector<Polyline3D>& lines, const Matrix3& rotation,
              Point3 translation, double length, const std::string& member)
{
    lines.push_back({{transformed({}, rotation, translation),
                      transformed({0.0, 0.0, length}, rotation, translation)},
                     "axis", false, member});
}

std::vector<std::vector<Point3>> spur_sections(const core::spur::SetGeometry& geometry,
                                                const std::string& role)
{
    const auto& member = geometry.member(role);
    std::vector<std::vector<Point3>> result;
    for (const double z : lerp(0.0, geometry.parameters.face_width, 5)) {
        const auto section = core::spur::tooth_space_section(geometry, role, z);
        const auto loop = ::geargen::preview::spur::tooth_loop(section, member.angular_pitch_rad());
        std::vector<Point3> points;
        for (const auto point : loop)
            points.push_back(core::spur::to_axial_3d(point.x, point.y,
                                                     section.phase_rad, section.z_mm));
        result.push_back(decimate(points));
    }
    return result;
}

std::vector<std::vector<Point3>> bevel_sections(const core::bevel::SetGeometry& geometry,
                                                 const std::string& role)
{
    const auto& member = geometry.member(role);
    std::vector<std::vector<Point3>> result;
    for (const double distance : lerp(geometry.inner_cone_dist_mm,
                                      geometry.outer_cone_dist_mm, 5)) {
        const auto section = core::bevel::tooth_space_section(
            geometry, role, "outer", core::involute::kFlankPoints, 0.0,
            distance);
        auto loop = ::geargen::preview::bevel::tooth_loop(section, core::kTau / member.virtual_teeth);
        std::vector<Point3> points;
        for (const auto point : loop)
            points.push_back(core::bevel::to_cone_3d(
                point.x, point.y, section.pitch_angle_rad,
                section.cone_apex_z_mm, section.phase_rad));
        result.push_back(decimate(points));
    }
    return result;
}

std::vector<std::vector<Point3>> hypoid_sections(const core::hypoid::SetGeometry& geometry,
                                                 const std::string& role)
{
    const auto& member = geometry.member(role);
    const auto bounds = core::hypoid::section_cone_bounds(geometry, role);
    std::vector<std::vector<Point3>> result;
    for (const double distance : lerp(bounds.tooth_face_inner_mm,
                                      bounds.tooth_face_outer_mm, 5)) {
        const auto section = core::hypoid::tooth_space_section(geometry, role, distance);
        auto loop = ::geargen::preview::tooth_boundary(section.segments,
                                             core::kTau / member.virtual_teeth);
        if (loop.empty()) loop = section.loop;
        std::vector<Point3> points;
        for (const auto point : loop)
            points.push_back(core::bevel::to_cone_3d(
                point.x, point.y, section.pitch_angle_rad,
                section.cone_apex_z_mm, section.phase_rad));
        result.push_back(decimate(points));
    }
    return result;
}

double pair_phase(int z1, int z2, bool internal, double mesh_position)
{
    return mesh_position / core::placement::angular_velocity_ratio(z1, z2, internal);
}

void finite_or_throw(const Scene3D& scene)
{
    for (const auto& line : scene.polylines)
        for (const auto point : line.points)
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw std::domain_error("3D preview contains a non-finite point");
}

} // namespace

Scene3D build_scene(const core::spur::SetGeometry& geometry, double mesh_position)
{
    const auto& p = geometry.parameters;
    std::vector<Polyline3D> lines;
    const Matrix3 pinion_rotation = matmul(core::placement::pinion_placement(), rot_z(mesh_position));
    const Matrix3 gear_rotation = matmul(
        core::spur::mesh::gear_placement(core::spur::mesh::clocking_for(geometry)),
        rot_z(pair_phase(geometry.pinion.teeth, geometry.gear.teeth,
                         p.internal, mesh_position)));
    for (const auto& [role, rotation, translation, style] : {
             std::tuple<std::string, Matrix3, Point3, std::string>{"pinion", pinion_rotation, {}, "pinion"},
             {"gear", gear_rotation, core::spur::mesh::gear_translation(geometry), "gear"}}) {
        const auto& member = geometry.member(role);
        add_member_wireframe(lines, spur_sections(geometry, role), member.teeth,
                             member.angular_pitch_rad(), rotation, translation,
                             style, role);
        for (const double z : {0.0, p.face_width}) {
            std::vector<Point3> circle_points;
            for (const auto point : ::geargen::preview::circle(member.working_radius_mm, 49))
                circle_points.push_back(transformed({point.x, point.y, z}, rotation, translation));
            lines.push_back({std::move(circle_points), "pitch", true, role});
        }
        add_axis(lines, rotation, translation, p.face_width, role);
    }
    lines.push_back({{{0.0, 0.0, 0.0}, core::spur::mesh::gear_translation(geometry)},
                     "reference", false, ""});
    Scene3D scene{std::move(lines), "3D assembly - " +
                  std::string(p.internal ? "internal ring" : "external pair"),
                  "assembly", {{"pinion", "pinion"}, {"gear", "gear/ring"},
                               {"pitch", "working pitch"}, {"axis", "axes"}}};
    finite_or_throw(scene);
    return scene;
}

Scene3D build_scene(const core::bevel::SetGeometry& geometry, double mesh_position)
{
    const auto& p = geometry.parameters;
    const double clocking = core::bevel::mesh::gear_clocking(geometry.gear.z);
    const Matrix3 pinion_rotation = matmul(core::bevel::mesh::pinion_placement(), rot_z(mesh_position));
    const Matrix3 gear_rotation = matmul(core::bevel::mesh::gear_placement(p.sigma(), clocking),
                                         rot_z(pair_phase(p.z1, p.z2, false, mesh_position)));
    std::vector<Polyline3D> lines;
    for (const auto& [role, rotation, style] : {
             std::tuple<std::string, Matrix3, std::string>{"pinion", pinion_rotation, "pinion"},
             {"gear", gear_rotation, "gear"}}) {
        const auto& member = geometry.member(role);
        add_member_wireframe(lines, bevel_sections(geometry, role), member.z,
                             member.angular_pitch_rad(), rotation, {}, style, role);
        add_axis(lines, rotation, {}, member.crown_to_apex_mm, role);
    }
    const std::string title = "3D assembly - bevel, " + std::to_string(p.z1) + ":" +
                              std::to_string(p.z2);
    Scene3D scene{std::move(lines), title, "assembly",
                  {{"pinion", "pinion"}, {"gear", "gear"}, {"axis", "shaft axes"}}};
    finite_or_throw(scene);
    return scene;
}

Scene3D build_scene(const core::hypoid::SetGeometry& geometry, double mesh_position)
{
    const auto& p = geometry.parameters;
    const double clocking = core::hypoid::mesh::gear_clocking(geometry);
    const Matrix3 pinion_rotation = matmul(core::hypoid::mesh::pinion_placement(geometry), rot_z(mesh_position));
    const Matrix3 gear_rotation = matmul(core::hypoid::mesh::gear_placement(p.sigma(), clocking),
                                         rot_z(pair_phase(p.z1, p.z2, false, mesh_position)));
    std::vector<Polyline3D> lines;
    for (const auto& [role, rotation, translation, style] : {
             std::tuple<std::string, Matrix3, Point3, std::string>{"pinion", pinion_rotation, {}, "pinion"},
             {"gear", gear_rotation, core::hypoid::mesh::gear_translation(geometry), "gear"}}) {
        const auto& member = geometry.member(role);
        add_member_wireframe(lines, hypoid_sections(geometry, role), member.z,
                             member.angular_pitch_rad(), rotation, translation, style, role);
        add_axis(lines, rotation, translation, member.outer_tip_z_mm, role);
    }
    Scene3D scene{std::move(lines), "3D assembly - hypoid " + std::to_string(p.z1) + ":" +
                  std::to_string(p.z2), "assembly",
                  {{"pinion", "pinion"}, {"gear", "gear"}, {"axis", "skew axes"}}};
    finite_or_throw(scene);
    return scene;
}

Scene3D build_scene(const core::planetary::SetGeometry& geometry, double)
{
    const auto& p = geometry.parameters;
    std::vector<Polyline3D> lines;
    const auto planet_sections = spur_sections(geometry.sun_planet, "gear");
    const auto add_member = [&lines](const core::spur::SetGeometry& pair,
                                     const std::string& role, double clocking,
                                     Point3 translation, const std::string& style,
                                     const std::string& label, double axis_length) {
        const auto& member = pair.member(role);
        add_member_wireframe(lines, spur_sections(pair, role), member.teeth,
                             member.angular_pitch_rad(),
                             core::planetary::mesh::member_placement(clocking),
                             translation, style, label);
        add_axis(lines, core::planetary::mesh::member_placement(clocking),
                 translation, axis_length, label);
    };
    add_member(geometry.sun_planet, "pinion", core::planetary::mesh::sun_clocking(),
               {}, "sun", "sun", p.face_width);
    add_member(geometry.planet_ring, "gear", core::planetary::mesh::ring_clocking(geometry),
               {}, "ring", "ring", p.face_width);
    for (const double z : {0.0, p.face_width}) {
        std::vector<Point3> ring;
        for (const auto point : ::geargen::preview::circle(geometry.ring_rim_radius_mm(), 73))
            ring.push_back({point.x, point.y, z});
        lines.push_back({std::move(ring), "ring", true, "ring"});
    }
    for (int i = 0; i < p.n_planets; ++i) {
        const auto translation = core::planetary::mesh::planet_translation(geometry, i);
        add_member(geometry.sun_planet, "gear",
                   core::planetary::mesh::planet_clocking(geometry, i),
                   translation, "planet", "planet " + std::to_string(i), p.face_width);
    }
    lines.push_back({{{0.0, 0.0, 0.0}, {geometry.centre_distance_mm, 0.0, 0.0}},
                     "reference", false, ""});
    Scene3D scene{std::move(lines), "3D assembly - planetary train", "assembly",
                  {{"sun", "sun"}, {"planet", "planets"}, {"ring", "ring"},
                   {"axis", "axes"}}};
    finite_or_throw(scene);
    return scene;
}

} // namespace geargen::preview::scene3d
