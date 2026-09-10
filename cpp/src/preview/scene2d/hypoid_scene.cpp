#include "preview/scene2d/hypoid_scene.hpp"

#include "core/bevel/bevel.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace geargen::preview::hypoid {

namespace {
using core::Point2;
using core::hypoid::SetGeometry;

Polyline2D line(std::vector<Point2> points, std::string style, bool closed = false)
{
    return {std::move(points), std::move(style), closed};
}

std::vector<Point2> tooth(const core::hypoid::Section& section, double step)
{
    auto result = tooth_boundary(section.segments, step);
    return result.empty() ? section.loop : result;
}

} // namespace

Scene2D contact_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto& member = geometry.member(name);
    const auto section = core::hypoid::tooth_space_section(geometry, name);
    const double step = core::kTau / static_cast<double>(member.z);
    const auto loop = tooth(section, step);
    std::vector<Polyline2D> lines{line(circle(member.pitch_radius_mm), "pitch")};
    for (int i = -1; i <= 1; ++i)
        lines.push_back(line(rotate(loop, i * step), "outer", true));
    return {"contact", name + " - approximate Tredgold hypoid contact, " +
                         std::to_string(member.z) + " teeth, offset " +
                         format_number(geometry.parameters.offset, 3) + " mm",
            std::move(lines), {{"outer", "tooth"}, {"pitch", "mean pitch circle"}}};
}

Scene2D section_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto& member = geometry.member(name);
    const auto section = core::hypoid::tooth_space_section(geometry, name);
    const double step = core::kTau / static_cast<double>(member.z);
    return {"section", name + " - Tredgold section, " + std::to_string(member.z) + " teeth",
            {line(circle(member.pitch_radius_mm), "pitch"),
             line(circle(member.virtual_base_r_mm), "reference"),
             line(circle(member.mean_root_radius_mm), "reference"),
             line(circle(member.tredgold_tip_radius_mm), "reference"),
             line(tooth(section, step), "outer", true),
             line(section.loop, "cut", true)},
            {{"outer", "tooth"}, {"cut", "tooth-space cut"},
             {"pitch", "mean pitch"}}};
}

Scene2D blank_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto outline = core::hypoid::blank_outline(geometry, name);
    const auto& member = geometry.member(name);
    double z_min = 0.0;
    double z_max = 0.0;
    for (const auto point : outline) {
        z_min = std::min(z_min, point.y);
        z_max = std::max(z_max, point.y);
    }
    return {"blank", name + " - hypoid blank meridian",
            {line({{0.0, z_min - 2.0}, {0.0, z_max + 2.0}}, "axis"),
             line(outline, "blank", true),
             line({{member.pitch_radius_mm, z_min}, {member.pitch_radius_mm, z_max}}, "pitch")},
            {{"blank", "revolved outline"}, {"pitch", "mean pitch radius"},
             {"axis", "axis"}}};
}

Scene2D build_scene(const SetGeometry& geometry, const std::string& member,
                    const std::string& key)
{
    if (key == "contact") return contact_scene(geometry, member);
    if (key == "section") return section_scene(geometry, member);
    if (key == "blank") return blank_scene(geometry, member);
    throw std::invalid_argument("unknown scene '" + key + "'");
}

std::vector<std::pair<std::string, std::string>> scene_labels()
{
    return {{"contact", "Contact geometry"}, {"section", "Tooth section"},
            {"blank", "Blank section"}};
}

std::vector<Row> derived_rows(const SetGeometry& geometry)
{
    const auto& a = geometry.pinion;
    const auto& b = geometry.gear;
    return {{"INPUT", {}, {}, {}, true},
            {"shaft angle", format_number(core::radians_to_degrees(geometry.parameters.sigma()), 4), {}, "deg"},
            {"hypoid offset", format_number(geometry.parameters.offset, 4), {}, "mm"},
            {"ratio", format_number(geometry.parameters.ratio(), 4)},
            {"pitch-plane offset", format_number(geometry.pitch_plane_offset_mm, 4), {}, "mm"},
            {"mean normal module", format_number(geometry.mean_normal_module_mm, 4), {}, "mm"},
            {"MEMBERS", "PINION", "GEAR", {}, true},
            {"teeth", std::to_string(a.z), std::to_string(b.z)},
            {"pitch cone angle", format_number(core::radians_to_degrees(a.pitch_angle_rad), 4),
             format_number(core::radians_to_degrees(b.pitch_angle_rad), 4), "deg"},
            {"mean pitch radius", format_number(a.pitch_radius_mm, 4),
             format_number(b.pitch_radius_mm, 4), "mm"},
            {"TREDGOLD APPROXIMATION", {}, {}, {}, true},
            {"developed tip radius", format_number(a.tredgold_tip_radius_mm, 4),
             format_number(b.tredgold_tip_radius_mm, 4), "mm"}};
}

std::vector<std::string> csv_lines(const SetGeometry& geometry,
                                   const std::string& name)
{
    std::vector<std::string> lines{"member,section_kind,index,x_dev,y_dev,x,y,z"};
    const auto bounds = core::hypoid::section_cone_bounds(geometry, name);
    for (const double cone : core::hypoid::section_cone_distances(geometry, name, 2)) {
        std::string label;
        if (std::abs(cone - bounds.tooth_face_inner_mm) < 1e-9)
            label = "physical-tooth-face-inner";
        else if (std::abs(cone - bounds.calculation_point_mm) < 1e-9)
            label = "method1-calculation-point";
        else if (std::abs(cone - bounds.tooth_face_outer_mm) < 1e-9)
            label = "physical-tooth-face-outer";
        else if (cone < bounds.tooth_face_inner_mm)
            label = "loft-only-inner-extension";
        else if (cone > bounds.tooth_face_outer_mm)
            label = "loft-only-outer-extension";
        else
            label = "physical-face-intermediate";
        const auto section = core::hypoid::tooth_space_section(geometry, name, cone, true);
        for (std::size_t i = 0; i < section.loop.size(); ++i) {
            const auto point = core::bevel::to_cone_3d(
                section.loop[i].x, section.loop[i].y, section.pitch_angle_rad,
                section.cone_apex_z_mm, section.phase_rad);
            std::ostringstream row;
            row << name << ',' << label << ',' << i << ','
                << format_number(section.loop[i].x, 6) << ','
                << format_number(section.loop[i].y, 6) << ','
                << format_number(point.x, 6) << ',' << format_number(point.y, 6)
                << ',' << format_number(point.z, 6);
            lines.push_back(row.str());
        }
    }
    return lines;
}

void write_csv(const std::filesystem::path& path, const SetGeometry& geometry,
               const std::string& member)
{
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) throw std::runtime_error("could not open CSV output: " + path.string());
    for (const auto& row : csv_lines(geometry, member)) output << row << '\n';
}

} // namespace geargen::preview::hypoid
