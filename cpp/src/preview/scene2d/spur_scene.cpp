#include "preview/scene2d/spur_scene.hpp"

#include "core/common/numerics.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace geargen::preview::spur {

namespace {
using core::Point2;
using core::spur::SetGeometry;

Polyline2D line(std::vector<Point2> points, std::string style,
                bool closed = false)
{
    return {std::move(points), std::move(style), closed};
}

std::string member_kind(const core::spur::MemberGeometry& member,
                        const std::string& name)
{
    return member.internal ? "internal ring" : name;
}

} // namespace

std::vector<core::Point2> space_loop(
    const core::spur::ToothSpaceSection& section, int tip_points)
{
    auto result = space_boundary(section.segments, tip_points);
    return result.empty() ? section.loop_2d : result;
}

std::vector<core::Point2> tooth_loop(
    const core::spur::ToothSpaceSection& section, double pitch_step_rad)
{
    auto result = tooth_boundary(section.segments, pitch_step_rad);
    return result.empty() ? section.loop_2d : result;
}

Scene2D transverse_scene(const SetGeometry& geometry, const std::string& name,
                         int neighbours)
{
    const auto& member = geometry.member(name);
    const auto section = core::spur::tooth_space_section(geometry, name);
    const double step = core::kTau / static_cast<double>(member.teeth);
    const double half = step / 2.0;
    const double span = (static_cast<double>(neighbours) + 0.75) * step;
    std::vector<Polyline2D> lines{
        line(arc(member.reference_radius_mm, half - span, half + span), "pitch"),
        line(arc(member.working_radius_mm, half - span, half + span), "working"),
    };
    for (const double radius : {member.base_radius_mm, member.root_radius_mm,
                                member.tip_radius_mm}) {
        lines.push_back(line(arc(radius, half - span, half + span), "reference"));
    }
    if (member.internal) {
        lines.push_back(line(arc(core::spur::rim_radius(geometry, name),
                                 half - span, half + span), "reference"));
    }

    const auto tooth = tooth_loop(section, step);
    for (int i = 1; i <= neighbours; ++i) {
        for (const int sign : {-1, 1}) {
            lines.push_back(line(rotate(tooth, sign * i * step), "neighbour", true));
        }
    }
    lines.push_back(line(section.loop_2d, "cut", true));
    lines.push_back(line(rotate(section.loop_2d, step), "cut", true));
    lines.push_back(line(tooth, "outer", true));

    std::ostringstream title;
    title << name << " - transverse tooth, " << member_kind(member, name)
          << ", " << member.teeth << " teeth at m_t = "
          << format_number(geometry.transverse_module_mm)
          << " mm, alpha_t = "
          << format_number(core::radians_to_degrees(geometry.reference_pressure_angle_rad), 3)
          << " deg";
    return {"transverse", title.str(), std::move(lines),
            {{"outer", "tooth"}, {"neighbour", "adjacent teeth"},
             {"cut", "loft cut boundary"}, {"pitch", "reference pitch circle"},
             {"working", "working pitch circle"},
             {"reference", "base / root / tip"}}};
}

Scene2D twist_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto& member = geometry.member(name);
    const double face_width = geometry.parameters.face_width;
    const auto front = core::spur::tooth_space_section(geometry, name, 0.0);
    const auto back = core::spur::tooth_space_section(geometry, name, face_width);
    std::vector<Polyline2D> lines{
        line(circle(member.reference_radius_mm), "pitch"),
        line(circle(member.working_radius_mm), "working"),
        line(circle(member.base_radius_mm), "reference"),
        line(circle(member.root_radius_mm), "reference"),
        line(circle(member.tip_radius_mm), "reference"),
    };
    if (member.internal)
        lines.push_back(line(circle(core::spur::rim_radius(geometry, name)), "blank"));
    const double step = member.angular_pitch_rad();
    const auto front_loop = tooth_loop(front, step);
    const auto back_loop = tooth_loop(back, step);
    for (int i = 0; i < member.teeth; ++i) {
        const double angle = static_cast<double>(i) * step;
        lines.push_back(line(rotate(front_loop, angle), "outer", true));
        if (std::abs(member.twist_rad) > 1e-12)
            lines.push_back(line(rotate(back_loop, angle + member.twist_rad), "inner", true));
    }
    std::string note = std::abs(member.twist_rad) > 1e-12
        ? "twist " + format_number(core::radians_to_degrees(member.twist_rad), 3) +
          " deg, " + core::hand_name(geometry.member(name).beta_rad < 0.0 ? core::Hand::Left : core::Hand::Right) + " hand"
        : "straight teeth";
    std::vector<LegendEntry> legend{{"outer", "front face, z = 0"}};
    if (std::abs(member.twist_rad) > 1e-12)
        legend.emplace_back("inner", "back face, z = " + format_number(face_width, 3) + " mm");
    legend.insert(legend.end(), {{"pitch", "reference pitch circle"},
                                  {"working", "working pitch circle"},
                                  {"reference", "base / root / tip"}});
    return {"twist", name + " - " + std::to_string(member.teeth) +
                     " teeth down the axis, " + note, std::move(lines), legend};
}

Scene2D blank_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto& member = geometry.member(name);
    const auto outline = core::spur::blank_outline(geometry, name);
    double z_lo = 0.0;
    double z_hi = 0.0;
    for (const auto point : outline) {
        z_lo = std::min(z_lo, point.y);
        z_hi = std::max(z_hi, point.y);
    }
    std::vector<Polyline2D> lines{
        line({{0.0, z_lo - 2.0}, {0.0, z_hi + 2.0}}, "axis"),
        line({{member.root_radius_mm, 0.0},
              {member.root_radius_mm, geometry.parameters.face_width}}, "reference"),
        line({{member.reference_radius_mm, 0.0},
              {member.reference_radius_mm, geometry.parameters.face_width}}, "pitch"),
        line({{member.working_radius_mm, 0.0},
              {member.working_radius_mm, geometry.parameters.face_width}}, "working"),
        line(outline, "blank", true),
    };
    const double overshoot = core::spur::end_overshoot(geometry);
    lines.push_back(line({{member.tip_radius_mm + 1.0, -overshoot},
                          {member.tip_radius_mm + 1.0,
                           geometry.parameters.face_width + overshoot}}, "cut"));
    return {"blank", name + " - blank meridian, R x z, front face at z = 0",
            std::move(lines), {{"blank", "revolved outline"},
                               {"pitch", "reference pitch radius"},
                               {"working", "working pitch radius"},
                               {"reference", "root radius"},
                               {"cut", "cut reach along z"}, {"axis", "gear axis"}}};
}

Scene2D build_scene(const SetGeometry& geometry, const std::string& member,
                    const std::string& key)
{
    if (key == "transverse") return transverse_scene(geometry, member);
    if (key == "twist") return twist_scene(geometry, member);
    if (key == "blank") return blank_scene(geometry, member);
    throw std::invalid_argument("unknown scene '" + key + "'");
}

std::vector<std::pair<std::string, std::string>> scene_labels()
{
    return {{"transverse", "Transverse section"}, {"twist", "Down the axis"},
            {"blank", "Blank section"}};
}

std::vector<Row> derived_rows(const SetGeometry& geometry)
{
    const auto& p = geometry.parameters;
    const auto& a = geometry.pinion;
    const auto& b = geometry.gear;
    return {{"SET", {}, {}, {}, true},
            {"arrangement", p.internal ? "internal" : "external"},
            {"ratio", format_number(p.ratio())},
            {"reference centre distance a", format_number(geometry.reference_centre_distance_mm), {}, "mm"},
            {"transverse module m_t", format_number(geometry.transverse_module_mm), {}, "mm"},
            {"teeth", std::to_string(a.teeth), std::to_string(b.teeth)},
            {"reference diameter d", format_number(2.0 * a.reference_radius_mm),
             format_number(2.0 * b.reference_radius_mm), "mm"},
            {"tip radius", format_number(a.tip_radius_mm), format_number(b.tip_radius_mm), "mm"},
            {"root radius", format_number(a.root_radius_mm), format_number(b.root_radius_mm), "mm"},
            {"twist over the face", format_number(core::radians_to_degrees(a.twist_rad)),
             format_number(core::radians_to_degrees(b.twist_rad)), "deg"}};
}

std::vector<std::string> csv_lines(const SetGeometry& geometry,
                                   const std::string& name)
{
    const double overshoot = core::spur::end_overshoot(geometry);
    const double width = geometry.parameters.face_width;
    std::vector<std::string> lines{"member,section,index,x,y,z"};
    for (const auto& [label, z] : std::vector<std::pair<std::string, double>>{
             {"front", -overshoot}, {"back", width + overshoot}}) {
        const auto section = core::spur::tooth_space_section(geometry, name, z);
        for (std::size_t i = 0; i < section.loop_3d().size(); ++i) {
            const auto point = section.loop_3d()[i];
            std::ostringstream row;
            row << name << ',' << label << ',' << i << ',' << format_number(point.x, 6)
                << ',' << format_number(point.y, 6) << ',' << format_number(point.z, 6);
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

} // namespace geargen::preview::spur
