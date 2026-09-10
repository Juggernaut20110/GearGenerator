#include "preview/scene2d/bevel_scene.hpp"

#include "core/common/numerics.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace geargen::preview::bevel {

namespace {
using core::Point2;
using core::bevel::SetGeometry;

Polyline2D line(std::vector<Point2> points, std::string style, bool closed = false)
{
    return {std::move(points), std::move(style), closed};
}

Point2 polar_point(double radius, double angle)
{
    return {radius * std::cos(angle), radius * std::sin(angle)};
}

} // namespace

std::vector<Point2> space_loop(const core::bevel::ToothSpaceSection& section,
                               int tip_points)
{
    auto result = space_boundary(section.profile.segments, tip_points);
    return result.empty() ? section.loop_2d() : result;
}

std::vector<Point2> tooth_loop(const core::bevel::ToothSpaceSection& section,
                               double pitch_step_rad)
{
    auto result = tooth_boundary(section.profile.segments, pitch_step_rad);
    return result.empty() ? section.loop_2d() : result;
}

std::vector<Point2> to_axial(const std::vector<Point2>& points,
                             const core::bevel::ToothSpaceSection& section)
{
    std::vector<Point2> result;
    result.reserve(points.size());
    for (const auto point : points) {
        const auto mapped = core::bevel::to_cone_3d(
            point.x, point.y, section.pitch_angle_rad, section.cone_apex_z_mm,
            section.phase_rad);
        result.push_back({mapped.x, mapped.y});
    }
    return result;
}

Scene2D developed_scene(const SetGeometry& geometry, const std::string& name,
                        int neighbours)
{
    const auto& member = geometry.member(name);
    const auto outer = core::bevel::tooth_space_section(geometry, name, "outer");
    const auto inner = core::bevel::tooth_space_section(geometry, name, "inner");
    const double step = core::kTau / member.virtual_teeth;
    const double half = step / 2.0;
    const double span = (static_cast<double>(neighbours) + 0.75) * step;
    const double scale = geometry.section_scale;
    std::vector<Polyline2D> lines{
        line(arc(member.virtual_pitch_r_mm, half - span, half + span), "pitch"),
    };
    for (const double radius : {member.virtual_base_r_mm, member.virtual_root_r_mm,
                                member.virtual_tip_r_mm,
                                scale * member.virtual_root_r_mm,
                                member.virtual_tip_r_inner_mm}) {
        lines.push_back(line(arc(radius, half - span, half + span), "reference"));
    }
    const auto outer_loop = tooth_loop(outer, step);
    const auto inner_loop = tooth_loop(inner, step);
    for (int i = 1; i <= neighbours; ++i) {
        for (const int sign : {-1, 1})
            lines.push_back(line(rotate(outer_loop, sign * i * step), "neighbour", true));
    }
    for (const auto* section : {&outer, &inner}) {
        lines.push_back(line(section->loop_2d(), "cut", true));
        lines.push_back(line(rotate(section->loop_2d(), step), "cut", true));
    }
    lines.push_back(line(outer_loop, "outer", true));
    lines.push_back(line(inner_loop, "inner", true));
    std::ostringstream title;
    title << name << " - developed tooth, z_v = " << format_number(member.virtual_teeth, 2)
          << " teeth at back cone radius " << format_number(member.virtual_pitch_r_mm, 3)
          << " mm";
    return {"developed", title.str(), std::move(lines),
            {{"outer", "outer end tooth"}, {"inner", "inner end tooth"},
             {"neighbour", "adjacent teeth"}, {"cut", "loft cut boundary"},
             {"pitch", "pitch circle"}, {"reference", "base / root / tip"}}};
}

Scene2D axial_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto& member = geometry.member(name);
    const auto outer = core::bevel::tooth_space_section(geometry, name, "outer");
    const auto inner = core::bevel::tooth_space_section(geometry, name, "inner");
    const double step = core::kTau / member.virtual_teeth;
    const auto outer_loop = to_axial(tooth_loop(outer, step), outer);
    const auto inner_loop = to_axial(tooth_loop(inner, step), inner);
    std::vector<Polyline2D> lines{
        line(circle(member.pitch_dia_mm / 2.0), "pitch"),
        line(circle(member.outside_dia_mm / 2.0), "reference"),
        line(circle(geometry.inner_cone_dist_mm * std::sin(member.pitch_angle_rad)), "reference"),
        line(circle(member.virtual_tip_r_inner_mm * std::cos(member.pitch_angle_rad)), "reference"),
    };
    for (int i = 1; i < member.z; ++i) {
        const double angle = static_cast<double>(i) * member.angular_pitch_rad();
        lines.push_back(line(rotate(outer_loop, angle), "neighbour", true));
        lines.push_back(line(rotate(inner_loop, angle), "neighbour", true));
    }
    lines.push_back(line(outer_loop, "outer", true));
    lines.push_back(line(inner_loop, "inner", true));
    return {"axial", name + " - " + std::to_string(member.z) +
                         " teeth about the axis, outside dia " +
                         format_number(member.outside_dia_mm, 3) + " mm",
            std::move(lines), {{"outer", "outer end tooth"},
                               {"inner", "inner end tooth"},
                               {"neighbour", "the other teeth"},
                               {"pitch", "pitch circle"},
                               {"reference", "tip and inner circles"}}};
}

Scene2D trace_scene(const SetGeometry& geometry, const std::string&, int neighbours)
{
    const auto& trace = geometry.trace;
    const double inner = geometry.inner_cone_dist_mm;
    const double mean = geometry.mean_cone_dist_mm;
    const double outer = geometry.outer_cone_dist_mm;
    const auto trace_points = [&trace, inner, outer](double offset) {
        std::vector<Point2> result;
        for (int i = 0; i < 81; ++i) {
            const double t = static_cast<double>(i) / 80.0;
            const double distance = inner + t * (outer - inner);
            const double angle = offset + (trace ? trace->sign * trace->theta_at(distance) : 0.0);
            result.push_back(polar_point(distance, angle));
        }
        return result;
    };
    const double crown_pitch = geometry.circular_pitch_mm * (mean / outer) / mean;
    const double span = (static_cast<double>(neighbours) + 0.75) * crown_pitch;
    std::vector<Polyline2D> lines{
        line(arc(mean, -span, span), "pitch"),
        line(arc(inner, -span, span), "reference"),
        line(arc(outer, -span, span), "reference"),
        line({polar_point(inner, 0.0), polar_point(outer, 0.0)}, "cut"),
    };
    for (int i = 1; i <= neighbours; ++i) {
        for (const int sign : {-1, 1})
            lines.push_back(line(trace_points(sign * i * crown_pitch), "neighbour"));
    }
    lines.push_back(line(trace_points(0.0), "outer"));
    const std::string title = trace
        ? "both members - tooth trace in the crown plane, " +
          format_number(trace->cutter_radius_mm, 3) + " mm cutter"
        : "both members - straight teeth - the trace is the cone generator itself";
    return {"trace", title, std::move(lines),
            {{"outer", "tooth trace"}, {"neighbour", "adjacent traces"},
             {"cut", "a straight tooth, for comparison"},
             {"pitch", "mean cone distance Am"},
             {"reference", "toe Ai and heel Ao"}}};
}

Scene2D blank_scene(const SetGeometry& geometry, const std::string& name)
{
    const auto& member = geometry.member(name);
    const auto raw = core::bevel::blank_outline(geometry, name);
    std::vector<Point2> outline;
    outline.reserve(raw.size());
    double z_max = 0.0;
    double r_max = 0.0;
    for (const auto point : raw) {
        outline.push_back({point.y, point.x});
        z_max = std::max(z_max, point.x);
        r_max = std::max(r_max, point.y);
    }
    const double delta = member.pitch_angle_rad;
    const Point2 pitch_end{geometry.outer_cone_dist_mm * std::cos(delta),
                           geometry.outer_cone_dist_mm * std::sin(delta)};
    const Point2 root_end{geometry.outer_cone_dist_mm * std::cos(member.root_angle_rad),
                          geometry.outer_cone_dist_mm * std::sin(member.root_angle_rad)};
    std::vector<Polyline2D> lines{
        line({{0.0, 0.0}, {1.05 * z_max, 0.0}}, "axis"),
        line({{0.0, 0.0}, pitch_end}, "pitch"),
        line({{0.0, 0.0}, {pitch_end.x, -pitch_end.y}}, "pitch"),
        line({{0.0, 0.0}, root_end}, "cone"),
        line({{0.0, 0.0}, {root_end.x, -root_end.y}}, "cone"),
        line(outline, "blank", true),
    };
    std::vector<Point2> mirrored;
    for (const auto point : outline) mirrored.push_back({point.x, -point.y});
    lines.push_back(line(mirrored, "blank", true));
    return {"blank", name + " - blank section, crown at z = " +
                         format_number(member.crown_to_apex_mm, 3) +
                         " mm, max radius " + format_number(r_max, 3) + " mm",
            std::move(lines), {{"blank", "revolved outline"},
                               {"pitch", "pitch cone"}, {"cone", "root cone"},
                               {"axis", "gear axis"}}};
}

Scene2D build_scene(const SetGeometry& geometry, const std::string& member,
                    const std::string& key)
{
    if (key == "developed") return developed_scene(geometry, member);
    if (key == "axial") return axial_scene(geometry, member);
    if (key == "trace") return trace_scene(geometry, member);
    if (key == "blank") return blank_scene(geometry, member);
    throw std::invalid_argument("unknown scene '" + key + "'");
}

std::vector<std::pair<std::string, std::string>> scene_labels()
{
    return {{"developed", "Developed section"}, {"axial", "Down the axis"},
            {"trace", "Tooth trace"}, {"blank", "Blank section"}};
}

std::vector<Row> derived_rows(const SetGeometry& geometry)
{
    const auto& a = geometry.pinion;
    const auto& b = geometry.gear;
    return {{"SET", {}, {}, {}, true},
            {"ratio", format_number(geometry.parameters.ratio())},
            {"outer cone distance Ao", format_number(geometry.outer_cone_dist_mm), {}, "mm"},
            {"mean cone distance Am", format_number(geometry.mean_cone_dist_mm), {}, "mm"},
            {"inner cone distance Ai", format_number(geometry.inner_cone_dist_mm), {}, "mm"},
            {"teeth", std::to_string(a.z), std::to_string(b.z)},
            {"pitch cone angle", format_number(a.pitch_angle_deg(), 4),
             format_number(b.pitch_angle_deg(), 4), "deg"},
            {"pitch diameter", format_number(a.pitch_dia_mm), format_number(b.pitch_dia_mm), "mm"},
            {"virtual teeth z_v", format_number(a.virtual_teeth), format_number(b.virtual_teeth)}};
}

std::vector<std::string> csv_lines(const SetGeometry& geometry,
                                   const std::string& name)
{
    std::vector<std::string> lines{"member,end,loop,index,x_dev,y_dev,x,y,z"};
    for (const std::string& end : {"outer", "inner"}) {
        const auto section = core::bevel::tooth_space_section(geometry, name, end);
        const auto space = space_loop(section);
        for (const auto& [kind, loop] : std::vector<std::pair<std::string, std::vector<Point2>>>{
                 {"space", space}, {"cut", section.loop_2d()}}) {
            for (std::size_t i = 0; i < loop.size(); ++i) {
                const auto point = core::bevel::to_cone_3d(
                    loop[i].x, loop[i].y, section.pitch_angle_rad,
                    section.cone_apex_z_mm);
                std::ostringstream row;
                row << name << ',' << end << ',' << kind << ',' << i << ','
                    << format_number(loop[i].x, 6) << ',' << format_number(loop[i].y, 6)
                    << ',' << format_number(point.x, 6) << ',' << format_number(point.y, 6)
                    << ',' << format_number(point.z, 6);
                lines.push_back(row.str());
            }
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

} // namespace geargen::preview::bevel
