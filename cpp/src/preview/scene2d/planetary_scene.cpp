#include "preview/scene2d/planetary_scene.hpp"

#include "core/planetary/mesh.hpp"
#include "preview/scene2d/spur_scene.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace geargen::preview::planetary {

namespace {
using core::Point2;
using core::planetary::SetGeometry;

Polyline2D line(std::vector<Point2> points, std::string style, bool closed = false)
{
    return {std::move(points), std::move(style), closed};
}

std::vector<Polyline2D> placed_teeth(const core::spur::SetGeometry& pair,
                                     const std::string& role, double clocking,
                                     Point2 centre, const std::string& style)
{
    const auto& member = pair.member(role);
    const auto section = core::spur::tooth_space_section(pair, role);
    const auto loop = preview::spur::tooth_loop(section, member.angular_pitch_rad());
    std::vector<Polyline2D> result;
    for (int i = 0; i < member.teeth; ++i) {
        const auto turned = rotate(loop, static_cast<double>(i) * member.angular_pitch_rad() + clocking);
        std::vector<Point2> translated;
        translated.reserve(turned.size());
        for (const auto point : turned) translated.push_back({point.x + centre.x, point.y + centre.y});
        result.push_back(line(std::move(translated), style, true));
    }
    return result;
}

} // namespace

Scene2D train_scene(const SetGeometry& geometry, const std::string&)
{
    const auto& p = geometry.parameters;
    std::vector<Polyline2D> lines{
        line(circle(geometry.ring_rim_radius_mm()), "blank"),
        line(circle(geometry.sun.reference_radius_mm), "pitch"),
        line(circle(geometry.ring.reference_radius_mm), "pitch"),
        line(circle(geometry.centre_distance_mm), "reference"),
    };
    auto add = [&lines](std::vector<Polyline2D> values) {
        lines.insert(lines.end(), std::make_move_iterator(values.begin()),
                     std::make_move_iterator(values.end()));
    };
    add(placed_teeth(geometry.sun_planet, "pinion",
                     core::planetary::mesh::sun_clocking(), {}, "outer"));
    add(placed_teeth(geometry.planet_ring, "gear",
                     core::planetary::mesh::ring_clocking(geometry), {}, "inner"));
    for (int i = 0; i < p.n_planets; ++i) {
        const auto translation = core::planetary::mesh::planet_translation(geometry, i);
        const Point2 centre{translation.x, translation.y};
        add(placed_teeth(geometry.sun_planet, "gear",
                         core::planetary::mesh::planet_clocking(geometry, i),
                         centre, "neighbour"));
        std::vector<Point2> orbit;
        for (const auto point : circle(geometry.planet.reference_radius_mm))
            orbit.push_back({point.x + centre.x, point.y + centre.y});
        lines.push_back(line(std::move(orbit), "pitch"));
    }
    std::ostringstream title;
    title << "the whole train - sun " << p.z_sun << ", " << p.n_planets
          << " planets of " << p.z_planet << ", ring " << p.z_ring()
          << ", at m_n = " << p.module << " mm";
    return {"train", title.str(), std::move(lines),
            {{"outer", "sun"}, {"neighbour", "planets"}, {"inner", "ring"},
             {"pitch", "pitch circles"}, {"reference", "planet orbit"},
             {"blank", "ring rim"}}};
}

Scene2D transverse_scene(const SetGeometry& geometry, const std::string& member)
{
    const auto& pair = geometry.mesh_for(member);
    const auto role = std::string(geometry.role_of(member));
    auto scene = preview::spur::transverse_scene(pair, role);
    const auto separator = scene.title.find(" - ");
    scene.title = member + (separator == std::string::npos ? " - " : scene.title.substr(separator)) ;
    scene.key = "transverse";
    return scene;
}

Scene2D blank_scene(const SetGeometry& geometry, const std::string& member)
{
    const auto& pair = geometry.mesh_for(member);
    auto scene = preview::spur::blank_scene(pair, geometry.role_of(member));
    const auto separator = scene.title.find(" - ");
    scene.title = member + (separator == std::string::npos ? " - " : scene.title.substr(separator));
    scene.key = "blank";
    return scene;
}

Scene2D build_scene(const SetGeometry& geometry, const std::string& member,
                    const std::string& key)
{
    if (key == "train") return train_scene(geometry, member);
    if (key == "transverse") return transverse_scene(geometry, member);
    if (key == "blank") return blank_scene(geometry, member);
    throw std::invalid_argument("unknown scene '" + key + "'");
}

std::vector<std::pair<std::string, std::string>> scene_labels()
{
    return {{"train", "The whole train"}, {"transverse", "Transverse section"},
            {"blank", "Blank section"}};
}

std::vector<Row> derived_rows(const SetGeometry& geometry)
{
    const auto& p = geometry.parameters;
    return {{"TRAIN", {}, {}, {}, true},
            {"ring teeth (derived)", std::to_string(p.z_ring())},
            {"planets", std::to_string(p.n_planets)},
            {"centre distance", format_number(geometry.centre_distance_mm), {}, "mm"},
            {"assembly condition", p.assembly_remainder() == 0 ? "ok" : "fails"},
            {"neighbour spacing", format_number(geometry.neighbour_spacing_mm()), {}, "mm"},
            {"MEMBERS", "SUN", "PLANET", {}, true, "RING"},
            {"teeth", std::to_string(geometry.sun.teeth), std::to_string(geometry.planet.teeth), {}, false,
             std::to_string(geometry.ring.teeth)},
            {"pitch diameter", format_number(2.0 * geometry.sun.reference_radius_mm),
             format_number(2.0 * geometry.planet.reference_radius_mm), "mm",
             false, format_number(2.0 * geometry.ring.reference_radius_mm)}};
}

std::vector<std::string> csv_lines(const SetGeometry& geometry,
                                   const std::string& member)
{
    const auto& pair = geometry.mesh_for(member);
    const auto section = core::spur::tooth_space_section(pair, geometry.role_of(member));
    std::vector<std::string> lines{"member,index,x,y,z"};
    const auto points = section.loop_3d();
    for (std::size_t i = 0; i < points.size(); ++i) {
        std::ostringstream row;
        row << member << ',' << i << ',' << format_number(points[i].x, 6) << ','
            << format_number(points[i].y, 6) << ',' << format_number(points[i].z, 6);
        lines.push_back(row.str());
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

} // namespace geargen::preview::planetary
