#include "preview/export/export.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace geargen::preview::exporter {

std::vector<std::string> dxf_lines(const Scene2D& scene)
{
    std::vector<std::string> lines{"0", "SECTION", "2", "ENTITIES"};
    for (const auto& polyline : scene.polylines) {
        if (polyline.points.size() < 2) {
            continue;
        }
        lines.insert(lines.end(), {"0", "LWPOLYLINE", "8", polyline.style,
                                   "90", std::to_string(polyline.points.size()),
                                   "70", polyline.closed ? "1" : "0"});
        for (const auto& point : polyline.points) {
            std::ostringstream x;
            std::ostringstream y;
            x << std::setprecision(17) << point.x;
            y << std::setprecision(17) << point.y;
            lines.insert(lines.end(), {"10", x.str(), "20", y.str()});
        }
    }
    lines.insert(lines.end(), {"0", "ENDSEC", "0", "EOF"});
    return lines;
}

void write_dxf(const std::filesystem::path& path, const Scene2D& scene)
{
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("could not open DXF output: " + path.string());
    }
    for (const auto& line : dxf_lines(scene)) {
        output << line << '\n';
    }
}

} // namespace geargen::preview::exporter
