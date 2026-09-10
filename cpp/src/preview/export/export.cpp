#include "preview/export/export.hpp"

#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace geargen::preview::exporter {

std::vector<std::string> dxf_lines(const Scene2D& scene)
{
    // Match gears.preview.dxf_lines: a minimal AutoCAD R12 file. R12's
    // POLYLINE/VERTEX form is intentionally used for broad CAD compatibility.
    std::vector<std::string> lines{
        "0", "SECTION", "2", "HEADER", "9", "$ACADVER", "1", "AC1009",
        "9", "$INSUNITS", "70", "4", "0", "ENDSEC", "0", "SECTION",
        "2", "ENTITIES"};
    const auto number = [](double value) {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::fixed << std::setprecision(6) << value;
        return stream.str();
    };
    for (const auto& polyline : scene.polylines) {
        if (polyline.points.size() < 2) {
            continue;
        }
        lines.insert(lines.end(), {"0", "POLYLINE", "8", polyline.style,
                                   "66", "1", "70", polyline.closed ? "1" : "0",
                                   "10", "0.000000", "20", "0.000000",
                                   "30", "0.000000"});
        for (const auto& point : polyline.points) {
            lines.insert(lines.end(), {"0", "VERTEX", "8", polyline.style,
                                       "10", number(point.x), "20", number(point.y),
                                       "30", "0.000000"});
        }
        lines.insert(lines.end(), {"0", "SEQEND", "8", polyline.style});
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
