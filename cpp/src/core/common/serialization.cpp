#include "core/common/serialization.hpp"

#include <iomanip>
#include <limits>
#include <sstream>

namespace geargen::core {

namespace {

std::string object(std::initializer_list<std::pair<std::string, std::string>> fields)
{
    std::ostringstream stream;
    stream << '{';
    bool first = true;
    for (const auto& [key, value] : fields) {
        if (!first) {
            stream << ',';
        }
        first = false;
        stream << json_string(key) << ':' << value;
    }
    stream << '}';
    return stream.str();
}

std::string issues(const std::vector<Issue>& values)
{
    std::ostringstream stream;
    stream << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            stream << ',';
        }
        stream << object({{"field", json_string(values[i].field)},
                          {"message", json_string(values[i].message)}});
    }
    stream << ']';
    return stream.str();
}

} // namespace

std::string json_number(double value)
{
    if (!std::isfinite(value)) {
        return "null";
    }
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10)
           << value;
    return stream.str();
}

std::string json_integer(int value)
{
    return std::to_string(value);
}

std::string json_bool(bool value)
{
    return value ? "true" : "false";
}

std::string json_string(const std::string& value)
{
    std::ostringstream stream;
    stream << '"';
    for (const unsigned char character : value) {
        switch (character) {
        case '\\': stream << "\\\\"; break;
        case '"': stream << "\\\""; break;
        case '\n': stream << "\\n"; break;
        case '\r': stream << "\\r"; break;
        case '\t': stream << "\\t"; break;
        default:
            if (character < 0x20U) {
                stream << "\\u00" << std::hex << std::setw(2)
                       << std::setfill('0') << static_cast<int>(character)
                       << std::dec << std::setfill(' ');
            } else {
                stream << static_cast<char>(character);
            }
        }
    }
    stream << '"';
    return stream.str();
}

std::string json_optional_number(const std::optional<double>& value)
{
    return value.has_value() ? json_number(*value) : "null";
}

std::string parameters_json(const SpurSetParams& p)
{
    return object({
        {"type", json_string("spur")},
        {"module", json_number(p.module)},
        {"z1", json_integer(p.z1)},
        {"z2", json_integer(p.z2)},
        {"face_width", json_number(p.face_width)},
        {"bore", json_number(p.bore)},
        {"hub_thickness", json_number(p.hub_thickness)},
        {"pressure_angle", json_number(p.pressure_angle)},
        {"helix_angle", json_number(p.helix_angle)},
        {"hand", json_string(hand_name(p.hand))},
        {"internal", json_bool(p.internal)},
        {"fillet_factor", json_number(p.fillet_factor)},
        {"backlash", json_number(p.backlash)},
        {"rim_thickness", json_number(p.rim_thickness)},
        {"profile_shift_1", json_number(p.profile_shift_1)},
        {"profile_shift_2", json_number(p.profile_shift_2)},
        {"basic_rack_addendum_factor", json_number(p.basic_rack_addendum_factor)},
        {"basic_rack_clearance_factor", json_number(p.basic_rack_clearance_factor)},
        {"basic_rack_root_radius_factor", json_number(p.basic_rack_root_radius_factor)},
        {"working_centre_distance", json_optional_number(p.working_centre_distance)},
        {"tip_alteration_mode", json_string(p.tip_alteration_mode)},
        {"tip_alteration_coefficient", json_optional_number(p.tip_alteration_coefficient)},
        {"root_geometry", json_string(p.root_geometry)},
        {"backlash_mode", json_string(p.backlash_mode)},
        {"backlash_allocation", json_number(p.backlash_allocation)},
    });
}

std::string parameters_json(const BevelSetParams& p)
{
    return object({
        {"type", json_string("bevel")},
        {"module", json_number(p.module)},
        {"z1", json_integer(p.z1)},
        {"z2", json_integer(p.z2)},
        {"face_width", json_number(p.face_width)},
        {"bore", json_number(p.bore)},
        {"hub_thickness", json_number(p.hub_thickness)},
        {"min_root_thickness", json_number(p.min_root_thickness)},
        {"pressure_angle", json_number(p.pressure_angle)},
        {"shaft_angle", json_number(p.shaft_angle)},
        {"spiral_angle", json_number(p.spiral_angle)},
        {"hand", json_string(hand_name(p.hand))},
        {"cutter_radius", json_optional_number(p.cutter_radius)},
        {"fillet_factor", json_number(p.fillet_factor)},
        {"backlash", json_number(p.backlash)},
    });
}

std::string parameters_json(const HypoidSetParams& p)
{
    return object({
        {"type", json_string("hypoid")},
        {"module", json_number(p.module)},
        {"z1", json_integer(p.z1)},
        {"z2", json_integer(p.z2)},
        {"face_width", json_number(p.face_width)},
        {"bore", json_number(p.bore)},
        {"hub_thickness", json_number(p.hub_thickness)},
        {"pressure_angle", json_number(p.pressure_angle)},
        {"shaft_angle", json_number(p.shaft_angle)},
        {"offset", json_number(p.offset)},
        {"spiral_angle", json_number(p.spiral_angle)},
        {"hand", json_string(hand_name(p.hand))},
        {"cutter_radius", json_optional_number(p.cutter_radius)},
        {"backlash", json_number(p.backlash)},
        {"min_root_thickness", json_number(p.min_root_thickness)},
        {"gear_mean_addendum_factor", json_number(p.gear_mean_addendum_factor)},
        {"depth_factor", json_number(p.depth_factor)},
        {"clearance_factor", json_number(p.clearance_factor)},
        {"thickness_factor", json_number(p.thickness_factor)},
        {"gear_addendum_angle", json_number(p.gear_addendum_angle)},
        {"gear_dedendum_angle", json_number(p.gear_dedendum_angle)},
        {"root_fillet_radius", json_optional_number(p.root_fillet_radius)},
    });
}

std::string parameters_json(const PlanetarySetParams& p)
{
    return object({
        {"type", json_string("planetary")},
        {"module", json_number(p.module)},
        {"z_sun", json_integer(p.z_sun)},
        {"z_planet", json_integer(p.z_planet)},
        {"n_planets", json_integer(p.n_planets)},
        {"face_width", json_number(p.face_width)},
        {"bore", json_number(p.bore)},
        {"hub_thickness", json_number(p.hub_thickness)},
        {"rim_thickness", json_number(p.rim_thickness)},
        {"pressure_angle", json_number(p.pressure_angle)},
        {"helix_angle", json_number(p.helix_angle)},
        {"hand", json_string(hand_name(p.hand))},
        {"fillet_factor", json_number(p.fillet_factor)},
        {"backlash", json_number(p.backlash)},
    });
}

std::string validation_json(const ValidationResult& result)
{
    return object({
        {"ok", json_bool(result.ok())},
        {"errors", issues(result.errors)},
        {"warnings", issues(result.warnings)},
        {"advisories", issues(result.advisories)},
    });
}

} // namespace geargen::core
