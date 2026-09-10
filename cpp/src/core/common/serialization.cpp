#include "core/common/serialization.hpp"

#include <iomanip>
#include <cctype>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
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

using JsonFields = std::map<std::string, std::string>;

JsonFields parse_flat_object(const std::string& text)
{
    std::size_t cursor = 0;
    const auto skip = [&]() {
        while (cursor < text.size() &&
               std::isspace(static_cast<unsigned char>(text[cursor]))) ++cursor;
    };
    const auto string_value = [&]() {
        if (cursor >= text.size() || text[cursor] != '"')
            throw std::runtime_error("preset JSON expected a string");
        ++cursor;
        std::string result;
        while (cursor < text.size()) {
            const char c = text[cursor++];
            if (c == '"') return result;
            if (c != '\\') { result += c; continue; }
            if (cursor >= text.size()) throw std::runtime_error("invalid JSON escape");
            const char escaped = text[cursor++];
            switch (escaped) {
            case '"': result += '"'; break;
            case '\\': result += '\\'; break;
            case '/': result += '/'; break;
            case 'b': result += '\b'; break;
            case 'f': result += '\f'; break;
            case 'n': result += '\n'; break;
            case 'r': result += '\r'; break;
            case 't': result += '\t'; break;
            default: throw std::runtime_error("unsupported JSON escape in preset");
            }
        }
        throw std::runtime_error("unterminated JSON string in preset");
    };
    skip();
    if (cursor >= text.size() || text[cursor++] != '{')
        throw std::runtime_error("preset JSON must contain an object");
    JsonFields fields;
    skip();
    if (cursor < text.size() && text[cursor] == '}') return fields;
    while (cursor < text.size()) {
        skip();
        const std::string key = string_value();
        skip();
        if (cursor >= text.size() || text[cursor++] != ':')
            throw std::runtime_error("preset JSON expected ':'");
        skip();
        std::string value;
        if (cursor < text.size() && text[cursor] == '"') {
            value = string_value();
        } else {
            const std::size_t start = cursor;
            while (cursor < text.size() && text[cursor] != ',' && text[cursor] != '}') ++cursor;
            value = text.substr(start, cursor - start);
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) value.pop_back();
        }
        fields[key] = value;
        skip();
        if (cursor >= text.size()) break;
        if (text[cursor] == '}') { ++cursor; return fields; }
        if (text[cursor++] != ',') throw std::runtime_error("preset JSON expected ','");
    }
    throw std::runtime_error("unterminated preset JSON object");
}

JsonFields read_fields(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("could not open preset: " + path.string());
    return parse_flat_object({std::istreambuf_iterator<char>(input), {}});
}

const std::string& required(const JsonFields& fields, const char* key)
{
    const auto found = fields.find(key);
    if (found == fields.end() || found->second.empty() || found->second == "null")
        throw std::runtime_error(std::string("preset is missing required field '") + key + "'");
    return found->second;
}

std::optional<double> optional_number(const JsonFields& fields, const char* key)
{
    const auto found = fields.find(key);
    if (found == fields.end() || found->second == "null" || found->second.empty()) return std::nullopt;
    return std::stod(found->second);
}

int integer_or(const JsonFields& fields, const char* key, int fallback)
{
    const auto found = fields.find(key);
    return found == fields.end() || found->second == "null" ? fallback : std::stoi(found->second);
}

bool boolean_or(const JsonFields& fields, const char* key, bool fallback)
{
    const auto found = fields.find(key);
    if (found == fields.end() || found->second == "null") return fallback;
    if (found->second == "true") return true;
    if (found->second == "false") return false;
    throw std::runtime_error(std::string("preset field '") + key + "' is not boolean");
}

std::string string_or(const JsonFields& fields, const char* key, std::string fallback)
{
    const auto found = fields.find(key);
    return found == fields.end() || found->second == "null" ? std::move(fallback) : found->second;
}

Hand hand_or(const JsonFields& fields, const char* key, Hand fallback)
{
    const auto value = string_or(fields, key, hand_name(fallback));
    if (value == "right") return Hand::Right;
    if (value == "left") return Hand::Left;
    throw std::runtime_error(std::string("preset field '") + key + "' has invalid hand");
}

void write_preset(const std::filesystem::path& path, const std::string& json)
{
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) throw std::runtime_error("could not write preset: " + path.string());
    output << json << '\n';
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

SpurSetParams load_spur_preset(const std::filesystem::path& path)
{
    const auto fields = read_fields(path);
    SpurDefaultOverrides o;
    o.face_width = optional_number(fields, "face_width");
    o.bore = optional_number(fields, "bore");
    o.hub_thickness = optional_number(fields, "hub_thickness");
    o.pressure_angle = optional_number(fields, "pressure_angle");
    o.helix_angle = optional_number(fields, "helix_angle");
    o.hand = hand_or(fields, "hand", Hand::Right);
    o.internal = boolean_or(fields, "internal", false);
    o.fillet_factor = optional_number(fields, "fillet_factor");
    o.backlash = optional_number(fields, "backlash");
    o.rim_thickness = optional_number(fields, "rim_thickness");
    o.profile_shift_1 = optional_number(fields, "profile_shift_1");
    o.profile_shift_2 = optional_number(fields, "profile_shift_2");
    o.basic_rack_addendum_factor = optional_number(fields, "basic_rack_addendum_factor");
    o.basic_rack_clearance_factor = optional_number(fields, "basic_rack_clearance_factor");
    o.basic_rack_root_radius_factor = optional_number(fields, "basic_rack_root_radius_factor");
    o.working_centre_distance = optional_number(fields, "working_centre_distance");
    o.tip_alteration_coefficient = optional_number(fields, "tip_alteration_coefficient");
    o.tip_alteration_mode = string_or(fields, "tip_alteration_mode", "iso_clearance");
    o.root_geometry = string_or(fields, "root_geometry", "legacy");
    o.backlash_mode = string_or(fields, "backlash_mode", "legacy_reference");
    o.backlash_allocation = optional_number(fields, "backlash_allocation");
    return SpurSetParams::with_defaults(std::stod(required(fields, "module")),
                                        std::stoi(required(fields, "z1")),
                                        std::stoi(required(fields, "z2")), o);
}

BevelSetParams load_bevel_preset(const std::filesystem::path& path)
{
    const auto fields = read_fields(path);
    BevelDefaultOverrides o;
    o.face_width = optional_number(fields, "face_width");
    o.bore = optional_number(fields, "bore");
    o.hub_thickness = optional_number(fields, "hub_thickness");
    o.min_root_thickness = optional_number(fields, "min_root_thickness");
    o.pressure_angle = optional_number(fields, "pressure_angle");
    o.shaft_angle = optional_number(fields, "shaft_angle");
    o.spiral_angle = optional_number(fields, "spiral_angle");
    o.hand = hand_or(fields, "hand", Hand::Right);
    o.cutter_radius = optional_number(fields, "cutter_radius");
    o.fillet_factor = optional_number(fields, "fillet_factor");
    o.backlash = optional_number(fields, "backlash");
    return BevelSetParams::with_defaults(std::stod(required(fields, "module")),
                                         std::stoi(required(fields, "z1")),
                                         std::stoi(required(fields, "z2")), false, o);
}

HypoidSetParams load_hypoid_preset(const std::filesystem::path& path)
{
    const auto fields = read_fields(path);
    HypoidDefaultOverrides o;
    o.face_width = optional_number(fields, "face_width");
    o.bore = optional_number(fields, "bore");
    o.hub_thickness = optional_number(fields, "hub_thickness");
    o.pressure_angle = optional_number(fields, "pressure_angle");
    o.shaft_angle = optional_number(fields, "shaft_angle");
    o.offset = optional_number(fields, "offset");
    o.spiral_angle = optional_number(fields, "spiral_angle");
    o.hand = hand_or(fields, "hand", Hand::Right);
    o.cutter_radius = optional_number(fields, "cutter_radius");
    o.backlash = optional_number(fields, "backlash");
    o.min_root_thickness = optional_number(fields, "min_root_thickness");
    o.gear_mean_addendum_factor = optional_number(fields, "gear_mean_addendum_factor");
    o.depth_factor = optional_number(fields, "depth_factor");
    o.clearance_factor = optional_number(fields, "clearance_factor");
    o.thickness_factor = optional_number(fields, "thickness_factor");
    o.gear_addendum_angle = optional_number(fields, "gear_addendum_angle");
    o.gear_dedendum_angle = optional_number(fields, "gear_dedendum_angle");
    o.root_fillet_radius = optional_number(fields, "root_fillet_radius");
    return HypoidSetParams::with_defaults(std::stod(required(fields, "module")),
                                          std::stoi(required(fields, "z1")),
                                          std::stoi(required(fields, "z2")), o);
}

PlanetarySetParams load_planetary_preset(const std::filesystem::path& path)
{
    const auto fields = read_fields(path);
    PlanetaryDefaultOverrides o;
    o.n_planets = integer_or(fields, "n_planets", 3);
    o.face_width = optional_number(fields, "face_width");
    o.bore = optional_number(fields, "bore");
    o.hub_thickness = optional_number(fields, "hub_thickness");
    o.rim_thickness = optional_number(fields, "rim_thickness");
    o.pressure_angle = optional_number(fields, "pressure_angle");
    o.helix_angle = optional_number(fields, "helix_angle");
    o.hand = hand_or(fields, "hand", Hand::Right);
    o.fillet_factor = optional_number(fields, "fillet_factor");
    o.backlash = optional_number(fields, "backlash");
    return PlanetarySetParams::with_defaults(std::stod(required(fields, "module")),
                                             std::stoi(required(fields, "z_sun")),
                                             std::stoi(required(fields, "z_planet")), o);
}

void save_preset(const std::filesystem::path& path, const SpurSetParams& p)
{ write_preset(path, parameters_json(p)); }
void save_preset(const std::filesystem::path& path, const BevelSetParams& p)
{ write_preset(path, parameters_json(p)); }
void save_preset(const std::filesystem::path& path, const HypoidSetParams& p)
{ write_preset(path, parameters_json(p)); }
void save_preset(const std::filesystem::path& path, const PlanetarySetParams& p)
{ write_preset(path, parameters_json(p)); }

} // namespace geargen::core
