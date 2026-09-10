#include "core/bevel/bevel.hpp"
#include "core/common/numerics.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/planetary/planetary.hpp"
#include "core/planetary/mesh.hpp"
#include "core/spur/spur.hpp"
#include "core/spur/mesh.hpp"
#include "core/involute/involute.hpp"
#include "core/validation/validation.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace {

struct Json {
    using Object = std::map<std::string, Json>;
    using Array = std::vector<Json>;
    std::variant<std::nullptr_t, bool, double, std::string, Object, Array> value;

    const Json& at(const std::string& key) const
    {
        return std::get<Object>(value).at(key);
    }
};

class JsonParser {
public:
    explicit JsonParser(std::string text) : text_(std::move(text)) {}

    Json parse()
    {
        Json result = value();
        whitespace();
        if (position_ != text_.size()) {
            throw std::runtime_error("trailing JSON data");
        }
        return result;
    }

private:
    std::string text_;
    std::size_t position_{0};

    void whitespace()
    {
        while (position_ < text_.size() &&
               (text_[position_] == ' ' || text_[position_] == '\n' ||
                text_[position_] == '\r' || text_[position_] == '\t')) {
            ++position_;
        }
    }

    char consume(char expected)
    {
        whitespace();
        if (position_ >= text_.size() || text_[position_] != expected) {
            throw std::runtime_error("unexpected JSON token");
        }
        return text_[position_++];
    }

    std::string string_value()
    {
        consume('"');
        std::string result;
        while (position_ < text_.size()) {
            const char character = text_[position_++];
            if (character == '"') {
                return result;
            }
            if (character != '\\') {
                result += character;
                continue;
            }
            if (position_ >= text_.size()) {
                throw std::runtime_error("unterminated JSON escape");
            }
            const char escaped = text_[position_++];
            switch (escaped) {
            case '"': result += '"'; break;
            case '\\': result += '\\'; break;
            case '/': result += '/'; break;
            case 'b': result += '\b'; break;
            case 'f': result += '\f'; break;
            case 'n': result += '\n'; break;
            case 'r': result += '\r'; break;
            case 't': result += '\t'; break;
            default: throw std::runtime_error("unsupported JSON escape");
            }
        }
        throw std::runtime_error("unterminated JSON string");
    }

    Json value()
    {
        whitespace();
        if (position_ >= text_.size()) {
            throw std::runtime_error("unexpected end of JSON");
        }
        if (text_[position_] == '{') {
            return object();
        }
        if (text_[position_] == '[') {
            return array();
        }
        if (text_[position_] == '"') {
            return {string_value()};
        }
        if (text_.compare(position_, 4, "true") == 0) {
            position_ += 4;
            return {true};
        }
        if (text_.compare(position_, 5, "false") == 0) {
            position_ += 5;
            return {false};
        }
        if (text_.compare(position_, 4, "null") == 0) {
            position_ += 4;
            return {nullptr};
        }
        const std::size_t start = position_;
        while (position_ < text_.size() &&
               std::string("-+0123456789.eE").find(text_[position_]) !=
                   std::string::npos) {
            ++position_;
        }
        if (start == position_) {
            throw std::runtime_error("invalid JSON number");
        }
        return {std::stod(text_.substr(start, position_ - start))};
    }

    Json object()
    {
        consume('{');
        Json::Object result;
        whitespace();
        if (position_ < text_.size() && text_[position_] == '}') {
            ++position_;
            return {std::move(result)};
        }
        while (true) {
            const std::string key = string_value();
            consume(':');
            result.emplace(key, value());
            whitespace();
            if (position_ < text_.size() && text_[position_] == '}') {
                ++position_;
                return {std::move(result)};
            }
            consume(',');
        }
    }

    Json array()
    {
        consume('[');
        Json::Array result;
        whitespace();
        if (position_ < text_.size() && text_[position_] == ']') {
            ++position_;
            return {std::move(result)};
        }
        while (true) {
            result.push_back(value());
            whitespace();
            if (position_ < text_.size() && text_[position_] == ']') {
                ++position_;
                return {std::move(result)};
            }
            consume(',');
        }
    }
};

double number(const Json& value)
{
    return std::get<double>(value.value);
}

bool boolean(const Json& value)
{
    return std::get<bool>(value.value);
}

std::string string(const Json& value)
{
    return std::get<std::string>(value.value);
}

bool check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool compare(double actual, const Json& expected, const std::string& label)
{
    const double reference = number(expected);
    if (!geargen::core::approximately_equal(actual, reference,
                                             geargen::core::kReferenceTolerance)) {
        std::cerr << "FAIL: " << label << " native=" << actual
                  << " python=" << reference << '\n';
        return false;
    }
    return true;
}

void compare_validation(const geargen::core::ValidationResult& actual,
                        const Json& expected, bool& ok,
                        const std::string& name)
{
    ok &= check(actual.ok() == boolean(expected.at("ok")),
                name + " validation ok flag");
    const auto& errors = std::get<Json::Array>(expected.at("errors").value);
    const auto& warnings = std::get<Json::Array>(expected.at("warnings").value);
    const auto& advisories =
        std::get<Json::Array>(expected.at("advisories").value);
    ok &= check(actual.errors.size() == errors.size(),
                name + " validation error count");
    ok &= check(actual.warnings.size() == warnings.size(),
                name + " validation warning count");
    ok &= check(actual.advisories.size() == advisories.size(),
                name + " validation advisory count");
    for (std::size_t i = 0; i < actual.errors.size() && i < errors.size(); ++i) {
        ok &= check(actual.errors[i].field == string(errors[i].at("field")),
                    name + " validation error field");
        ok &= check(actual.errors[i].message == string(errors[i].at("message")),
                    name + " validation error message");
    }
    for (std::size_t i = 0; i < actual.warnings.size() && i < warnings.size(); ++i) {
        ok &= check(actual.warnings[i].field == string(warnings[i].at("field")),
                    name + " validation warning field");
        if (actual.warnings[i].message != string(warnings[i].at("message"))) {
            std::cerr << "FAIL: " << name
                      << " warning native=" << actual.warnings[i].message
                      << " python=" << string(warnings[i].at("message")) << '\n';
            ok = false;
        }
    }
}

void compare_points(const std::vector<geargen::core::Point2>& actual,
                    const Json& expected, bool& ok,
                    const std::string& label)
{
    const auto& points = std::get<Json::Array>(expected.value);
    ok &= check(actual.size() == points.size(), label + " point count");
    for (std::size_t i = 0; i < actual.size() && i < points.size(); ++i) {
        const auto& point = std::get<Json::Array>(points[i].value);
        ok &= compare(actual[i].x, point.at(0), label + "[x]");
        ok &= compare(actual[i].y, point.at(1), label + "[y]");
    }
}

void compare_numbers(const std::vector<double>& actual, const Json& expected,
                     bool& ok, const std::string& label)
{
    const auto& values = std::get<Json::Array>(expected.value);
    ok &= check(actual.size() == values.size(), label + " count");
    for (std::size_t i = 0; i < actual.size() && i < values.size(); ++i) {
        ok &= compare(actual[i], values[i], label + "[" + std::to_string(i) + "]");
    }
}

void compare_points3(const std::vector<geargen::core::Point3>& actual,
                     const Json& expected, bool& ok,
                     const std::string& label)
{
    const auto& points = std::get<Json::Array>(expected.value);
    ok &= check(actual.size() == points.size(), label + " count");
    for (std::size_t i = 0; i < actual.size() && i < points.size(); ++i) {
        const auto& point = std::get<Json::Array>(points[i].value);
        ok &= compare(actual[i].x, point.at(0), label + "[x]");
        ok &= compare(actual[i].y, point.at(1), label + "[y]");
        ok &= compare(actual[i].z, point.at(2), label + "[z]");
    }
}

void compare_optional_number(const std::optional<double>& actual,
                             const Json& expected, bool& ok,
                             const std::string& label)
{
    if (std::holds_alternative<std::nullptr_t>(expected.value)) {
        ok &= check(!actual.has_value(), label + " is absent");
    } else {
        ok &= check(actual.has_value(), label + " is present");
        if (actual.has_value()) {
            ok &= compare(*actual, expected, label);
        }
    }
}

void compare_optional_bool(const std::optional<bool>& actual,
                           const Json& expected, bool& ok,
                           const std::string& label)
{
    if (std::holds_alternative<std::nullptr_t>(expected.value)) {
        ok &= check(!actual.has_value(), label + " is absent");
    } else {
        ok &= check(actual.has_value(), label + " is present");
        if (actual.has_value()) {
            ok &= check(*actual == boolean(expected), label + " value");
        }
    }
}

void compare_spur_geometry(const geargen::core::spur::SetGeometry& geometry,
                           const Json& snapshot, bool& ok,
                           const std::string& name)
{
    const std::string member_name = string(snapshot.at("member"));
    const auto& member = geometry.member(member_name);
    const int flank_count = static_cast<int>(number(snapshot.at("flank_count")));
    std::vector<geargen::core::Point2> flank;
    if (member.internal) {
        flank = geargen::core::involute::internal_flank_points(
            member.base_radius_mm, member.root_radius_mm,
            member.tip_radius_mm, member.psi0_rad, flank_count);
    } else {
        flank = geargen::core::involute::flank_points(
            member.base_radius_mm, member.root_radius_mm,
            member.tip_radius_mm, member.psi0_rad, member.half_pitch_rad,
            flank_count);
    }
    compare_points(flank, snapshot.at("involute_flank"), ok,
                   name + " involute flank");

    const int section_flank_count = static_cast<int>(
        number(snapshot.at("section_flank_count")));
    const auto section = geargen::core::spur::tooth_space_section(
        geometry, member_name, 0.0, section_flank_count, true);
    ok &= check(section.filleted == boolean(snapshot.at("section_filleted")),
                name + " section fillet flag");
    ok &= check(section.rack_generated() ==
                    boolean(snapshot.at("section_rack_generated")),
                name + " section generated-root flag");
    compare_optional_number(section.generated_root_radius_mm,
                            snapshot.at("section_generated_root_r"), ok,
                            name + " section generated root");
    compare_optional_number(section.root_form_radius_mm,
                            snapshot.at("section_root_form_r"), ok,
                            name + " section root form");
    compare_optional_number(section.start_of_involute_angle_rad,
                            snapshot.at("section_start_of_involute_angle"), ok,
                            name + " section start angle");
    compare_optional_number(section.involute_roll_parameter,
                            snapshot.at("section_involute_roll_parameter"), ok,
                            name + " section roll");
    compare_optional_bool(section.undercut, snapshot.at("section_undercut"),
                          ok, name + " section undercut");
    compare_points(section.loop_2d, snapshot.at("section_loop"), ok,
                   name + " section loop");

    const auto& expected_segments =
        std::get<Json::Object>(snapshot.at("section_segments").value);
    for (const auto& [segment_name, expected] : expected_segments) {
        const auto* segment = section.segments.empty()
                                  ? nullptr
                                  : [&]() -> const std::vector<geargen::core::Point2>* {
                                        for (const auto& item : section.segments) {
                                            if (item.name == segment_name) {
                                                return &item.points;
                                            }
                                        }
                                        return nullptr;
                                    }();
        const std::vector<geargen::core::Point2> empty;
        compare_points(segment == nullptr ? empty : *segment, expected, ok,
                       name + " segment " + segment_name);
    }
}

void compare_spur(const Json& fixture, bool& ok)
{
    const std::string name = string(fixture.at("name"));
    geargen::core::SpurSetParams p;
    if (name == "spur_external") {
        p = geargen::core::SpurSetParams::with_defaults(2.0, 20, 40);
    } else if (name == "spur_helical") {
        geargen::core::SpurDefaultOverrides o;
        o.helix_angle = 15.0;
        p = geargen::core::SpurSetParams::with_defaults(2.0, 17, 43, o);
    } else if (name == "spur_internal_profile_shift") {
        geargen::core::SpurDefaultOverrides o;
        o.internal = true;
        o.profile_shift_1 = 0.3;
        p = geargen::core::SpurSetParams::with_defaults(2.0, 18, 60, o);
    } else if (name == "spur_rack_generated") {
        geargen::core::SpurDefaultOverrides o;
        o.root_geometry = "rack_generated";
        p = geargen::core::SpurSetParams::with_defaults(2.0, 12, 43, o);
    } else {
        p = geargen::core::SpurSetParams::with_defaults(2.0, 20, 40);
        p.module = -1.0;
        p.z1 = 3;
        p.pressure_angle = 30.0;
    }
    const auto validation = geargen::core::validate(p);
    compare_validation(validation, fixture.at("validation"), ok, name);
    if (!validation.ok()) {
        return;
    }
    const auto geometry = geargen::core::spur::derive(p);
    const auto& derived = fixture.at("derived");
    ok &= compare(p.transverse_module(), derived.at("transverse_module"), name + " m_t");
    ok &= compare(p.alpha_n(), derived.at("alpha_n"), name + " alpha_n");
    ok &= compare(p.alpha_t(), derived.at("alpha_t"), name + " alpha_t");
    ok &= compare(p.beta(), derived.at("beta"), name + " beta");
    ok &= compare(p.reference_centre_distance(), derived.at("reference_centre_distance"), name + " reference distance");
    ok &= compare(geometry.working_centre_distance_mm, derived.at("working_centre_distance"), name + " working distance");
    ok &= compare(geometry.working_pressure_angle_rad, derived.at("working_pressure_angle"), name + " working angle");
    ok &= compare(geometry.circular_pitch_mm, derived.at("circular_pitch"), name + " circular pitch");
    ok &= compare(geometry.whole_depth_mm, derived.at("whole_depth"), name + " whole depth");
    ok &= compare(geometry.tip_alteration_coefficient,
                  derived.at("tip_alteration_coefficient"),
                  name + " tip alteration");
    ok &= compare(geometry.working_depth_mm, derived.at("working_depth"),
                  name + " working depth");
    ok &= compare(geometry.tip_clearance_1_mm, derived.at("tip_clearance_1"),
                  name + " tip clearance 1");
    ok &= compare(geometry.tip_clearance_2_mm, derived.at("tip_clearance_2"),
                  name + " tip clearance 2");
    ok &= compare(geometry.minimum_tip_clearance_mm,
                  derived.at("minimum_tip_clearance"),
                  name + " minimum tip clearance");
    ok &= compare(geometry.path_of_contact_mm, derived.at("path_of_contact"),
                  name + " path of contact");
    ok &= check(geometry.contact_ratio_basis ==
                    string(derived.at("contact_ratio_basis")),
                name + " contact ratio basis");
    ok &= compare(geometry.transverse_contact_ratio,
                  derived.at("transverse_contact_ratio"),
                  name + " transverse contact ratio");
    ok &= compare(geometry.axial_contact_ratio,
                  derived.at("axial_contact_ratio"),
                  name + " axial contact ratio");
    for (const auto& item : {std::pair{"pinion", &geometry.pinion},
                             std::pair{"gear", &geometry.gear}}) {
        const auto& expected = derived.at(item.first);
        ok &= compare(item.second->reference_radius_mm, expected.at("reference_r"), name + " " + item.first + " reference");
        ok &= compare(item.second->base_radius_mm, expected.at("base_r"), name + " " + item.first + " base");
        ok &= compare(item.second->working_radius_mm, expected.at("working_r"), name + " " + item.first + " working");
        ok &= compare(item.second->tip_radius_mm, expected.at("tip_r"), name + " " + item.first + " tip");
        ok &= compare(item.second->root_radius_mm, expected.at("root_r"), name + " " + item.first + " root");
        ok &= compare(item.second->twist_rad, expected.at("twist"), name + " " + item.first + " twist");
    }
    compare_spur_geometry(geometry, fixture.at("geometry"), ok, name);
}

void compare_bevel(const Json& fixture, bool& ok)
{
    const std::string name = string(fixture.at("name"));
    geargen::core::BevelSetParams p;
    if (name == "bevel_straight") {
        p = geargen::core::BevelSetParams::with_defaults(2.0, 25, 40);
    } else if (name == "bevel_spiral") {
        geargen::core::BevelDefaultOverrides o;
        o.spiral_angle = 35.0;
        p = geargen::core::BevelSetParams::with_defaults(2.0, 25, 40, false, o);
    } else {
        p = geargen::core::BevelSetParams::with_defaults(2.0, 25, 40, true);
    }
    const auto validation = geargen::core::validate(p);
    compare_validation(validation, fixture.at("validation"), ok, name);
    if (!validation.ok()) {
        return;
    }
    const auto geometry = geargen::core::bevel::derive(p);
    const auto& derived = fixture.at("derived");
    ok &= compare(p.alpha(), derived.at("alpha"), name + " alpha");
    ok &= compare(p.sigma(), derived.at("sigma"), name + " sigma");
    ok &= compare(p.psi_m(), derived.at("psi_m"), name + " psi");
    ok &= check(std::string(p.trace_kind()) == string(derived.at("trace_kind")), name + " trace kind");
    ok &= check(p.is_curved() == boolean(derived.at("is_curved")), name + " curved flag");
    ok &= compare(geometry.outer_cone_dist_mm, derived.at("outer_cone_dist"), name + " outer cone");
    ok &= compare(geometry.mean_cone_dist_mm, derived.at("mean_cone_dist"), name + " mean cone");
    ok &= compare(geometry.inner_cone_dist_mm, derived.at("inner_cone_dist"), name + " inner cone");
    ok &= compare(geometry.working_depth_mm, derived.at("working_depth"), name + " working depth");
    ok &= compare(geometry.whole_depth_mm, derived.at("whole_depth"), name + " whole depth");
    ok &= compare(geometry.clearance_mm, derived.at("clearance"), name + " clearance");
    ok &= compare(geometry.pinion.pitch_angle_rad, derived.at("pinion_pitch_angle"), name + " pinion angle");
    ok &= compare(geometry.gear.pitch_angle_rad, derived.at("gear_pitch_angle"), name + " gear angle");
    ok &= compare(geometry.pinion.virtual_teeth, derived.at("pinion_virtual_teeth"), name + " pinion virtual teeth");
    ok &= compare(geometry.gear.virtual_teeth, derived.at("gear_virtual_teeth"), name + " gear virtual teeth");
}

void compare_hypoid(const Json& fixture, bool& ok)
{
    geargen::core::HypoidDefaultOverrides o;
    o.offset = 15.0;
    o.face_width = 30.0;
    o.spiral_angle = 50.0;
    o.cutter_radius = 63.5;
    const auto p = geargen::core::HypoidSetParams::with_defaults(
        170.0 / 42.0, 13, 42, o);
    const auto validation = geargen::core::validate(p);
    compare_validation(validation, fixture.at("validation"), ok, "hypoid_offset_sample");
    if (!validation.ok()) {
        return;
    }
    const auto& derived = fixture.at("derived");
    ok &= compare(p.alpha(), derived.at("alpha"), "hypoid alpha");
    ok &= compare(p.sigma(), derived.at("sigma"), "hypoid sigma");
    ok &= compare(p.psi1(), derived.at("psi1"), "hypoid psi");
    ok &= compare(p.ratio(), derived.at("ratio"), "hypoid ratio");
    ok &= compare(p.wheel_outer_diameter(), derived.at("wheel_outer_diameter"), "hypoid wheel diameter");
    ok &= compare(p.effective_root_fillet_radius(), derived.at("effective_root_fillet_radius"), "hypoid root fillet");
}

void compare_planetary(const Json& fixture, bool& ok)
{
    const std::string name = string(fixture.at("name"));
    geargen::core::PlanetaryDefaultOverrides o;
    if (name == "planetary_anchor") {
        o.n_planets = 3;
    } else {
        o.n_planets = 5;
    }
    const auto p = name == "planetary_anchor"
        ? geargen::core::PlanetarySetParams::with_defaults(2.0, 24, 18, o)
        : geargen::core::PlanetarySetParams::with_defaults(2.0, 30, 20, o);
    const auto validation = geargen::core::validate(p);
    compare_validation(validation, fixture.at("validation"), ok, name);
    if (!validation.ok()) {
        return;
    }
    const auto geometry = geargen::core::planetary::derive(p);
    const auto& derived = fixture.at("derived");
    ok &= check(p.z_ring() == static_cast<int>(number(derived.at("z_ring"))), name + " ring count");
    ok &= compare(p.alpha_n(), derived.at("alpha_n"), name + " alpha n");
    ok &= compare(p.alpha_t(), derived.at("alpha_t"), name + " alpha t");
    ok &= compare(p.beta(), derived.at("beta"), name + " beta");
    ok &= compare(p.transverse_module(), derived.at("transverse_module"), name + " transverse module");
    ok &= compare(p.centre_distance(), derived.at("centre_distance"), name + " centre distance");
    ok &= check(p.assembly_remainder() == static_cast<int>(number(derived.at("assembly_remainder"))), name + " assembly remainder");
    ok &= compare(p.ratio_carrier_to_sun(), derived.at("ratio_carrier_to_sun"), name + " carrier ratio");
    ok &= compare(p.ratio_ring_to_sun(), derived.at("ratio_ring_to_sun"), name + " ring ratio");
    ok &= compare(geometry.circular_pitch_mm, derived.at("circular_pitch"), name + " circular pitch");
    ok &= compare(geometry.whole_depth_mm, derived.at("whole_depth"), name + " whole depth");

    const auto& snapshot_geometry = fixture.at("geometry");
    std::vector<double> planet_angles;
    std::vector<geargen::core::Point3> translations;
    std::vector<double> planet_clockings;
    std::vector<double> ring_clockings;
    for (int index = 0; index < p.n_planets; ++index) {
        planet_angles.push_back(
            geargen::core::planetary::mesh::carrier_angle(geometry, index));
        translations.push_back(
            geargen::core::planetary::mesh::planet_translation(geometry, index));
        planet_clockings.push_back(
            geargen::core::planetary::mesh::planet_clocking(geometry, index));
        ring_clockings.push_back(
            geargen::core::planetary::mesh::ring_clocking(geometry, index));
    }
    compare_numbers(planet_angles, snapshot_geometry.at("planet_angles"), ok,
                    name + " planet angles");
    compare_points3(translations, snapshot_geometry.at("planet_translations"),
                    ok, name + " planet translations");
    compare_numbers(planet_clockings,
                    snapshot_geometry.at("planet_clocking"), ok,
                    name + " planet clocking");
    compare_numbers(ring_clockings, snapshot_geometry.at("ring_clocking"), ok,
                    name + " ring clocking");
}

} // namespace

int main()
{
    try {
        std::ifstream input(GEARGEN_REFERENCE_FIXTURE_PATH);
        if (!input) {
            std::cerr << "reference fixture not found: "
                      << GEARGEN_REFERENCE_FIXTURE_PATH << '\n';
            return 1;
        }
        const Json root = JsonParser{
            std::string(std::istreambuf_iterator<char>(input),
                        std::istreambuf_iterator<char>{})}
                              .parse();
        const auto& fixtures = std::get<Json::Array>(root.at("fixtures").value);
        bool ok = true;
        for (const auto& fixture : fixtures) {
            const std::string type = string(fixture.at("type"));
            if (type == "spur") {
                compare_spur(fixture, ok);
            } else if (type == "bevel") {
                compare_bevel(fixture, ok);
            } else if (type == "hypoid") {
                compare_hypoid(fixture, ok);
            } else if (type == "planetary") {
                compare_planetary(fixture, ok);
            } else {
                ok &= check(false, "unknown fixture type " + type);
            }
        }
        return ok ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "reference regression exception: " << error.what() << '\n';
        return 1;
    }
}
