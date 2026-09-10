#include "solidworks/assembly_builder.hpp"

#ifdef _WIN32

#include "core/bevel/mesh.hpp"
#include "core/hypoid/mesh.hpp"
#include "core/placement/placement.hpp"
#include "core/planetary/mesh.hpp"
#include "core/spur/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <map>
#include <sstream>

namespace geargen::solidworks::detail {

namespace {

enum class SelectionKind { Feature, Point, Plane };

struct Selection {
    SelectionKind kind;
    com::Dispatch object;
    std::vector<std::string> names;
    std::string what;
    long mark{kMarkMateEntity};
};

Selection feature_selection(const com::Dispatch& feature, std::string what)
{
    return {SelectionKind::Feature, feature, {}, std::move(what), kMarkMateEntity};
}

Selection point_selection(const com::Dispatch& point, std::string what)
{
    return {SelectionKind::Point, point, {}, std::move(what), 0};
}

Selection plane_selection(std::vector<std::string> names, std::string what)
{
    return {SelectionKind::Plane, {}, std::move(names), std::move(what), kMarkMateEntity};
}

void select_entity(const com::Dispatch& model, const Selection& selection, bool append)
{
    switch (selection.kind) {
    case SelectionKind::Feature:
        require_true(call(selection.object, "Select2",
                          {com::Variant(append), com::Variant(selection.mark)},
                          selection.what), selection.what);
        break;
    case SelectionKind::Point:
        require_true(call(selection.object, "Select4",
                          {com::Variant(append), com::Variant::null_dispatch()},
                          selection.what), selection.what);
        break;
    case SelectionKind::Plane:
        select_first(model, selection.names, "PLANE", append, selection.mark);
        break;
    }
}

void clear_selection(const com::Dispatch& model)
{
    call(model, "ClearSelection2", {com::Variant(true)}, "clear assembly selection");
}

com::Dispatch add_component(const com::Dispatch& model, const std::filesystem::path& path)
{
    return require_dispatch(
        call(model, "AddComponent5",
             {com::Variant(path.string()),
              com::Variant(static_cast<long>(AddComponentConfigOptions::CurrentConfiguration)),
              com::Variant(""), com::Variant(false), com::Variant(""),
              com::Variant(0.0), com::Variant(0.0), com::Variant(0.0)},
             "insert component " + path.string()),
        "insert component " + path.string());
}

void place(const Session& session, const com::Dispatch& component,
           const core::Matrix3& matrix, std::string_view what,
           core::Point3 translation_mm = {})
{
    const auto array = core::placement::to_solidworks_array(
        matrix, {millimeters_to_meters(translation_mm.x),
                 millimeters_to_meters(translation_mm.y),
                 millimeters_to_meters(translation_mm.z)});
    const auto transform = dispatch(session.math_utility(), "CreateTransform",
                                    {com::Variant::doubles(std::vector<double>(array.begin(), array.end()))},
                                    "build component transform for " + std::string(what));
    component.put("Transform2", com::Variant(transform.get()),
                  "place component " + std::string(what));
}

core::Point3 transform_vector(const com::Dispatch& component, std::size_t offset)
{
    const auto transform = property_dispatch(component, "Transform2", "read component transform");
    const auto data = value(transform, "ArrayData", "read component transform data").as_doubles();
    if (data.size() < 12 || offset + 2 >= data.size())
        throw SwError("component transform returned too few values");
    return {data[offset], data[offset + 1], data[offset + 2]};
}

core::Point3 measure_axis(const com::Dispatch& component)
{
    return transform_vector(component, 6);
}

core::Point3 measure_origin(const com::Dispatch& component)
{
    const auto point = transform_vector(component, 9);
    return {point.x * 1000.0, point.y * 1000.0, point.z * 1000.0};
}

com::Dispatch feature_of_type(const com::Dispatch& owner, std::string_view type,
                              std::string_view what)
{
    auto current_result = call(owner, "FirstFeature", {}, "read first feature");
    while (!current_result.is_nullish()) {
        const auto current = require_dispatch(current_result, "read feature");
        if (string_value(current, "GetTypeName2", "read feature type") == type)
            return current;
        current_result = call(current, "GetNextFeature", {}, "walk feature tree");
    }
    throw SwError("could not find " + std::string(what));
}

com::Dispatch component_axis(const com::Dispatch& component, std::string_view what)
{
    try {
        const auto result = call(component, "FeatureByName",
                                {com::Variant(kAxisFeatureName)},
                                "find named component axis");
        if (!result.is_nullish()) return require_dispatch(result, "read named component axis");
    } catch (...) {
    }
    return feature_of_type(component, "RefAxis", std::string(what) + " axis");
}

com::Dispatch origin_point(const com::Dispatch& owner, std::string_view what)
{
    const auto origin = feature_of_type(owner, "OriginProfileFeature", what);
    const auto sketch = dispatch(origin, "GetSpecificFeature2", {},
                                 "read sketch behind " + std::string(what));
    const auto points = dispatches(call(sketch, "GetSketchPoints2", {},
                                         "read origin sketch points"),
                                   "read origin sketch points");
    if (points.size() != 1)
        throw SwError("expected " + std::string(what) + " to hold one point, found " +
                      std::to_string(points.size()));
    return points.front();
}

void unfix(const com::Dispatch& model, const com::Dispatch& component,
           std::string_view what)
{
    clear_selection(model);
    require_true(call(component, "Select2", {com::Variant(false), com::Variant(0)},
                      "select " + std::string(what) + " to float"),
                 "select " + std::string(what) + " to float");
    call(model, "UnfixComponent", {}, "float " + std::string(what));
    clear_selection(model);
    if (bool_value(component, "IsFixed", "read component fixed state"))
        throw SwError(std::string(what) + " is still fixed");
}

void add_mate(const com::Dispatch& model, const std::vector<Selection>& selections,
              MateType type, std::string_view what, double angle_rad = 0.0,
              double distance_mm = 0.0, std::pair<double, double> ratio = {1.0, 1.0},
              bool flip = false)
{
    clear_selection(model);
    for (std::size_t i = 0; i < selections.size(); ++i)
        select_entity(model, selections[i], i != 0);
    com::Variant errors = com::Variant::byref_i4_variant();
    const auto result = call(
        model, "AddMate5",
        {com::Variant(static_cast<long>(type)), com::Variant(kMateAlignClosest),
         com::Variant(flip), com::Variant(millimeters_to_meters(distance_mm)),
         com::Variant(0.0), com::Variant(0.0), com::Variant(ratio.first),
         com::Variant(ratio.second), com::Variant(angle_rad), com::Variant(0.0),
         com::Variant(0.0), com::Variant(false), com::Variant(false),
         com::Variant(0), errors},
        std::string("add ") + std::string(what) + " mate");
    clear_selection(model);
    const long status = errors.byref_i4_value();
    if (result.is_nullish() || status != kAddMateNoError) {
        throw SwError("SOLIDWORKS would not add the " + std::string(what) +
                      " mate (status " + std::to_string(status) + ")");
    }
}

std::string component_status(const com::Dispatch& component)
{
    try {
        const long status = value(component, "GetConstrainedStatus",
                                  "read component constrained status").as_long();
        switch (status) {
        case 1: return "unknown";
        case 2: return "under defined";
        case 3: return "fully defined";
        case 4: return "over defined";
        case 5: return "no solution";
        case 6: return "invalid solution";
        case 7: return "autosolve off";
        default: return std::to_string(status);
        }
    } catch (...) {
        return "not checked";
    }
}

std::pair<int, double> check_interference(const com::Dispatch& model)
{
    try {
        const auto manager = property_dispatch(model, "InterferenceDetectionManager",
                                               "read interference manager");
        manager.put("TreatCoincidenceAsInterference", com::Variant(false),
                    "disable coincident-face interference");
        manager.put("TreatSubAssembliesAsComponents", com::Variant(false),
                    "disable subassembly interference");
        const long count = value(manager, "GetInterferenceCount",
                                 "read interference count").as_long();
        double volume = 0.0;
        if (count != 0) {
            for (const auto& item : dispatches(call(manager, "GetInterferences", {},
                                                    "read interferences"),
                                               "read interferences")) {
                try { volume += number_value(item, "Volume", "read interference volume") * 1e9; }
                catch (...) {}
            }
        }
        try { call(manager, "Done", {}, "finish interference detection"); } catch (...) {}
        return {static_cast<int>(count), volume};
    } catch (...) {
        return {-1, 0.0};
    }
}

void rebuild(const com::Dispatch& model)
{
    call(model, "EditRebuild3", {}, "rebuild assembly");
    try { call(model, "ViewZoomtofit2", {}, "fit assembly view"); } catch (...) {}
}

std::string module_token(double module)
{
    std::ostringstream stream;
    stream << std::setprecision(6) << std::defaultfloat << module;
    auto result = stream.str();
    std::replace(result.begin(), result.end(), '.', 'p');
    return result;
}

template <typename Geometry>
std::string pair_part_filename(const Geometry& geometry, std::string_view member,
                               std::string_view prefix = {})
{
    const auto& m = geometry.member(std::string(member));
    const int teeth = [&] {
        if constexpr (requires { m.teeth; }) return m.teeth;
        else return m.z;
    }();
    return std::string(prefix) + std::string(member) + "_m" + module_token(geometry.parameters.module) +
           "_z" + std::to_string(teeth) + ".sldprt";
}

std::filesystem::path prepare_output(const std::filesystem::path& input)
{
    const auto output = std::filesystem::absolute(input.empty() ? std::filesystem::current_path() : input);
    std::filesystem::create_directories(output);
    return output;
}

void append_parts(BuildResult& result, const PartArtifact& first,
                  const PartArtifact& second)
{
    result.parts.push_back(first.report);
    result.parts.push_back(second.report);
}

void common_pair_report(BuildResult& result, const com::Dispatch& model,
                        const Session& session, const std::filesystem::path& output,
                        std::string filename, std::vector<std::string> mates,
                        std::vector<std::pair<double, double>> ratios,
                        const BuildOptions& options)
{
    result.mates = std::move(mates);
    result.gear_ratios = std::move(ratios);
    const auto [count, volume] = check_interference(model);
    result.interference_count = count;
    result.interference_volume_mm3 = volume;
    if (options.save_assembly) {
        result.assembly_path = output / filename;
        session.save(model, result.assembly_path);
    }
}

std::pair<std::vector<std::string>, std::vector<std::pair<double, double>>>
add_spur_mates(const com::Dispatch& model, const com::Dispatch& pinion,
               const com::Dispatch& gear, const core::spur::SetGeometry& geometry,
               bool reverse)
{
    unfix(model, pinion, "pinion");
    unfix(model, gear, "gear");
    const auto origin = point_selection(origin_point(model, "assembly origin"), "assembly origin");
    const auto top = plane_selection({"Top Plane", "Top", "Plane2"}, "assembly Top plane");
    const auto right = plane_selection({"Right Plane", "Right", "Plane3"}, "assembly Right plane");
    const auto front = plane_selection({"Front Plane", "Front", "Plane1"}, "assembly Front plane");
    const auto pinion_axis = feature_selection(component_axis(pinion, "pinion"), "pinion axis");
    const auto gear_axis = feature_selection(component_axis(gear, "gear"), "gear axis");
    const auto pinion_origin = point_selection(origin_point(pinion, "pinion origin"), "pinion origin");
    const auto gear_origin = point_selection(origin_point(gear, "gear origin"), "gear origin");
    std::vector<std::string> names;
    const auto mate = [&](std::vector<Selection> picks, MateType type, std::string what,
                          double angle = 0.0, double distance = 0.0,
                          std::pair<double, double> ratio = {1.0, 1.0}, bool flip = false) {
        add_mate(model, picks, type, what, angle, distance, ratio, flip);
        names.push_back(std::move(what));
    };
    mate({pinion_origin, origin}, MateType::Coincident, "pinion origin - assembly origin");
    mate({pinion_axis, top}, MateType::Coincident, "pinion axis - Top plane");
    mate({pinion_axis, right}, MateType::Coincident, "pinion axis - Right plane");
    mate({gear_axis, pinion_axis}, MateType::Parallel, "gear axis parallel to pinion axis");
    mate({gear_axis, top}, MateType::Coincident, "gear axis - Top plane");
    mate({gear_origin, front}, MateType::Coincident, "gear front face - Front plane");
    mate({gear_axis, pinion_axis}, MateType::Distance,
         "working centre distance", 0.0, geometry.working_centre_distance_mm);
    const auto ratio = core::spur::mesh::mate_ratio(geometry);
    mate({pinion_axis, gear_axis}, MateType::Gear, "gear mate", 0.0, 0.0, ratio, reverse);
    return {std::move(names), {ratio}};
}

std::pair<std::vector<std::string>, std::vector<std::pair<double, double>>>
add_bevel_mates(const com::Dispatch& model, const com::Dispatch& pinion,
                const com::Dispatch& gear, const core::bevel::SetGeometry& geometry,
                bool reverse)
{
    unfix(model, pinion, "pinion");
    unfix(model, gear, "gear");
    const auto origin = point_selection(origin_point(model, "assembly origin"), "assembly origin");
    const auto top = plane_selection({"Top Plane", "Top", "Plane2"}, "assembly Top plane");
    const auto right = plane_selection({"Right Plane", "Right", "Plane3"}, "assembly Right plane");
    const auto pinion_axis = feature_selection(component_axis(pinion, "pinion"), "pinion axis");
    const auto gear_axis = feature_selection(component_axis(gear, "gear"), "gear axis");
    const auto pinion_origin = point_selection(origin_point(pinion, "pinion apex"), "pinion apex");
    const auto gear_origin = point_selection(origin_point(gear, "gear apex"), "gear apex");
    std::vector<std::string> names;
    const auto mate = [&](std::vector<Selection> picks, MateType type, std::string what,
                          double angle = 0.0, double distance = 0.0,
                          std::pair<double, double> ratio = {1.0, 1.0}, bool flip = false) {
        add_mate(model, picks, type, what, angle, distance, ratio, flip);
        names.push_back(std::move(what));
    };
    mate({pinion_origin, origin}, MateType::Coincident, "pinion apex - assembly origin");
    mate({pinion_axis, top}, MateType::Coincident, "pinion axis - Top plane");
    mate({pinion_axis, right}, MateType::Coincident, "pinion axis - Right plane");
    mate({gear_origin, origin}, MateType::Coincident, "gear apex - assembly origin");
    mate({gear_axis, top}, MateType::Coincident, "gear axis - Top plane");
    mate({gear_axis, pinion_axis}, MateType::Angle, "shaft angle", geometry.parameters.sigma());
    const auto ratio = core::placement::gear_mate_ratio(geometry.pinion.z, geometry.gear.z);
    mate({pinion_axis, gear_axis}, MateType::Gear, "gear mate", 0.0, 0.0, ratio, reverse);
    return {std::move(names), {ratio}};
}

std::pair<std::vector<std::string>, std::vector<std::pair<double, double>>>
add_hypoid_mates(const com::Dispatch& model, const com::Dispatch& pinion,
                 const com::Dispatch& gear, const core::hypoid::SetGeometry& geometry,
                 bool reverse)
{
    unfix(model, pinion, "pinion");
    unfix(model, gear, "gear");
    const auto origin = point_selection(origin_point(model, "assembly origin"), "assembly origin");
    const auto top = plane_selection({"Top Plane", "Top", "Plane2"}, "assembly Top plane");
    const auto right = plane_selection({"Right Plane", "Right", "Plane3"}, "assembly Right plane");
    const auto front = plane_selection({"Front Plane", "Front", "Plane1"}, "assembly Front plane");
    const auto pinion_axis = feature_selection(component_axis(pinion, "pinion"), "pinion axis");
    const auto gear_axis = feature_selection(component_axis(gear, "gear"), "gear axis");
    const auto pinion_origin = point_selection(origin_point(pinion, "pinion origin"), "pinion origin");
    const auto gear_origin = point_selection(origin_point(gear, "gear origin"), "gear origin");
    const auto position = core::hypoid::mesh::gear_translation(geometry);
    std::vector<std::string> names;
    const auto mate = [&](std::vector<Selection> picks, MateType type, std::string what,
                          double angle = 0.0, double distance = 0.0,
                          std::pair<double, double> ratio = {1.0, 1.0}, bool flip = false) {
        add_mate(model, picks, type, what, angle, distance, ratio, flip);
        names.push_back(std::move(what));
    };
    mate({pinion_origin, origin}, MateType::Coincident, "pinion origin - assembly origin");
    mate({pinion_axis, top}, MateType::Coincident, "pinion axis - Top plane");
    mate({pinion_axis, right}, MateType::Coincident, "pinion axis - Right plane");
    if (std::abs(geometry.parameters.offset) < kCoordinateToleranceMm)
        mate({gear_axis, top}, MateType::Coincident, "gear axis - Top plane");
    else
        mate({gear_axis, top}, MateType::Distance, "hypoid offset", 0.0,
             std::abs(geometry.parameters.offset), {1.0, 1.0}, geometry.parameters.offset < 0.0);
    const auto coordinate_mate = [&](double coordinate, const Selection& plane,
                                     std::string label) {
        if (std::abs(coordinate) < kCoordinateToleranceMm)
            mate({gear_origin, plane}, MateType::Coincident, "gear origin - " + label + " plane");
        else
            mate({gear_origin, plane}, MateType::Distance, "gear origin " + label,
                 0.0, std::abs(coordinate), {1.0, 1.0}, coordinate < 0.0);
    };
    coordinate_mate(position.x, right, "Right");
    coordinate_mate(position.z, front, "Front");
    mate({gear_axis, pinion_axis}, MateType::Angle, "shaft angle", geometry.parameters.sigma());
    const auto ratio = core::placement::gear_mate_ratio(geometry.pinion.z, geometry.gear.z);
    mate({pinion_axis, gear_axis}, MateType::Gear, "gear mate", 0.0, 0.0, ratio, reverse);

    try {
        call(model, "EditRebuild3", {}, "rebuild hypoid mate group");
        const auto group = feature_of_type(model, "MateGroup", "assembly MateGroup");
        auto current = call(group, "GetFirstSubFeature", {}, "read MateGroup");
        com::Dispatch found;
        while (!current.is_nullish()) {
            const auto feature = require_dispatch(current, "read mate feature");
            if (string_value(feature, "GetTypeName2", "read mate type") == "MateGearDim") found = feature;
            current = call(feature, "GetNextSubFeature", {}, "walk MateGroup");
        }
        if (!found) throw SwError("gear mate was accepted but is absent from MateGroup");
        const std::string name = "HypoidGearMate_" + std::to_string(geometry.pinion.z) +
                                 "_" + std::to_string(geometry.gear.z);
        found.put("Name", com::Variant(name), "name persisted hypoid gear mate");
    } catch (const SwError&) {
        throw;
    }
    return {std::move(names), {ratio}};
}

std::pair<std::vector<std::string>, std::vector<std::pair<double, double>>>
add_planetary_mates(const com::Dispatch& model, const com::Dispatch& sun,
                    const com::Dispatch& ring,
                    const std::vector<com::Dispatch>& planets,
                    const core::planetary::SetGeometry& geometry, bool reverse)
{
    unfix(model, sun, "sun");
    unfix(model, ring, "ring");
    for (std::size_t i = 0; i < planets.size(); ++i) unfix(model, planets[i], "planet " + std::to_string(i));
    const auto origin = point_selection(origin_point(model, "assembly origin"), "assembly origin");
    const auto top = plane_selection({"Top Plane", "Top", "Plane2"}, "assembly Top plane");
    const auto right = plane_selection({"Right Plane", "Right", "Plane3"}, "assembly Right plane");
    const auto front = plane_selection({"Front Plane", "Front", "Plane1"}, "assembly Front plane");
    const auto sun_axis = feature_selection(component_axis(sun, "sun"), "sun axis");
    const auto ring_axis = feature_selection(component_axis(ring, "ring"), "ring axis");
    const auto sun_origin = point_selection(origin_point(sun, "sun origin"), "sun origin");
    const auto ring_origin = point_selection(origin_point(ring, "ring origin"), "ring origin");
    std::vector<std::string> names;
    const auto mate = [&](std::vector<Selection> picks, MateType type, std::string what,
                          double angle = 0.0, double distance = 0.0,
                          std::pair<double, double> ratio = {1.0, 1.0}, bool flip = false) {
        add_mate(model, picks, type, what, angle, distance, ratio, flip);
        names.push_back(std::move(what));
    };
    mate({sun_origin, origin}, MateType::Coincident, "sun origin - assembly origin");
    mate({sun_axis, top}, MateType::Coincident, "sun axis - Top plane");
    mate({sun_axis, right}, MateType::Coincident, "sun axis - Right plane");
    mate({ring_axis, sun_axis}, MateType::Coincident, "ring axis - sun axis");
    mate({ring_origin, front}, MateType::Coincident, "ring front face - Front plane");
    for (std::size_t i = 0; i < planets.size(); ++i) {
        const auto axis = feature_selection(component_axis(planets[i], "planet " + std::to_string(i)),
                                            "planet " + std::to_string(i) + " axis");
        const auto point = point_selection(origin_point(planets[i], "planet " + std::to_string(i) + " origin"),
                                           "planet " + std::to_string(i) + " origin");
        mate({axis, sun_axis}, MateType::Parallel, "planet " + std::to_string(i) + " axis parallel");
        mate({point, front}, MateType::Coincident, "planet " + std::to_string(i) + " front face");
        mate({axis, sun_axis}, MateType::Distance, "planet " + std::to_string(i) + " orbit radius",
             0.0, geometry.centre_distance_mm);
        const double offset = geometry.centre_distance_mm *
                              std::sin(core::planetary::mesh::carrier_angle(geometry, static_cast<int>(i)));
        if (std::abs(offset) < 1e-9)
            mate({axis, top}, MateType::Coincident, "planet " + std::to_string(i) + " axis - Top plane");
        else
            mate({axis, top}, MateType::Distance, "planet " + std::to_string(i) + " station",
                 0.0, std::abs(offset), {1.0, 1.0}, offset < 0.0);
    }
    const bool sun_flip = false != reverse;
    const bool ring_flip = true != reverse;
    std::vector<std::pair<double, double>> ratios;
    const auto sun_ratio = core::planetary::mesh::sun_planet_ratio(geometry);
    const auto ring_ratio = core::planetary::mesh::planet_ring_ratio(geometry);
    for (std::size_t i = 0; i < planets.size(); ++i) {
        const auto axis = feature_selection(component_axis(planets[i], "planet " + std::to_string(i)),
                                            "planet " + std::to_string(i) + " axis");
        mate({sun_axis, axis}, MateType::Gear, "gear mate sun:planet " + std::to_string(i),
             0.0, 0.0, sun_ratio, sun_flip);
        ratios.push_back(sun_ratio);
        if (i == 0) {
            mate({axis, ring_axis}, MateType::Gear, "gear mate planet:ring",
                 0.0, 0.0, ring_ratio, ring_flip);
            ratios.push_back(ring_ratio);
        }
    }
    return {std::move(names), std::move(ratios)};
}

BuildResult build_pair_set(const Session& session, const BuildOptions& options,
                           std::string kind, const std::filesystem::path& output,
                           PartArtifact first, PartArtifact second,
                           const std::function<void(const Session&, const com::Dispatch&,
                                                    const com::Dispatch&)>& placer,
                           const std::function<std::pair<std::vector<std::string>,
                                                         std::vector<std::pair<double, double>>>(
                               const com::Dispatch&, const com::Dispatch&, const com::Dispatch&, bool)>& mate_builder,
                           std::string assembly_filename)
{
    const auto model = session.new_assembly();
    const auto pinion = add_component(model, first.report.path);
    const auto gear = add_component(model, second.report.path);
    placer(session, pinion, gear);
    BuildResult result;
    result.gear_kind = std::move(kind);
    append_parts(result, first, second);
    std::vector<std::string> mates;
    std::vector<std::pair<double, double>> ratios;
    if (options.mate) {
        auto built = mate_builder(model, pinion, gear, options.reverse_gear_mate);
        mates = std::move(built.first);
        ratios = std::move(built.second);
    }
    rebuild(model);
    common_pair_report(result, model, session, output, std::move(assembly_filename),
                       std::move(mates), std::move(ratios), options);
    result.ok = true;
    result.message = "SOLIDWORKS " + result.gear_kind + " assembly built";
    return result;
}

} // namespace

BuildResult build_spur_set(const Session& session, const core::spur::SetGeometry& geometry,
                           const std::filesystem::path& input_output,
                           const BuildOptions& options)
{
    const auto output = prepare_output(input_output);
    const std::string prefix = geometry.parameters.internal ? "internal_" : "spur_";
    const auto pinion_path = output / pair_part_filename(geometry, "pinion", prefix);
    const auto gear_path = output / pair_part_filename(geometry, "gear", prefix);
    auto pinion = build_spur_part(session, geometry, "pinion", pinion_path);
    auto gear = build_spur_part(session, geometry, "gear", gear_path);
    const std::string assembly = prefix + "m" + module_token(geometry.parameters.module) + "_z" +
                                 std::to_string(geometry.pinion.teeth) + "x" +
                                 std::to_string(geometry.gear.teeth) + ".sldasm";
    return build_pair_set(session, options, "spur", output, std::move(pinion), std::move(gear),
                          [&](const auto& session_ref, const auto& a, const auto& b) {
                              place(session_ref, a, core::placement::pinion_placement(), "pinion");
                              place(session_ref, b,
                                    core::spur::mesh::gear_placement(
                                        core::spur::mesh::clocking_for(geometry)),
                                    "gear", core::spur::mesh::gear_translation(geometry));
                          },
                          [&](const auto& model, const auto& a, const auto& b, bool reverse) {
                              return add_spur_mates(model, a, b, geometry, reverse);
                          }, assembly);
}

BuildResult build_bevel_set(const Session& session, const core::bevel::SetGeometry& geometry,
                            const std::filesystem::path& input_output,
                            const BuildOptions& options)
{
    const auto output = prepare_output(input_output);
    const auto pinion_path = output / pair_part_filename(geometry, "pinion");
    const auto gear_path = output / pair_part_filename(geometry, "gear");
    auto pinion = build_bevel_part(session, geometry, "pinion", pinion_path);
    auto gear = build_bevel_part(session, geometry, "gear", gear_path);
    const std::string assembly = "set_m" + module_token(geometry.parameters.module) + "_z" +
                                 std::to_string(geometry.pinion.z) + "x" +
                                 std::to_string(geometry.gear.z) + ".sldasm";
    return build_pair_set(session, options, "bevel", output, std::move(pinion), std::move(gear),
                          [&](const auto& session_ref, const auto& a, const auto& b) {
                              place(session_ref, a, core::placement::pinion_placement(), "pinion");
                              place(session_ref, b,
                                    core::bevel::mesh::gear_placement(
                                        geometry.parameters.sigma(),
                                        core::bevel::mesh::gear_clocking(geometry.gear.z)),
                                    "gear");
                          },
                          [&](const auto& model, const auto& a, const auto& b, bool reverse) {
                              return add_bevel_mates(model, a, b, geometry, reverse);
                          }, assembly);
}

BuildResult build_hypoid_set(const Session& session, const core::hypoid::SetGeometry& geometry,
                             const std::filesystem::path& input_output,
                             const BuildOptions& options)
{
    const auto output = prepare_output(input_output);
    const auto pinion_path = output / pair_part_filename(geometry, "pinion", "hypoid_");
    const auto gear_path = output / pair_part_filename(geometry, "gear", "hypoid_");
    auto pinion = build_hypoid_part(session, geometry, "pinion", pinion_path);
    auto gear = build_hypoid_part(session, geometry, "gear", gear_path);
    const std::string assembly = "hypoid_m" + module_token(geometry.parameters.module) + "_z" +
                                 std::to_string(geometry.pinion.z) + "x" +
                                 std::to_string(geometry.gear.z) + ".sldasm";
    return build_pair_set(session, options, "hypoid", output, std::move(pinion), std::move(gear),
                          [&](const auto& session_ref, const auto& a, const auto& b) {
                              place(session_ref, a, core::hypoid::mesh::pinion_placement(geometry), "pinion");
                              place(session_ref, b,
                                    core::hypoid::mesh::gear_placement(
                                        geometry.parameters.sigma(),
                                        core::hypoid::mesh::gear_clocking(geometry)),
                                    "gear", core::hypoid::mesh::gear_translation(geometry));
                          },
                          [&](const auto& model, const auto& a, const auto& b, bool reverse) {
                              return add_hypoid_mates(model, a, b, geometry, reverse);
                          }, assembly);
}

BuildResult build_planetary_set(const Session& session,
                                const core::planetary::SetGeometry& geometry,
                                const std::filesystem::path& input_output,
                                const BuildOptions& options)
{
    const auto output = prepare_output(input_output);
    const auto sun_path = output / ("planetary_sun_m" + module_token(geometry.parameters.module) +
                                    "_z" + std::to_string(geometry.sun.teeth) + ".sldprt");
    const auto planet_path = output / ("planetary_planet_m" + module_token(geometry.parameters.module) +
                                       "_z" + std::to_string(geometry.planet.teeth) + ".sldprt");
    const auto ring_path = output / ("planetary_ring_m" + module_token(geometry.parameters.module) +
                                     "_z" + std::to_string(geometry.ring.teeth) + ".sldprt");
    auto sun = build_spur_part(session, geometry.sun_planet, "pinion", sun_path);
    sun.report.member = "sun";
    auto planet = build_spur_part(session, geometry.sun_planet, "gear", planet_path);
    planet.report.member = "planet";
    auto ring = build_spur_part(session, geometry.planet_ring, "gear", ring_path);
    ring.report.member = "ring";

    const auto model = session.new_assembly();
    const auto sun_component = add_component(model, sun.report.path);
    const auto ring_component = add_component(model, ring.report.path);
    std::vector<com::Dispatch> planets;
    for (int i = 0; i < geometry.parameters.n_planets; ++i)
        planets.push_back(add_component(model, planet.report.path));
    place(session, sun_component, core::planetary::mesh::member_placement(
                                      core::planetary::mesh::sun_clocking()), "sun");
    place(session, ring_component, core::planetary::mesh::member_placement(
                                       core::planetary::mesh::ring_clocking(geometry)), "ring");
    for (std::size_t i = 0; i < planets.size(); ++i)
        place(session, planets[i], core::planetary::mesh::member_placement(
                                       core::planetary::mesh::planet_clocking(geometry, static_cast<int>(i))),
              "planet " + std::to_string(i),
              core::planetary::mesh::planet_translation(geometry, static_cast<int>(i)));

    BuildResult result;
    result.gear_kind = "planetary";
    result.parts = {sun.report, planet.report, ring.report};
    if (options.mate) {
        auto built = add_planetary_mates(model, sun_component, ring_component, planets,
                                         geometry, options.reverse_gear_mate);
        result.mates = std::move(built.first);
        result.gear_ratios = std::move(built.second);
    }
    rebuild(model);
    const auto [count, volume] = check_interference(model);
    result.interference_count = count;
    result.interference_volume_mm3 = volume;
    if (options.save_assembly) {
        result.assembly_path = output / ("planetary_m" + module_token(geometry.parameters.module) +
                                         "_s" + std::to_string(geometry.parameters.z_sun) +
                                         "p" + std::to_string(geometry.parameters.z_planet) +
                                         "r" + std::to_string(geometry.parameters.z_ring()) +
                                         "x" + std::to_string(geometry.parameters.n_planets) + ".sldasm");
        session.save(model, result.assembly_path);
    }
    result.ok = true;
    result.message = "SOLIDWORKS planetary assembly built";
    return result;
}

} // namespace geargen::solidworks::detail

#endif
