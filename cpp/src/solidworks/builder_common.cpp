#include "solidworks/builder_common.hpp"

#ifdef _WIN32

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace geargen::solidworks::detail {

namespace {

constexpr long kSolidBody = static_cast<long>(BodyType::Solid);

std::string step(std::string_view what, const std::exception& error)
{
    return std::string(what) + ": " + error.what();
}

[[nodiscard]] com::Dispatch extension(const com::Dispatch& model)
{
    return property_dispatch(model, "Extension", "read model Extension");
}

[[nodiscard]] com::Dispatch sketch_manager(const com::Dispatch& model)
{
    return property_dispatch(model, "SketchManager", "read SketchManager");
}

class SketchFlags {
public:
    explicit SketchFlags(const com::Dispatch& manager)
        : manager_(manager)
    {
        manager_.put("AddToDB", com::Variant(true), "enable SketchManager.AddToDB");
        manager_.put("DisplayWhenAdded", com::Variant(false),
                     "disable SketchManager.DisplayWhenAdded");
    }

    SketchFlags(const SketchFlags&) = delete;
    SketchFlags& operator=(const SketchFlags&) = delete;

    ~SketchFlags()
    {
        try {
            manager_.put("AddToDB", com::Variant(false), "restore SketchManager.AddToDB");
            manager_.put("DisplayWhenAdded", com::Variant(true),
                         "restore SketchManager.DisplayWhenAdded");
        } catch (...) {
        }
    }

private:
    com::Dispatch manager_;
};

void clear_selection(const com::Dispatch& model)
{
    call(model, "ClearSelection2", {com::Variant(true)}, "clear SOLIDWORKS selection");
}

[[nodiscard]] std::vector<com::Dispatch> dispatch_array(const com::Variant& result,
                                                         std::string_view what)
{
    try {
        std::vector<com::Dispatch> output;
        for (const auto& pointer : result.as_dispatches()) output.emplace_back(pointer.get());
        return output;
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

} // namespace

com::Variant call(const com::Dispatch& object, std::string_view member,
                  const std::vector<com::Variant>& arguments, std::string_view what)
{
    try {
        return object.call(member, arguments, what);
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

std::vector<com::Dispatch> dispatches(const com::Variant& result, std::string_view what)
{
    return dispatch_array(result, what);
}

com::Variant value(const com::Dispatch& object, std::string_view member,
                   std::string_view what)
{
    return call(object, member, {}, what);
}

com::Dispatch dispatch(const com::Dispatch& object, std::string_view member,
                       const std::vector<com::Variant>& arguments, std::string_view what)
{
    try {
        return object.call_dispatch(member, arguments, what);
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

com::Dispatch property_dispatch(const com::Dispatch& object, std::string_view member,
                                std::string_view what)
{
    try {
        return object.get_dispatch(member, what);
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

com::Dispatch require_dispatch(const com::Variant& result, std::string_view what)
{
    if (result.is_nullish()) throw SwError(std::string("SOLIDWORKS returned no object for ") + std::string(what));
    try {
        return com::Dispatch(result.as_dispatch());
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

void require_true(const com::Variant& result, std::string_view what)
{
    if (result.is_nullish()) throw SwError(std::string("SOLIDWORKS returned no result for ") + std::string(what));
    try {
        if (!result.as_bool()) throw SwError(std::string("SOLIDWORKS rejected ") + std::string(what));
    } catch (const SwError&) {
        throw;
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

std::string string_value(const com::Dispatch& object, std::string_view member,
                         std::string_view what)
{
    try {
        return value(object, member, what).as_string();
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

double number_value(const com::Dispatch& object, std::string_view member,
                    std::string_view what)
{
    try {
        return value(object, member, what).as_double();
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

bool bool_value(const com::Dispatch& object, std::string_view member,
                std::string_view what)
{
    try {
        return value(object, member, what).as_bool();
    } catch (const std::exception& error) {
        throw SwError(step(what, error));
    }
}

std::string equation_number(double value)
{
    std::ostringstream stream;
    stream << std::setprecision(9) << std::defaultfloat << value;
    return stream.str();
}

void select(const com::Dispatch& model, std::string_view name,
            std::string_view type, bool append, long mark)
{
    require_true(
        call(extension(model), "SelectByID2",
             {com::Variant(name), com::Variant(type), com::Variant(0.0),
              com::Variant(0.0), com::Variant(0.0), com::Variant(append),
              com::Variant(mark), com::Variant::null_dispatch(), com::Variant(0)},
             std::string("select ") + std::string(type) + " named " + std::string(name)),
        std::string("select ") + std::string(type) + " named " + std::string(name));
}

std::string select_first(const com::Dispatch& model, const std::vector<std::string>& names,
                         std::string_view type, bool append, long mark)
{
    for (const auto& name : names) {
        try {
            const auto result = call(
                extension(model), "SelectByID2",
                {com::Variant(name), com::Variant(type), com::Variant(0.0),
                 com::Variant(0.0), com::Variant(0.0), com::Variant(append),
                 com::Variant(mark), com::Variant::null_dispatch(), com::Variant(0)},
                "select candidate");
            if (!result.is_nullish() && result.as_bool()) return name;
        } catch (const std::exception&) {
        }
    }
    throw SwError("none of the configured names could be selected as " + std::string(type));
}

void select_entities(const com::Dispatch& model,
                     const std::vector<com::Dispatch>& entities,
                     std::string_view what)
{
    clear_selection(model);
    for (std::size_t i = 0; i < entities.size(); ++i) {
        require_true(
            call(entities[i], "Select4",
                 {com::Variant(i != 0), com::Variant::null_dispatch()},
                 std::string(what) + " entity " + std::to_string(i)),
            std::string(what) + " entity " + std::to_string(i));
    }
}

void select_feature(const com::Dispatch& feature, std::string_view what,
                    bool append, long mark)
{
    require_true(call(feature, "Select2", {com::Variant(append), com::Variant(mark)}, what), what);
}

com::Dispatch last_feature(const com::Dispatch& model)
{
    return dispatch(model, "FeatureByPositionReverse", {com::Variant(0)},
                    "read back the feature just created");
}

com::Dispatch endpoint(const com::Dispatch& line, bool start)
{
    return dispatch(line, start ? "GetStartPoint2" : "GetEndPoint2", {},
                    start ? "read line start point" : "read line end point");
}

com::Dispatch axis_datum(const com::Dispatch& axis)
{
    const auto first = endpoint(axis, true);
    const auto second = endpoint(axis, false);
    const double first_radius = std::hypot(number_value(first, "X", "read datum X"),
                                           number_value(first, "Y", "read datum Y"));
    const double second_radius = std::hypot(number_value(second, "X", "read endpoint X"),
                                            number_value(second, "Y", "read endpoint Y"));
    return first_radius <= second_radius ? first : second;
}

com::Dispatch create_axis(const com::Dispatch& model)
{
    clear_selection(model);
    select_first(model, {"Top Plane", "Top", "Plane2"}, "PLANE");
    select_first(model, {"Right Plane", "Right", "Plane3"}, "PLANE", true);
    require_true(call(model, "InsertAxis2", {com::Variant(true)}, "InsertAxis2"), "InsertAxis2");
    const auto axis = last_feature(model);
    try {
        axis.put("Name", com::Variant(kAxisFeatureName), "name reference axis");
    } catch (...) {
        // The feature object is still usable if a localized template refuses
        // the cosmetic rename; assembly lookup has a type-based fallback.
    }
    return axis;
}

DimensionFlags::DimensionFlags(const com::Dispatch& application)
    : application_(application)
{
    for (const long preference : {kPreferenceInputDimensionValueOnCreate,
                                  kPreferenceOverdefinedDimensionsPrompt,
                                  kPreferenceOverdefinedDrivenByDefault}) {
        try {
            const bool old = call(application_, "GetUserPreferenceToggle",
                                  {com::Variant(preference)}, "read dimension preference").as_bool();
            saved_.emplace_back(preference, old);
            call(application_, "SetUserPreferenceToggle",
                 {com::Variant(preference), com::Variant(false)},
                 "disable dimension preference");
        } catch (...) {
        }
    }
}

DimensionFlags::~DimensionFlags()
{
    for (const auto& [preference, value_to_restore] : saved_) {
        try {
            call(application_, "SetUserPreferenceToggle",
                 {com::Variant(preference), com::Variant(value_to_restore)},
                 "restore dimension preference");
        } catch (...) {
        }
    }
}

void require_angle_precision(const com::Dispatch& model, long decimals)
{
    const auto ext = extension(model);
    const long current = call(ext, "GetUserPreferenceInteger",
                              {com::Variant(kUnitsAngularDecimals), com::Variant(0)},
                              "read angular precision").as_long();
    if (current < decimals) {
        call(ext, "SetUserPreferenceInteger",
             {com::Variant(kUnitsAngularDecimals), com::Variant(0), com::Variant(decimals)},
             "raise angular precision");
    }
}

void add_dimension(const com::Dispatch& model,
                   const std::vector<com::Dispatch>& entities,
                   core::Point3 position_m, double value_si,
                   std::string_view what, std::string_view name)
{
    select_entities(model, entities, what);
    const auto display = dispatch(model, "AddDimension2",
                                  {com::Variant(position_m.x), com::Variant(position_m.y),
                                   com::Variant(position_m.z)},
                                  std::string("dimension ") + std::string(what));
    clear_selection(model);
    const auto dimension = dispatch(display, "GetDimension", {},
                                    std::string("read back dimension for ") + std::string(what));
    if (number_value(dimension, "DrivenState", "read dimension driven state") !=
        static_cast<double>(kDimensionDriving)) {
        dimension.put("DrivenState", com::Variant(kDimensionDriving),
                      std::string("make dimension driving for ") + std::string(what));
        if (number_value(dimension, "DrivenState", "verify dimension driven state") !=
            static_cast<double>(kDimensionDriving)) {
            throw SwError(std::string(what) + " would not go driving");
        }
    }
    const double actual = number_value(dimension, "SystemValue", "read dimension value");
    if (std::abs(actual - value_si) > 1e-6) {
        std::ostringstream message;
        message << what << " came back as " << actual << " but the geometry is at "
                << value_si << " (SI units)";
        throw SwError(message.str());
    }
    dimension.put("SystemValue", com::Variant(value_si),
                  std::string("set dimension value for ") + std::string(what));
    if (!name.empty()) {
        try {
            dimension.put("Name", com::Variant(name), "name dimension");
        } catch (...) {
        }
    }
}

void require_fully_defined(const com::Dispatch& sketch, std::string_view what)
{
    const long status = value(sketch, "GetConstrainedStatus", what).as_long();
    if (status != kFullyConstrained) {
        throw SwError(std::string(what) + " is not fully defined (status " +
                      std::to_string(status) + ")");
    }
}

core::Point3 radial_position(double radius_mm, double z_mm) noexcept
{
    return {millimeters_to_meters(0.5 * radius_mm), 0.0,
            millimeters_to_meters(z_mm)};
}

core::Point3 axial_position(double reach_mm, double z_mm) noexcept
{
    return {millimeters_to_meters(-0.3 * reach_mm), 0.0,
            millimeters_to_meters(z_mm)};
}

core::Point2 model_to_sketch(const com::Dispatch& sketch, core::Point3 model_point)
{
    const auto transform = property_dispatch(sketch, "ModelToSketchTransform",
                                             "read ModelToSketchTransform");
    const auto data = value(transform, "ArrayData", "read sketch transform data").as_doubles();
    if (data.size() < 13) throw SwError("ModelToSketchTransform returned fewer than 13 values");
    const double scale = data[12] == 0.0 ? 1.0 : data[12];
    return {(data[0] * model_point.x + data[3] * model_point.y + data[6] * model_point.z) * scale + data[9],
            (data[1] * model_point.x + data[4] * model_point.y + data[7] * model_point.z) * scale + data[10]};
}

BlankSketch sketch_blank_outline(const com::Dispatch& model,
                                 const std::vector<core::Point2>& outline,
                                 double axis_overshoot_mm)
{
    const auto manager = sketch_manager(model);
    const double z_hi = std::max_element(
        outline.begin(), outline.end(), [](auto lhs, auto rhs) { return lhs.y < rhs.y; })->y;
    clear_selection(model);
    select_first(model, {"Top Plane", "Top", "Plane2"}, "PLANE");
    call(manager, "InsertSketch", {com::Variant(true)}, "open blank profile sketch");
    const auto active_sketch = property_dispatch(manager, "ActiveSketch", "read active sketch");

    std::vector<com::Dispatch> lines;
    com::Dispatch axis;
    {
        SketchFlags flags(manager);
        const auto first = model_to_sketch(active_sketch, {0.0, 0.0, 0.0});
        const auto second = model_to_sketch(active_sketch,
                                            {0.0, 0.0, millimeters_to_meters(z_hi + axis_overshoot_mm)});
        axis = dispatch(manager, "CreateCenterLine",
                        {com::Variant(first.x), com::Variant(first.y), com::Variant(0.0),
                         com::Variant(second.x), com::Variant(second.y), com::Variant(0.0)},
                        "blank centreline");
        for (std::size_t i = 0; i < outline.size(); ++i) {
            const auto& first_point = outline[i];
            const auto& second_point = outline[(i + 1) % outline.size()];
            const auto a = model_to_sketch(active_sketch,
                                           {millimeters_to_meters(first_point.x), 0.0,
                                            millimeters_to_meters(first_point.y)});
            const auto b = model_to_sketch(active_sketch,
                                           {millimeters_to_meters(second_point.x), 0.0,
                                            millimeters_to_meters(second_point.y)});
            lines.push_back(dispatch(manager, "CreateLine",
                                     {com::Variant(a.x), com::Variant(a.y), com::Variant(0.0),
                                      com::Variant(b.x), com::Variant(b.y), com::Variant(0.0)},
                                     "blank outline segment " + std::to_string(i)));
        }
    }
    return {manager, axis, std::move(lines)};
}

void constrain_blank(const com::Dispatch& model, const BlankSketch& sketch,
                     const std::vector<core::Point2>& outline)
{
    select_entities(model, {sketch.axis}, "fix the gear axis");
    call(model, "SketchAddConstraints", {com::Variant(kRelationFixed)},
         "fix the gear axis");
    for (std::size_t i = 0; i < sketch.lines.size(); ++i) {
        const auto& a = outline[i];
        const auto& b = outline[(i + 1) % outline.size()];
        if (std::abs(a.y - b.y) < kCoordinateToleranceMm) {
            select_entities(model, {sketch.lines[i]}, "add horizontal blank relation");
            call(model, "SketchAddConstraints", {com::Variant(kRelationHorizontal)},
                 "add horizontal blank relation");
            clear_selection(model);
        } else if (std::abs(a.x - b.x) < kCoordinateToleranceMm) {
            select_entities(model, {sketch.lines[i]}, "add vertical blank relation");
            call(model, "SketchAddConstraints", {com::Variant(kRelationVertical)},
                 "add vertical blank relation");
            clear_selection(model);
        }
    }
}

void link_blank_equations(const com::Dispatch& model, std::string_view sketch_name,
                          const std::vector<BlankDimension>& dimensions,
                          const std::vector<BlankVariable>& variables)
{
    require_angle_precision(model);
    const auto manager = dispatch(model, "GetEquationMgr", {}, "GetEquationMgr");
    for (const auto& variable : variables) {
        const auto result = call(manager, "Add2", {com::Variant(-1),
                                                     com::Variant(std::string("\"") + variable.name + "\" = " + variable.text),
                                                     com::Variant(true)},
                                  variable.what + " variable");
        if (result.as_long() < 0)
            throw SwError("SOLIDWORKS rejected equation for " + variable.what);
    }
    for (const auto& dimension : dimensions) {
        const std::string rhs = dimension.definition.has_value()
                                     ? *dimension.definition
                                     : equation_number(dimension.value) + dimension.unit;
        const std::string equation = std::string("\"") + dimension.name + "\" = " + rhs;
        const auto result = call(manager, "Add2", {com::Variant(-1), com::Variant(equation),
                                                      com::Variant(true)},
                                  dimension.what + " variable");
        if (result.as_long() < 0) throw SwError("SOLIDWORKS rejected equation for " + dimension.what);
    }
    for (const auto& dimension : dimensions) {
        const std::string full = dimension.name + "@" + std::string(sketch_name);
        const std::string equation = std::string("\"") + full + "\" = \"" + dimension.name + "\"";
        const auto result = call(manager, "Add2", {com::Variant(-1), com::Variant(equation),
                                                      com::Variant(true)},
                                  dimension.what + " link");
        if (result.as_long() < 0) throw SwError("SOLIDWORKS rejected equation link for " + dimension.what);
    }
    call(manager, "EvaluateAll", {}, "evaluate equations");
    for (const auto& dimension : dimensions) {
        const auto parameter = dispatch(model, "Parameter", {com::Variant(dimension.name + "@" + std::string(sketch_name))},
                                         "read linked dimension");
        const double actual = number_value(parameter, "SystemValue", "read linked dimension value");
        if (std::abs(actual - dimension.si) > 1e-6)
            throw SwError("linked dimension " + dimension.name + " moved the geometry");
    }
}

com::Dispatch close_and_revolve_blank(const com::Dispatch& model,
                                      const BlankSketch& sketch,
                                      const std::vector<BlankDimension>& dimensions,
                                      const std::vector<BlankVariable>& variables)
{
    const auto active = property_dispatch(sketch.manager, "ActiveSketch", "read blank sketch");
    require_fully_defined(active, "blank profile sketch");
    call(sketch.manager, "InsertSketch", {com::Variant(true)}, "close blank profile sketch");
    clear_selection(model);
    const auto blank_sketch = last_feature(model);
    const std::string sketch_name = string_value(blank_sketch, "Name", "read blank sketch name");
    link_blank_equations(model, sketch_name, dimensions, variables);
    select_feature(blank_sketch, "blank profile sketch");
    const auto manager = property_dispatch(model, "FeatureManager", "read FeatureManager");
    require_true(call(manager, "FeatureRevolve2",
                      {com::Variant(true), com::Variant(true), com::Variant(false),
                       com::Variant(false), com::Variant(false), com::Variant(false),
                       com::Variant(0), com::Variant(0), com::Variant(2.0 * core::kPi),
                       com::Variant(0.0), com::Variant(false), com::Variant(false),
                       com::Variant(0.0), com::Variant(0.0), com::Variant(0),
                       com::Variant(0.0), com::Variant(0.0), com::Variant(true),
                       com::Variant(true), com::Variant(true)},
                      "revolve the blank"),
                 "revolve the blank");
    return last_feature(model);
}

com::Dispatch draw_curves_3d(
    const com::Dispatch& model,
    const std::vector<std::pair<std::string, std::vector<core::Point3>>>& curves,
    std::string_view what)
{
    const auto manager = sketch_manager(model);
    call(manager, "Insert3DSketch", {com::Variant(true)}, std::string("open ") + std::string(what));
    {
        SketchFlags flags(manager);
        for (const auto& [name, points] : curves) {
            if (points.size() < 2) continue;
            if (points.size() == 2) {
                const auto& a = points[0];
                const auto& b = points[1];
                require_true(call(manager, "CreateLine",
                                  {com::Variant(millimeters_to_meters(a.x)),
                                   com::Variant(millimeters_to_meters(a.y)),
                                   com::Variant(millimeters_to_meters(a.z)),
                                   com::Variant(millimeters_to_meters(b.x)),
                                   com::Variant(millimeters_to_meters(b.y)),
                                   com::Variant(millimeters_to_meters(b.z))},
                                  std::string(what) + " line " + name),
                             std::string(what) + " line " + name);
            } else {
                std::vector<double> flat;
                flat.reserve(points.size() * 3);
                for (const auto& point : points) {
                    flat.push_back(millimeters_to_meters(point.x));
                    flat.push_back(millimeters_to_meters(point.y));
                    flat.push_back(millimeters_to_meters(point.z));
                }
                const auto result = call(manager, "CreateSpline",
                                         {com::Variant::doubles(flat)},
                                         std::string(what) + " spline " + name);
                if (result.is_nullish()) throw SwError(std::string(what) + " spline " + name + " was rejected");
            }
        }
    }
    call(manager, "Insert3DSketch", {com::Variant(true)}, std::string("close ") + std::string(what));
    clear_selection(model);
    return last_feature(model);
}

com::Dispatch loft_cut(const com::Dispatch& model,
                       const std::vector<com::Dispatch>& profiles,
                       const std::vector<com::Dispatch>& guides,
                       long guide_mark)
{
    clear_selection(model);
    for (std::size_t i = 0; i < profiles.size(); ++i)
        select_feature(profiles[i], "loft profile " + std::to_string(i), i != 0, kMarkLoftProfile);
    for (std::size_t i = 0; i < guides.size(); ++i)
        select_feature(guides[i], "loft guide " + std::to_string(i), true, guide_mark);
    const auto manager = property_dispatch(model, "FeatureManager", "read FeatureManager");
    require_true(call(manager, "InsertCutBlend",
                      {com::Variant(false), com::Variant(false), com::Variant(false),
                       com::Variant(1.0), com::Variant(0), com::Variant(0),
                       com::Variant(false), com::Variant(0.0), com::Variant(0.0),
                       com::Variant(0), com::Variant(true), com::Variant(true)},
                      "loft cut the tooth space"),
                 "loft cut the tooth space");
    return last_feature(model);
}

void drop_offcut(const com::Dispatch& model)
{
    const auto bodies = dispatch_array(call(model, "GetBodies2",
                                             {com::Variant(kSolidBody), com::Variant(true)},
                                             "read solid bodies"),
                                       "read solid bodies");
    if (bodies.size() <= 1) return;
    auto keep = bodies.front();
    double largest = -std::numeric_limits<double>::infinity();
    for (const auto& body : bodies) {
        const auto mass = call(body, "GetMassProperties", {com::Variant(1.0)},
                               "measure off-cut body").as_doubles();
        if (mass.size() > 3 && mass[3] > largest) {
            largest = mass[3];
            keep = body;
        }
    }
    clear_selection(model);
    require_true(call(keep, "Select2", {com::Variant(false), com::Variant()},
                      "select gear body to keep"),
                 "select gear body to keep");
    const auto manager = property_dispatch(model, "FeatureManager", "read FeatureManager");
    require_true(call(manager, "InsertDeleteBody2", {com::Variant(true)},
                      "discard off-cut bodies"),
                 "discard off-cut bodies");
}

void pattern_teeth(const com::Dispatch& model, const com::Dispatch& cut,
                   const com::Dispatch& axis, int teeth)
{
    clear_selection(model);
    select_feature(cut, "loft cut", false, kMarkPatternFeature);
    select_feature(axis, "gear axis", true, kMarkPatternAxis);
    const auto manager = property_dispatch(model, "FeatureManager", "read FeatureManager");
    require_true(call(manager, "FeatureCircularPattern5",
                      {com::Variant(teeth), com::Variant(2.0 * core::kPi),
                       com::Variant(false), com::Variant("NULL"), com::Variant(true),
                       com::Variant(true), com::Variant(false), com::Variant(false),
                       com::Variant(false), com::Variant(false), com::Variant(1),
                       com::Variant(0.0), com::Variant("NULL"), com::Variant(false)},
                      "circular pattern of " + std::to_string(teeth) + " teeth"),
                 "circular pattern of " + std::to_string(teeth) + " teeth");
}

PartArtifact measure_part(const Session& session, const com::Dispatch& model,
                          std::string member, int teeth, double expected_radius_mm,
                          const std::filesystem::path& save_path)
{
    call(model, "EditRebuild3", {}, "rebuild part");
    try { call(model, "ViewZoomtofit2", {}, "fit part view"); } catch (...) {}
    const auto bodies = dispatch_array(call(model, "GetBodies2",
                                             {com::Variant(kSolidBody), com::Variant(true)},
                                             "read rebuilt solid bodies"),
                                       "read rebuilt solid bodies");
    if (bodies.size() != 1)
        throw SwError("expected exactly one solid body, got " + std::to_string(bodies.size()));
    const auto box = call(bodies.front(), "GetBodyBox", {}, "measure part bounding box").as_doubles();
    if (box.size() < 6) throw SwError("GetBodyBox returned fewer than six coordinates");
    std::array<double, 6> box_mm{};
    std::copy_n(box.begin(), 6, box_mm.begin());
    for (auto& value_mm : box_mm) value_mm *= 1000.0;
    const auto faces = dispatch_array(call(bodies.front(), "GetFaces", {}, "read part faces"),
                                       "read part faces");
    const double max_radius = std::max({std::abs(box_mm[0]), std::abs(box_mm[1]),
                                        std::abs(box_mm[3]), std::abs(box_mm[4])});
    if (!save_path.empty()) session.save(model, save_path);
    const std::string title = string_value(model, "GetTitle", "read part title");
    PartResult report;
    report.member = std::move(member);
    report.title = title;
    report.teeth = teeth;
    report.body_count = static_cast<int>(bodies.size());
    report.face_count = static_cast<int>(faces.size());
    report.box_mm = box_mm;
    report.max_radius_mm = max_radius;
    report.expected_radius_mm = expected_radius_mm;
    report.radius_error_pct = expected_radius_mm == 0.0
                                  ? 0.0
                                  : 100.0 * (max_radius - expected_radius_mm) / expected_radius_mm;
    report.path = save_path;
    return {std::move(report), model};
}

} // namespace geargen::solidworks::detail

#endif
