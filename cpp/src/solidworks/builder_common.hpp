#pragma once

#include "core/common/geometry_types.hpp"
#include "solidworks/solidworks.hpp"

#include <optional>
#include <string>
#include <vector>

namespace geargen::solidworks::detail {

struct BlankDimension {
    std::string name;
    double value{};
    std::string unit;
    double si{};
    std::string what;
    std::optional<std::string> definition;
};

struct BlankVariable {
    std::string name;
    std::string text;
    std::string what;
};

struct BlankSketch {
    com::Dispatch manager;
    com::Dispatch axis;
    std::vector<com::Dispatch> lines;
};

struct PartArtifact {
    PartResult report;
    com::Dispatch model;
};

[[nodiscard]] com::Variant call(const com::Dispatch& object, std::string_view member,
                                const std::vector<com::Variant>& arguments,
                                std::string_view what);
[[nodiscard]] com::Variant value(const com::Dispatch& object, std::string_view member,
                                 std::string_view what);
[[nodiscard]] com::Dispatch dispatch(const com::Dispatch& object, std::string_view member,
                                     const std::vector<com::Variant>& arguments,
                                     std::string_view what);
[[nodiscard]] com::Dispatch property_dispatch(const com::Dispatch& object,
                                              std::string_view member,
                                              std::string_view what);
[[nodiscard]] com::Dispatch require_dispatch(const com::Variant& result,
                                              std::string_view what);
[[nodiscard]] std::vector<com::Dispatch> dispatches(const com::Variant& result,
                                                    std::string_view what);
void require_true(const com::Variant& result, std::string_view what);

[[nodiscard]] std::string string_value(const com::Dispatch& object,
                                       std::string_view member,
                                       std::string_view what);
[[nodiscard]] double number_value(const com::Dispatch& object,
                                  std::string_view member,
                                  std::string_view what);
[[nodiscard]] bool bool_value(const com::Dispatch& object, std::string_view member,
                              std::string_view what);
[[nodiscard]] std::string equation_number(double value);

void select(const com::Dispatch& model, std::string_view name,
            std::string_view type, bool append = false, long mark = 0);
[[nodiscard]] std::string select_first(const com::Dispatch& model,
                                       const std::vector<std::string>& names,
                                       std::string_view type, bool append = false,
                                       long mark = 0);
void select_entities(const com::Dispatch& model,
                     const std::vector<com::Dispatch>& entities,
                     std::string_view what);
void select_feature(const com::Dispatch& feature, std::string_view what,
                    bool append = false, long mark = 0);

[[nodiscard]] com::Dispatch last_feature(const com::Dispatch& model);
[[nodiscard]] com::Dispatch endpoint(const com::Dispatch& line, bool start);
[[nodiscard]] com::Dispatch axis_datum(const com::Dispatch& axis);
[[nodiscard]] com::Dispatch create_axis(const com::Dispatch& model);

class DimensionFlags {
public:
    explicit DimensionFlags(const com::Dispatch& application);
    DimensionFlags(const DimensionFlags&) = delete;
    DimensionFlags& operator=(const DimensionFlags&) = delete;
    ~DimensionFlags();

private:
    com::Dispatch application_;
    std::vector<std::pair<long, bool>> saved_;
};

void require_angle_precision(const com::Dispatch& model,
                             long decimals = kEquationAngleDecimals);
void add_dimension(const com::Dispatch& model,
                   const std::vector<com::Dispatch>& entities,
                   core::Point3 position_m, double value_si,
                   std::string_view what, std::string_view name = {});
void require_fully_defined(const com::Dispatch& sketch, std::string_view what);

[[nodiscard]] core::Point3 radial_position(double radius_mm, double z_mm) noexcept;
[[nodiscard]] core::Point3 axial_position(double reach_mm, double z_mm) noexcept;
[[nodiscard]] core::Point2 model_to_sketch(const com::Dispatch& sketch,
                                           core::Point3 model_point);

[[nodiscard]] BlankSketch sketch_blank_outline(
    const com::Dispatch& model, const std::vector<core::Point2>& outline,
    double axis_overshoot_mm = 5.0);
void constrain_blank(const com::Dispatch& model, const BlankSketch& sketch,
                     const std::vector<core::Point2>& outline);

void link_blank_equations(const com::Dispatch& model, std::string_view sketch_name,
                          const std::vector<BlankDimension>& dimensions,
                          const std::vector<BlankVariable>& variables = {});
[[nodiscard]] com::Dispatch close_and_revolve_blank(
    const com::Dispatch& model, const BlankSketch& sketch,
    const std::vector<BlankDimension>& dimensions,
    const std::vector<BlankVariable>& variables = {});

[[nodiscard]] com::Dispatch draw_curves_3d(
    const com::Dispatch& model,
    const std::vector<std::pair<std::string, std::vector<core::Point3>>>& curves,
    std::string_view what);
[[nodiscard]] com::Dispatch loft_cut(const com::Dispatch& model,
                                     const std::vector<com::Dispatch>& profiles,
                                     const std::vector<com::Dispatch>& guides = {},
                                     long guide_mark = 0);
void drop_offcut(const com::Dispatch& model);
void pattern_teeth(const com::Dispatch& model, const com::Dispatch& cut,
                   const com::Dispatch& axis, int teeth);

[[nodiscard]] PartArtifact measure_part(const Session& session,
                                        const com::Dispatch& model,
                                        std::string member, int teeth,
                                        double expected_radius_mm,
                                        const std::filesystem::path& save_path);

} // namespace geargen::solidworks::detail
