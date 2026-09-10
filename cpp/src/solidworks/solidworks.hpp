#pragma once

#include "core/common/parameters.hpp"
#include "solidworks/com.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace geargen::solidworks {

inline constexpr const char* kAxisFeatureName = "GearAxis";
inline constexpr double kCoordinateToleranceMm = 1e-9;
inline constexpr double kStraightTwistToleranceRad = 1e-9;

// Values are copied from swconst.tlb in the reference project. Keeping the
// subset here avoids requiring a SOLIDWORKS SDK/type-library installation just
// to compile the native application.
enum class UserPreferenceString : long {
    DefaultTemplatePart = 8,
    DefaultTemplateAssembly = 9,
};

enum class BodyType : long { Solid = 0 };
enum class AddComponentConfigOptions : long { CurrentConfiguration = 0 };
enum class EndCondition : long { Blind = 0 };
enum class SaveAsVersion : long { Current = 0 };
enum class SaveAsOptions : long { Silent = 1 };

enum class MateType : long {
    Coincident = 0,
    Parallel = 3,
    Distance = 5,
    Angle = 6,
    Gear = 10,
};

inline constexpr long kMateAlignClosest = 2;
inline constexpr long kAddMateNoError = 1;
inline constexpr long kDimensionDriven = 1;
inline constexpr long kDimensionDriving = 2;
inline constexpr long kFullyConstrained = 3;
inline constexpr long kUnitsAngularDecimals = 52;
inline constexpr long kEquationAngleDecimals = 6;

inline constexpr long kPreferenceInputDimensionValueOnCreate = 10;
inline constexpr long kPreferenceOverdefinedDimensionsPrompt = 100;
inline constexpr long kPreferenceOverdefinedDrivenByDefault = 101;

inline constexpr const char* kRelationFixed = "sgFIXED";
inline constexpr const char* kRelationHorizontal = "sgHORIZONTAL";
inline constexpr const char* kRelationVertical = "sgVERTICAL";
inline constexpr const char* kRelationCoincident = "sgCOINCIDENT";

inline constexpr long kMarkLoftProfile = 1;
inline constexpr long kMarkLoftGuide = 2;
inline constexpr long kMarkPatternAxis = 1;
inline constexpr long kMarkPatternFeature = 4;
inline constexpr long kMarkMateEntity = 1;

struct SwError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct BuildOptions {
    bool visible{true};
    bool silent{true};
    bool save_assembly{true};
    bool mate{true};
    bool reverse_gear_mate{false};
};

using Parameters = std::variant<core::BevelSetParams, core::SpurSetParams,
                                 core::HypoidSetParams, core::PlanetarySetParams>;

struct BuildRequest {
    Parameters parameters;
    std::filesystem::path output_directory;
    BuildOptions options;
};

struct PartResult {
    std::string member;
    std::string title;
    int teeth{};
    int body_count{};
    int face_count{};
    std::array<double, 6> box_mm{};
    double max_radius_mm{};
    double expected_radius_mm{};
    double radius_error_pct{};
    std::filesystem::path path;
};

struct BuildResult {
    bool ok{false};
    std::string message;
    std::string gear_kind;
    std::filesystem::path assembly_path;
    std::vector<PartResult> parts;
    std::vector<std::string> mates;
    std::vector<std::pair<double, double>> gear_ratios;
    int interference_count{-1};
    double interference_volume_mm3{};
};

class Session {
public:
    explicit Session(bool visible = true, bool silent = true);
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) noexcept;
    Session& operator=(Session&&) noexcept;
    ~Session();

    void initialize();
    void close() noexcept;
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }
    [[nodiscard]] bool visible() const noexcept { return visible_; }
    [[nodiscard]] bool silent() const noexcept { return silent_; }

    [[nodiscard]] com::Dispatch new_part() const;
    [[nodiscard]] com::Dispatch new_assembly() const;
    [[nodiscard]] com::Dispatch application() const;
    [[nodiscard]] com::Dispatch math_utility() const;
    void save(const com::Dispatch& model, const std::filesystem::path& path) const;
    void close_document(const com::Dispatch& model,
                        const std::filesystem::path* save_as = nullptr) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool visible_{true};
    bool silent_{true};
    bool initialized_{false};
};

class Backend {
public:
    virtual ~Backend() = default;
    [[nodiscard]] virtual BuildResult build(const BuildRequest& request) = 0;
};

class NativeBackend final : public Backend {
public:
    [[nodiscard]] BuildResult build(const BuildRequest& request) override;
};

[[nodiscard]] BuildResult build(const BuildRequest& request);
[[nodiscard]] double millimeters_to_meters(double millimeters) noexcept;

} // namespace geargen::solidworks
