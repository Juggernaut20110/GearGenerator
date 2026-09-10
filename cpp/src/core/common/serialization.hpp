#pragma once

#include "core/common/parameters.hpp"
#include "core/validation/validation.hpp"

#include <optional>
#include <filesystem>
#include <string>

namespace geargen::core {

// Output-only JSON helpers keep the core independent of Qt and of a third
// party JSON package at this stage.  The Python preset contract is intentionally
// tolerant: unknown keys are ignored and type-specific migration hooks run at
// the file boundary. Native file loading will use the same policy when the GUI
// port reaches presets; reference snapshots only need deterministic emission.
[[nodiscard]] std::string json_number(double value);
[[nodiscard]] std::string json_integer(int value);
[[nodiscard]] std::string json_bool(bool value);
[[nodiscard]] std::string json_string(const std::string& value);
[[nodiscard]] std::string json_optional_number(
    const std::optional<double>& value);

[[nodiscard]] std::string parameters_json(const SpurSetParams& parameters);
[[nodiscard]] std::string parameters_json(const BevelSetParams& parameters);
[[nodiscard]] std::string parameters_json(const HypoidSetParams& parameters);
[[nodiscard]] std::string parameters_json(const PlanetarySetParams& parameters);
[[nodiscard]] std::string validation_json(const ValidationResult& result);

// Python presets are flat JSON objects produced from a dataclass. The native
// loader accepts that format, ignores unknown keys, and applies the same
// size-dependent defaults before known overrides. A small dependency-free
// parser keeps this contract available to the core and to non-Qt tools.
[[nodiscard]] SpurSetParams load_spur_preset(const std::filesystem::path& path);
[[nodiscard]] BevelSetParams load_bevel_preset(const std::filesystem::path& path);
[[nodiscard]] HypoidSetParams load_hypoid_preset(const std::filesystem::path& path);
[[nodiscard]] PlanetarySetParams load_planetary_preset(const std::filesystem::path& path);
void save_preset(const std::filesystem::path& path, const SpurSetParams& parameters);
void save_preset(const std::filesystem::path& path, const BevelSetParams& parameters);
void save_preset(const std::filesystem::path& path, const HypoidSetParams& parameters);
void save_preset(const std::filesystem::path& path, const PlanetarySetParams& parameters);

} // namespace geargen::core
