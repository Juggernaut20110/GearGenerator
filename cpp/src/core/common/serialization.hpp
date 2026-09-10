#pragma once

#include "core/common/parameters.hpp"
#include "core/validation/validation.hpp"

#include <optional>
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

} // namespace geargen::core
