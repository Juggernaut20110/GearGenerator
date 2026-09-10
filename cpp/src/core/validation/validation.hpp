#pragma once

#include "core/common/parameters.hpp"

#include <string>
#include <vector>

namespace geargen::core {

struct Issue {
    std::string field;
    std::string message;
};

class ValidationResult {
public:
    [[nodiscard]] bool ok() const noexcept { return errors.empty(); }
    [[nodiscard]] bool has_warnings() const noexcept { return !warnings.empty(); }

    void error(std::string field, std::string message);
    void warning(std::string field, std::string message);
    void advisory(std::string field, std::string message);

    std::vector<Issue> errors;
    std::vector<Issue> warnings;
    // Advisory/informational conditions do not affect buildability.  Python's
    // current UI calls these out separately from warnings; keeping a third
    // channel prevents a future native UI from collapsing that distinction.
    std::vector<Issue> advisories;
};

// Shared sanity rails. Type-specific validators will append their own
// geometry-dependent errors after parameter validation succeeds.
void validate_pair_basics(double module_mm, int z1, int z2,
                          double pressure_angle_deg,
                          ValidationResult& result);

[[nodiscard]] ValidationResult validate(const SpurSetParams& parameters);
[[nodiscard]] ValidationResult validate(const BevelSetParams& parameters);
[[nodiscard]] ValidationResult validate(const HypoidSetParams& parameters);
[[nodiscard]] ValidationResult validate(const PlanetarySetParams& parameters);

} // namespace geargen::core
