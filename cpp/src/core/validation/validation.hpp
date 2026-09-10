#pragma once

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

    void error(std::string field, std::string message);
    void warning(std::string field, std::string message);

    std::vector<Issue> errors;
    std::vector<Issue> warnings;
};

// Shared sanity rails. Type-specific validators will append their own
// geometry-dependent errors after parameter validation succeeds.
void validate_pair_basics(double module_mm, int z1, int z2,
                          double pressure_angle_deg,
                          ValidationResult& result);

} // namespace geargen::core
