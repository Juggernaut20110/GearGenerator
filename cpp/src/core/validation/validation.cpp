#include "core/validation/validation.hpp"

#include <cmath>
#include <utility>

namespace geargen::core {

void ValidationResult::error(std::string field, std::string message)
{
    errors.push_back({std::move(field), std::move(message)});
}

void ValidationResult::warning(std::string field, std::string message)
{
    warnings.push_back({std::move(field), std::move(message)});
}

void validate_pair_basics(double module_mm, int z1, int z2,
                          double pressure_angle_deg,
                          ValidationResult& result)
{
    if (!std::isfinite(module_mm) || module_mm <= 0.0) {
        result.error("module", "must be greater than zero");
    }
    if (z1 < 6) {
        result.error("z1", "must be at least 6 teeth");
    }
    if (z2 < 6) {
        result.error("z2", "must be at least 6 teeth");
    }
    if (!std::isfinite(pressure_angle_deg) ||
        pressure_angle_deg < 14.5 || pressure_angle_deg > 25.0) {
        result.error("pressure_angle", "must be between 14.5 and 25 degrees");
    }
}

} // namespace geargen::core
