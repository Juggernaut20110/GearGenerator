#include "solidworks/solidworks.hpp"

namespace geargen::solidworks {

double millimeters_to_meters(double millimeters) noexcept
{
    return millimeters * 0.001;
}

} // namespace geargen::solidworks
