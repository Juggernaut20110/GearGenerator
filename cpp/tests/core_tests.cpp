#include "core/bevel/bevel.hpp"
#include "core/common/geometry_types.hpp"
#include "core/hypoid/hypoid.hpp"
#include "core/involute/involute.hpp"
#include "core/placement/placement.hpp"
#include "core/planetary/planetary.hpp"
#include "core/spur/spur.hpp"
#include "core/validation/validation.hpp"
#include "preview/export/export.hpp"
#include "preview/scene2d/scene.hpp"
#include "solidworks/solidworks.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

} // namespace

int main()
{
    using namespace geargen::core;
    using namespace geargen::core::placement;

    bool ok = true;
    const auto spur = spur::default_parameters(2.0, 17, 43);
    ok &= check(spur.face_width_mm > 0.0 && spur.bore_mm > 0.0,
                "spur defaults are positive");
    const auto planetary = planetary::default_parameters(2.0, 24, 18);
    ok &= check(planetary.z_ring() == 60, "planetary ring count is derived");
    const auto bevel = bevel::default_parameters(2.0, 17, 43);
    ok &= check(bevel.face_width_mm > 0.0, "bevel defaults are positive");
    const auto hypoid = hypoid::default_parameters(170.0 / 42.0, 13, 42);
    ok &= check(hypoid.face_width_mm > 0.0, "hypoid defaults are positive");

    ValidationResult validation;
    validate_pair_basics(2.0, 17, 43, 20.0, validation);
    ok &= check(validation.ok(), "valid anchor parameters pass basic validation");
    ValidationResult invalid;
    validate_pair_basics(-1.0, 3, 43, 30.0, invalid);
    ok &= check(!invalid.ok() && invalid.errors.size() == 3,
                "basic validation accumulates independent errors");

    ok &= check(std::abs(gear_clocking(17)) < 1e-12,
                "odd-tooth gear needs no clocking");
    ok &= check(std::abs(gear_clocking(60) - kPi / 60.0) < 1e-12,
                "even-tooth gear receives half-pitch clocking");
    ok &= check(angular_velocity_ratio(17, 43) < 0.0 &&
                    angular_velocity_ratio(17, 43, true) > 0.0,
                "external and internal mesh senses differ");

    const auto packed = to_solidworks_array(rot_z(kPi / 2.0));
    ok &= check(std::abs(packed[0]) < 1e-12 && std::abs(packed[1] - 1.0) < 1e-12,
                "SOLIDWORKS transform packing is column-major");
    ok &= check(std::abs(involute::involute_function(0.0)) < 1e-12,
                "involute primitive is available in core");

    geargen::preview::Scene2D scene;
    scene.polylines.push_back({{{0.0, 1.0}, {2.0, 3.0}}, "test", false});
    const auto bounds = scene.bounds();
    ok &= check(bounds[0] == 0.0 && bounds[3] == 3.0,
                "2D preview scene computes bounds");
    ok &= check(geargen::preview::exporter::dxf_lines(scene).size() > 4,
                "DXF export has a complete section");
    ok &= check(std::abs(geargen::solidworks::millimeters_to_meters(2.0) - 0.002) < 1e-12,
                "SOLIDWORKS unit boundary converts millimetres");

    return ok ? 0 : 1;
}
