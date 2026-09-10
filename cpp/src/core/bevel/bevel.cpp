#include "core/bevel/bevel.hpp"

#include "core/common/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace geargen::core::bevel {
namespace {

constexpr double kWorkingDepthFactor = 2.0;
constexpr double kWholeDepthFactor = 2.188;
constexpr double kClearanceFactor = 0.188;
constexpr double kEndOvershootFraction = 0.05;
constexpr double kEndOvershootMinMm = 0.5;

double clamp_unit(double value) noexcept { return std::clamp(value, -1.0, 1.0); }

const MemberGeometry& get_member(const SetGeometry& geo, const std::string& name)
{
    if (name == "pinion") return geo.pinion;
    if (name == "gear") return geo.gear;
    throw std::invalid_argument("bevel member must be 'pinion' or 'gear'");
}

double front_face_z(const SetGeometry& geo, const MemberGeometry& m)
{
    return geo.inner_cone_dist_mm / std::cos(m.pitch_angle_rad) -
           m.virtual_tip_r_inner_mm * std::sin(m.pitch_angle_rad);
}

double beyond_back_cone(const SetGeometry& geo, const MemberGeometry& m,
                        Point3 point)
{
    const double sd = std::sin(m.pitch_angle_rad);
    const double cd = std::cos(m.pitch_angle_rad);
    return (std::hypot(point.x, point.y) - geo.outer_cone_dist_mm * sd) * sd +
           (point.z - geo.outer_cone_dist_mm * cd) * cd;
}

Point3 swept_tip_point(const SetGeometry& geo, const std::string& member,
                       double cone_dist, double radius)
{
    const double phase = phase_at_cone_distance(geo, member, cone_dist);
    return {radius * std::cos(phase), radius * std::sin(phase), cone_dist};
}

double chord_deviation(const SetGeometry& geo, const std::string& member,
                       double a, double b, int samples = 25)
{
    const double radius = get_member(geo, member).outside_dia_mm / 2.0;
    const Point3 pa = swept_tip_point(geo, member, a, radius);
    const Point3 pb = swept_tip_point(geo, member, b, radius);
    double worst = 0.0;
    for (int i = 1; i < samples; ++i) {
        const double f = static_cast<double>(i) / samples;
        const Point3 chord{pa.x + f * (pb.x - pa.x),
                           pa.y + f * (pb.y - pa.y),
                           pa.z + f * (pb.z - pa.z)};
        const Point3 truth = swept_tip_point(geo, member, a + (b - a) * f,
                                             radius);
        worst = std::max(worst, distance(chord, truth));
    }
    return worst;
}

double worst_sagitta(const SetGeometry& geo, const std::string& member,
                     const std::vector<double>& distances)
{
    double worst = 0.0;
    for (std::size_t i = 0; i + 1 < distances.size(); ++i) {
        const double a = distances[i];
        const double b = distances[i + 1];
        if (std::max(a, b) < geo.inner_cone_dist_mm ||
            std::min(a, b) > geo.outer_cone_dist_mm) continue;
        worst = std::max(worst, chord_deviation(geo, member, a, b));
    }
    return worst;
}

} // namespace

CrownTrace CrownTrace::for_set(double psi_m_rad, double cutter_radius,
                               double mean_cone_dist, double trace_sign)
{
    const double rho_sq = mean_cone_dist * mean_cone_dist +
                          cutter_radius * cutter_radius -
                          2.0 * mean_cone_dist * cutter_radius *
                              std::sin(std::abs(psi_m_rad));
    return {cutter_radius, std::sqrt(std::max(0.0, rho_sq)), mean_cone_dist,
            trace_sign};
}

double CrownTrace::spiral_angle_at(double cone_dist) const
{
    const double value = (cone_dist * cone_dist + cutter_radius_mm * cutter_radius_mm -
                          centre_distance_mm * centre_distance_mm) /
                         (2.0 * cone_dist * cutter_radius_mm);
    return sign * std::asin(clamp_unit(value));
}

double CrownTrace::theta_raw(double cone_dist) const
{
    if (centre_distance_mm <= 0.0 || cone_dist <= 0.0) return 0.0;
    const double value = (centre_distance_mm * centre_distance_mm +
                          cone_dist * cone_dist - cutter_radius_mm * cutter_radius_mm) /
                         (2.0 * centre_distance_mm * cone_dist);
    return std::acos(clamp_unit(value));
}

double CrownTrace::theta_at(double cone_dist) const
{
    return theta_raw(cone_dist) - theta_raw(mean_cone_dist_mm);
}

bool CrownTrace::reaches(double inner, double outer) const noexcept
{
    const double lo = std::abs(centre_distance_mm - cutter_radius_mm);
    const double hi = centre_distance_mm + cutter_radius_mm;
    return lo <= inner && outer <= hi;
}

double MemberGeometry::pitch_angle_deg() const noexcept
{
    return radians_to_degrees(pitch_angle_rad);
}

double MemberGeometry::face_angle_deg() const noexcept
{
    return radians_to_degrees(face_angle_rad);
}

double MemberGeometry::root_angle_deg() const noexcept
{
    return radians_to_degrees(root_angle_rad);
}

double MemberGeometry::angular_pitch_rad() const noexcept
{
    return kTau / static_cast<double>(z);
}

const MemberGeometry& SetGeometry::member(const std::string& which) const
{
    return get_member(*this, which);
}

double SetGeometry::face_contact_ratio() const noexcept
{
    if (!trace.has_value() || outer_cone_dist_mm == 0.0 ||
        circular_pitch_mm == 0.0) return 0.0;
    const double beta = trace->spiral_angle_at(mean_cone_dist_mm);
    return std::abs(parameters.face_width * std::tan(beta) /
                    (circular_pitch_mm * mean_cone_dist_mm / outer_cone_dist_mm));
}

BevelSetParams default_parameters(double module_mm, int z1, int z2, bool zerol)
{
    return BevelSetParams::with_defaults(module_mm, z1, z2, zerol);
}

SetGeometry derive(const BevelSetParams& p)
{
    const double delta1 = std::atan2(std::sin(p.sigma()),
                                     p.ratio() + std::cos(p.sigma()));
    const double delta2 = p.sigma() - delta1;
    const double d1 = p.module * p.z1;
    const double d2 = p.module * p.z2;
    const double outer = d1 / (2.0 * std::sin(delta1));
    const double inner = outer - p.face_width;
    const double mean = outer - p.face_width / 2.0;
    const double working = kWorkingDepthFactor * p.module;
    const double whole = kWholeDepthFactor * p.module;
    const double clearance = kClearanceFactor * p.module;
    const double equiv_ratio = (p.z2 * std::cos(delta1)) /
                               (p.z1 * std::cos(delta2));
    const double a2 = 0.54 * p.module + 0.46 * p.module / equiv_ratio;
    const double a1 = working - a2;
    const double bf1 = whole - a1;
    const double bf2 = whole - a2;
    const double theta_f1 = std::atan(bf1 / outer);
    const double theta_f2 = std::atan(bf2 / outer);
    const double theta_a1 = theta_f2;
    const double theta_a2 = theta_f1;
    const auto build_member = [&](const char* name, int z, double d,
                                  double delta, double addendum,
                                  double dedendum, double theta_a,
                                  double theta_f, double mate_d) {
        const double sd = std::sin(delta);
        const double cd = std::cos(delta);
        const double pitch_r = d / 2.0 / cd;
        const double pitch_R = outer * sd;
        const double pitch_z = outer * cd;
        const double tip_R = pitch_R + addendum * cd;
        const double tip_z = pitch_z - addendum * sd;
        const double root_R = pitch_R - dedendum * cd;
        const double root_z = pitch_z + dedendum * sd;
        const double tip_inner = tip_radius_at_cone_distance(
            inner, delta, delta + theta_a, tip_R, tip_z);
        return MemberGeometry{
            name, z, delta, d, addendum, dedendum, theta_a, theta_f,
            delta + theta_a, delta - theta_f,
            static_cast<double>(z) / cd, pitch_r, pitch_r * std::cos(p.alpha()),
            pitch_r + addendum, pitch_r - dedendum, tip_inner,
            d + 2.0 * addendum * cd, tip_z,
            mate_d / 2.0 - addendum * sd, root_R, root_z};
    };
    std::optional<CrownTrace> trace;
    if (p.is_curved()) {
        const double cutter = p.cutter_radius.value_or(mean);
        trace = CrownTrace::for_set(p.psi_m(), cutter, mean, p.trace_sign());
    }
    return {p, outer, mean, inner, inner / outer, working, whole, clearance,
            kPi * p.module,
            build_member("pinion", p.z1, d1, delta1, a1, bf1, theta_a1,
                         theta_f1, d2),
            build_member("gear", p.z2, d2, delta2, a2, bf2, theta_a2,
                         theta_f2, d1),
            trace};
}

double phase_at_cone_distance(const SetGeometry& geo, const std::string& name,
                              double cone_dist)
{
    if (!geo.trace.has_value()) return 0.0;
    const auto& m = geo.member(name);
    const double s = std::sin(m.pitch_angle_rad);
    if (std::abs(s) < 1e-12) return 0.0;
    const double sense = name == "gear" ? -1.0 : 1.0;
    return sense * geo.trace->sign * geo.trace->theta_at(cone_dist) / s;
}

double tip_radius_at_cone_distance(double cone_dist, double pitch_angle,
                                   double face_angle, double crown_radius,
                                   double crown_to_apex)
{
    const double sp = std::sin(pitch_angle);
    const double cp = std::cos(pitch_angle);
    const double sf = std::sin(face_angle);
    const double cf = std::cos(face_angle);
    const double det = sf * (-sp) - cp * cf;
    if (std::abs(det) < 1e-12)
        throw std::domain_error("bevel tip cone intersection is singular");
    const double section_apex_z = cone_dist / cp;
    const double rhs1 = crown_radius;
    const double rhs2 = crown_to_apex - section_apex_z;
    return (sf * rhs2 - rhs1 * cf) / det;
}

Point3 to_cone_3d(double x, double y, double delta, double cone_apex_z,
                 double phase) noexcept
{
    const double r = std::hypot(x, y);
    const double phi = std::atan2(y, x);
    const double R = r * std::cos(delta);
    const double theta = phi / std::cos(delta) + phase;
    return {R * std::cos(theta), R * std::sin(theta),
            cone_apex_z - r * std::sin(delta)};
}

ToothSpaceSection tooth_space_section(const SetGeometry& geo,
                                      const std::string& name,
                                      const std::string& requested_end,
                                      int flank_points, double overshoot,
                                      std::optional<double> requested_cone,
                                      bool split_cap)
{
    std::string end = requested_end;
    double cone_dist = requested_cone.value_or(0.0);
    if (!requested_cone.has_value()) {
        if (end == "outer") cone_dist = geo.outer_cone_dist_mm + overshoot;
        else if (end == "inner") cone_dist = std::max(
            geo.inner_cone_dist_mm - overshoot, 0.05 * geo.outer_cone_dist_mm);
        else throw std::invalid_argument("bevel section end must be outer or inner");
    } else {
        cone_dist = std::max(cone_dist, 0.05 * geo.outer_cone_dist_mm);
        end = "mid";
    }
    const auto& p = geo.parameters;
    const auto& m = geo.member(name);
    const double scale = cone_dist / geo.outer_cone_dist_mm;
    const double base = scale * m.virtual_base_r_mm;
    const double root = scale * m.virtual_root_r_mm;
    const double tooth_thickness = kPi * p.module / 2.0 - p.backlash / 2.0;
    const double psi0 = tooth_thickness / (2.0 * m.virtual_pitch_r_mm) +
                        involute::inv(p.alpha());
    const double half_pitch = kPi / m.virtual_teeth;
    double tip = tip_radius_at_cone_distance(
        cone_dist, m.pitch_angle_rad, m.face_angle_rad, m.outside_dia_mm / 2.0,
        m.crown_to_apex_mm);
    tip = std::min(tip, involute::max_tip_radius(
        base, psi0, involute::kMinTopLandFactor * p.module * scale));
    const double cap = tip + involute::kCutOvershootFactor * p.module *
                       std::max(scale, 0.2);
    const auto profile = involute::tooth_space_loop(
        base, root, tip, cap, psi0, half_pitch,
        p.fillet_factor * p.module * scale, flank_points, split_cap);
    return {name, end, cone_dist / std::cos(m.pitch_angle_rad),
            m.pitch_angle_rad, root, tip, cap, profile.filleted, cone_dist,
            phase_at_cone_distance(geo, name, cone_dist), profile};
}

std::vector<Point3> ToothSpaceSection::loop_3d() const
{
    std::vector<Point3> result;
    result.reserve(profile.loop.size());
    for (const auto& point : profile.loop)
        result.push_back(to_cone_3d(point.x, point.y, pitch_angle_rad,
                                    cone_apex_z_mm, phase_rad));
    return result;
}

std::vector<Point2> blank_outline(const SetGeometry& geo, const std::string& name)
{
    const auto& p = geo.parameters;
    const auto& m = geo.member(name);
    const double bore = p.bore / 2.0;
    const double min_wall = std::max(p.module, 1.0);
    const double z_back = m.root_to_apex_mm + std::max(0.0, p.min_root_thickness);
    const double z_front = front_face_z(geo, m);
    std::vector<Point2> outline{{bore, z_front},
                                {m.virtual_tip_r_inner_mm * std::cos(m.pitch_angle_rad), z_front},
                                {m.outside_dia_mm / 2.0, m.crown_to_apex_mm},
                                {m.outer_root_radius_mm, m.root_to_apex_mm}};
    if (z_back > m.root_to_apex_mm)
        outline.push_back({m.outer_root_radius_mm, z_back});
    if (p.hub_thickness > 0.0) {
        const double hub_r = std::max(
            bore + min_wall, std::min(bore + 2.0 * min_wall,
                                      0.7 * m.outer_root_radius_mm));
        outline.push_back({hub_r, z_back});
        outline.push_back({hub_r, z_back + p.hub_thickness});
        outline.push_back({bore, z_back + p.hub_thickness});
    } else outline.push_back({bore, z_back});
    return outline;
}

double end_overshoot(const SetGeometry& geo, const std::string& name,
                     std::optional<double> margin)
{
    const auto& p = geo.parameters;
    const auto& m = geo.member(name);
    const double margin_value = margin.value_or(std::max(0.2, 0.1 * p.module));
    const auto outline = blank_outline(geo, name);
    const double z_front = front_face_z(geo, m);
    double reach = 0.0;
    for (const auto& point : outline) {
        if (point.x >= m.outer_root_radius_mm - 1e-9)
            reach = std::max(reach, beyond_back_cone(geo, m,
                                                     {point.x, 0.0, point.y}));
    }
    double over = std::max(kEndOvershootMinMm,
                           kEndOvershootFraction * p.face_width);
    for (int i = 0; i < 40; ++i) {
        const auto outer = tooth_space_section(geo, name, "outer",
                                               involute::kFlankPoints, over);
        const auto inner = tooth_space_section(geo, name, "inner",
                                               involute::kFlankPoints, over);
        double min_back = std::numeric_limits<double>::infinity();
        double max_front = -std::numeric_limits<double>::infinity();
        for (const auto& point : outer.loop_3d())
            min_back = std::min(min_back, beyond_back_cone(geo, m, point));
        for (const auto& point : inner.loop_3d()) max_front = std::max(max_front, point.z);
        if (min_back > reach + margin_value && max_front < z_front - margin_value)
            return over;
        over *= 1.25;
    }
    return over;
}

std::pair<double, double> section_span(const SetGeometry& geo,
                                       const std::string& name)
{
    const double over = end_overshoot(geo, name);
    return {geo.outer_cone_dist_mm + over,
            std::max(geo.inner_cone_dist_mm - over,
                     0.05 * geo.outer_cone_dist_mm)};
}

std::vector<double> section_cone_distances(const SetGeometry& geo,
                                           const std::string& name,
                                           double max_sagitta)
{
    const auto [hi, lo] = section_span(geo, name);
    auto spread = [&](int n) {
        std::vector<double> result;
        result.reserve(n);
        for (int i = 0; i < n; ++i)
            result.push_back(hi + (lo - hi) * i / std::max(n - 1, 1));
        return result;
    };
    if (!geo.trace.has_value() || max_sagitta <= 0.0) return spread(2);
    for (int n = 2; n <= 201; ++n) {
        auto distances = spread(n);
        if (worst_sagitta(geo, name, distances) <= max_sagitta)
            return distances;
    }
    return spread(202);
}

int section_count(const SetGeometry& geo, const std::string& name,
                  double max_sagitta)
{
    return static_cast<int>(section_cone_distances(geo, name, max_sagitta).size());
}

std::vector<Point3> guide_spiral(const SetGeometry& geo, const std::string& name,
                                 double a_hi, double a_lo, int points)
{
    if (points < 2) throw std::invalid_argument("guide spiral needs two points");
    std::vector<Point3> result;
    result.reserve(points);
    for (int i = 0; i < points; ++i) {
        const double a = a_hi + (a_lo - a_hi) * i / (points - 1);
        const auto section = tooth_space_section(geo, name, "mid",
                                                 involute::kFlankPoints, 0.0, a);
        result.push_back(to_cone_3d(section.r_cap_mm, 0.0,
                                    section.pitch_angle_rad,
                                    section.cone_apex_z_mm, section.phase_rad));
    }
    return result;
}

} // namespace geargen::core::bevel
