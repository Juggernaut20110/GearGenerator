#include "core/involute/involute.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using geargen::core::Point2;
using geargen::core::distance;
using geargen::core::involute::polar;

double radius_of(Point2 point) noexcept
{
    return std::hypot(point.x, point.y);
}

double angle_of(Point2 point) noexcept
{
    return std::atan2(point.y, point.x);
}

Point2 mirror(Point2 point) noexcept
{
    return {point.x, -point.y};
}

std::pair<double, Point2> point_segment_distance(Point2 p, Point2 a,
                                                 Point2 b) noexcept
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double denominator = dx * dx + dy * dy;
    if (denominator < 1e-18) {
        return {distance(p, a), a};
    }
    const double t = std::clamp(
        ((p.x - a.x) * dx + (p.y - a.y) * dy) / denominator, 0.0, 1.0);
    const Point2 q{a.x + t * dx, a.y + t * dy};
    return {distance(p, q), q};
}

struct PolylineDistance {
    double distance{std::numeric_limits<double>::infinity()};
    Point2 point{};
    int segment{};
};

PolylineDistance distance_to_polyline(Point2 p,
                                       const std::vector<Point2>& poly)
{
    if (poly.empty()) {
        throw std::invalid_argument("polyline must contain at least one point");
    }
    PolylineDistance best{std::numeric_limits<double>::infinity(), poly.front(),
                          0};
    for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
        const auto [d, q] = point_segment_distance(p, poly[i], poly[i + 1]);
        if (d < best.distance) {
            best = {d, q, static_cast<int>(i)};
        }
    }
    return best;
}

double angular_intersection_residual(
    const geargen::core::involute::RackGeneratedRoot& root, double phi)
{
    const auto [radius, angle] = root.polar(phi);
    if (radius < root.r_base - 1e-11 * std::max(1.0, root.r_base)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto [unused_point, roll] = root.nominal_involute_at_radius(radius);
    (void)unused_point;
    const double involute_angle = root.base_space_angle() + roll - std::atan(roll);
    return angle - involute_angle;
}

double base_circle_boundary(const geargen::core::involute::RackGeneratedRoot& root,
                            double valid_phi, double invalid_phi)
{
    if (root.polar(valid_phi).first < root.r_base ||
        root.polar(invalid_phi).first >= root.r_base) {
        throw std::invalid_argument("invalid base-circle boundary bracket");
    }
    double lo = valid_phi;
    double hi = invalid_phi;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (root.polar(mid).first >= root.r_base) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

double base_boundary_residual(
    const geargen::core::involute::RackGeneratedRoot& root, double phi)
{
    return root.polar(phi).second - root.base_space_angle();
}

double bisect_intersection(
    const geargen::core::involute::RackGeneratedRoot& root, double lo,
    double hi, double flo, double fhi)
{
    if (std::abs(flo) <= 1e-14) {
        return lo;
    }
    if (std::abs(fhi) <= 1e-14) {
        return hi;
    }
    for (int i = 0; i < 120; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double fmid = angular_intersection_residual(root, mid);
        if (std::isnan(fmid)) {
            lo = mid;
            continue;
        }
        if (std::abs(fmid) <= 1e-14) {
            return mid;
        }
        if (flo * fmid <= 0.0) {
            hi = mid;
            fhi = fmid;
        } else {
            lo = mid;
            flo = fmid;
        }
    }
    return 0.5 * (lo + hi);
}

} // namespace

namespace geargen::core::involute {

double involute_function(double pressure_angle_rad) noexcept
{
    return std::tan(pressure_angle_rad) - pressure_angle_rad;
}

double inv(double angle_rad) noexcept
{
    return involute_function(angle_rad);
}

Point2 polar(double radius, double angle_rad) noexcept
{
    return {radius * std::cos(angle_rad), radius * std::sin(angle_rad)};
}

ProfilePoint external_point(double base_radius_mm, double roll_parameter,
                            double base_space_angle_rad)
{
    const double radius = base_radius_mm *
                          std::sqrt(1.0 + roll_parameter * roll_parameter);
    const double angle = base_space_angle_rad + roll_parameter -
                         std::atan(roll_parameter);
    return {polar(radius, angle), roll_parameter};
}

double top_land(double radius, double base_radius, double psi0)
{
    return 2.0 * (psi0 - inv(std::acos(std::min(1.0, base_radius / radius)))) *
           radius;
}

double max_tip_radius(double base_radius, double psi0, double min_land)
{
    if (top_land(base_radius, base_radius, psi0) <= min_land) {
        return base_radius;
    }
    double lo = base_radius;
    double hi = base_radius;
    for (int i = 0; i < 200; ++i) {
        hi *= 1.05;
        if (top_land(hi, base_radius, psi0) <= min_land) {
            break;
        }
    }
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (top_land(mid, base_radius, psi0) > min_land) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

double internal_space_width(double radius, double base_radius, double psi0)
{
    return top_land(radius, base_radius, psi0);
}

double internal_tooth_width(double radius, double base_radius, double psi0,
                            double half_pitch)
{
    const double alpha = std::acos(std::min(1.0, base_radius / radius));
    return 2.0 * (half_pitch - (psi0 - inv(alpha))) * radius;
}

double min_internal_tip_radius(double base_radius, double psi0,
                               double half_pitch, double min_land)
{
    if (internal_tooth_width(base_radius, base_radius, psi0, half_pitch) >
        min_land) {
        return base_radius;
    }
    double lo = base_radius;
    double hi = base_radius;
    for (int i = 0; i < 200; ++i) {
        hi *= 1.05;
        if (internal_tooth_width(hi, base_radius, psi0, half_pitch) > min_land) {
            break;
        }
    }
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (internal_tooth_width(mid, base_radius, psi0, half_pitch) > min_land) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return hi;
}

std::vector<Point2> flank_points(double base_radius, double root_radius,
                                 double tip_radius, double psi0,
                                 double half_pitch, int count)
{
    if (count < 2) {
        throw std::invalid_argument("at least two flank samples are required");
    }
    const auto space_angle = [&](double radius) {
        const double alpha = std::acos(std::min(1.0, base_radius / radius));
        return half_pitch - psi0 + inv(alpha);
    };
    std::vector<Point2> points;
    const double r_lo = std::max(root_radius, base_radius);
    if (root_radius < base_radius) {
        points.push_back(polar(root_radius, space_angle(base_radius)));
    }
    const double t_lo =
        std::sqrt(std::max(0.0, (r_lo / base_radius) * (r_lo / base_radius) - 1.0));
    const double t_hi =
        std::sqrt(std::max(0.0, (tip_radius / base_radius) * (tip_radius / base_radius) - 1.0));
    points.reserve(points.size() + static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double t = t_lo + (t_hi - t_lo) * i / (count - 1.0);
        const double radius = base_radius * std::sqrt(1.0 + t * t);
        points.push_back(polar(radius, space_angle(radius)));
    }
    return points;
}

std::vector<Point2> internal_flank_points(double base_radius, double root_radius,
                                          double tip_radius, double psi0,
                                          int count)
{
    if (count < 2) {
        throw std::invalid_argument("at least two flank samples are required");
    }
    const double tip = std::max(tip_radius, base_radius);
    const double t_lo = std::sqrt(std::max(
        0.0, (tip / base_radius) * (tip / base_radius) - 1.0));
    const double t_hi = std::sqrt(
        std::max(0.0, (root_radius / base_radius) * (root_radius / base_radius) -
                           1.0));
    std::vector<Point2> points;
    points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double t = t_lo + (t_hi - t_lo) * i / (count - 1.0);
        const double radius = base_radius * std::sqrt(1.0 + t * t);
        const double alpha = std::acos(std::min(1.0, base_radius / radius));
        points.push_back(polar(radius, psi0 - inv(alpha)));
    }
    return points;
}

Point2 RackGeneratedRoot::point(double phi) const
{
    const double c = std::cos(phi);
    const double s = std::sin(phi);
    const double a = reference_r + v_centre;
    const double b = u_centre - reference_r * phi;
    const double cx = c * a - s * b;
    const double cy = s * a + c * b;
    const double dcx = -s * a - c * b + s * reference_r;
    const double dcy = c * a - s * b - c * reference_r;
    const double speed = std::hypot(dcx, dcy);
    if (speed <= 1e-14) {
        throw std::runtime_error("rack root envelope has a stationary tool corner");
    }
    const Point2 raw{cx + rack_root_radius * dcy / speed,
                     cy - rack_root_radius * dcx / speed};
    return rotate(raw, pitch_space_angle);
}

std::pair<double, double> RackGeneratedRoot::polar(double phi) const
{
    const Point2 p = point(phi);
    return {radius_of(p), angle_of(p)};
}

Point2 RackGeneratedRoot::nominal_involute_point(double roll) const
{
    if (roll < -1e-14) {
        throw std::invalid_argument("involute roll parameter must be non-negative");
    }
    roll = std::max(0.0, roll);
    const double radius = r_base * std::sqrt(1.0 + roll * roll);
    const double angle = base_space_angle() + roll - std::atan(roll);
    return geargen::core::involute::polar(radius, angle);
}

std::pair<Point2, double> RackGeneratedRoot::nominal_involute_at_radius(
    double radius) const
{
    if (radius < r_base - 1e-12) {
        throw std::invalid_argument("nominal involute is undefined below base circle");
    }
    const double roll = std::sqrt(std::max(0.0, (radius / r_base) *
                                                    (radius / r_base) - 1.0));
    return {nominal_involute_point(roll), roll};
}

std::vector<Point2> RackGeneratedRoot::sample_to(double phi_end, int n) const
{
    if (n < 2) {
        throw std::invalid_argument("at least two root samples are required");
    }
    std::vector<Point2> points;
    points.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double phi = phi_root + (phi_end - phi_root) * i / (n - 1.0);
        points.push_back(point(phi));
    }
    points.front() = geargen::core::involute::polar(
        generated_root_r(), pitch_space_angle + phi_root);
    return points;
}

bool RackGeneratedRoot::undercut() const noexcept
{
    const double corner_depth =
        cutter_tip_depth - rack_root_radius * (1.0 - std::sin(alpha));
    const double scale = std::max({1.0, std::abs(reference_r),
                                   std::abs(corner_depth)});
    return reference_r * std::sin(alpha) * std::sin(alpha) - corner_depth <
           -1e-14 * scale;
}

std::optional<RackGeneratedRoot> rack_generated_root(
    double reference_radius, double base_radius, double root_radius,
    double psi0, double half_pitch, double cutter_tip_depth,
    double rack_root_radius)
{
    if (reference_radius <= 0.0 || base_radius <= 0.0 || root_radius <= 0.0 ||
        cutter_tip_depth <= 0.0 || rack_root_radius <= 0.0) {
        return std::nullopt;
    }
    const double ratio = base_radius / reference_radius;
    if (ratio <= 0.0 || ratio > 1.0 + 1e-12) {
        return std::nullopt;
    }
    const double alpha = std::acos(std::min(1.0, ratio));
    if (alpha <= 1e-12 || alpha >= kPi / 2.0) {
        return std::nullopt;
    }
    const double cos_alpha = std::cos(alpha);
    const double tan_alpha = std::tan(alpha);
    const double sec_alpha = 1.0 / cos_alpha;
    const double v_centre = -cutter_tip_depth + rack_root_radius;
    const double u_centre = tan_alpha * v_centre - rack_root_radius * sec_alpha;
    const double v_touch = v_centre - rack_root_radius * std::sin(alpha);
    const double phi_transition =
        v_touch * (1.0 + tan_alpha * tan_alpha) /
        (tan_alpha * reference_radius);
    const double phi_root = u_centre / reference_radius;
    if (phi_transition >= phi_root) {
        return std::nullopt;
    }
    if (std::abs(reference_radius - cutter_tip_depth - root_radius) >
        1e-8 * std::max(1.0, reference_radius)) {
        return std::nullopt;
    }
    RackGeneratedRoot result{
        reference_radius,
        base_radius,
        root_radius,
        psi0,
        half_pitch,
        cutter_tip_depth,
        rack_root_radius,
        alpha,
        half_pitch - psi0 + inv(alpha),
        phi_root,
        phi_transition,
        v_centre,
        u_centre,
    };
    try {
        const Point2 root_point = result.point(phi_root);
        const Point2 transition_point = result.point(phi_transition);
        if (!std::isfinite(radius_of(root_point)) ||
            radius_of(transition_point) <= root_radius ||
            !std::isfinite(radius_of(transition_point))) {
            return std::nullopt;
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return result;
}

std::optional<StartOfInvolute> solve_start_of_involute(
    const RackGeneratedRoot& root)
{
    const bool is_undercut = root.undercut();
    if (!is_undercut) {
        const double phi = root.phi_flank_transition;
        const auto [radius, angle] = root.polar(phi);
        if (radius < root.r_base) {
            return std::nullopt;
        }
        const auto [unused_point, roll] = root.nominal_involute_at_radius(radius);
        (void)unused_point;
        return StartOfInvolute{radius, angle, roll, phi, false, root.point(phi)};
    }

    const double lo = std::min(root.phi_root, root.phi_flank_transition);
    const double hi = std::max(root.phi_root, root.phi_flank_transition);
    struct Bracket {
        double lo;
        double hi;
        double flo;
        double fhi;
    };
    std::vector<Bracket> brackets;
    std::optional<std::pair<double, double>> previous;
    for (int i = 0; i < 2049; ++i) {
        const double phi = lo + (hi - lo) * i / 2048.0;
        const double value = angular_intersection_residual(root, phi);
        if (std::isnan(value)) {
            if (previous.has_value()) {
                const double boundary = base_circle_boundary(
                    root, previous->first, phi);
                const double boundary_value =
                    base_boundary_residual(root, boundary);
                if (previous->second * boundary_value < 0.0) {
                    brackets.push_back(
                        {previous->first, boundary, previous->second,
                         boundary_value});
                }
                previous.reset();
            }
            continue;
        }
        if (std::abs(value) <= 1e-13) {
            brackets.push_back({phi, phi, value, value});
        } else if (previous.has_value() && previous->second * value < 0.0) {
            brackets.push_back(
                {previous->first, phi, previous->second, value});
        }
        previous = std::make_pair(phi, value);
    }
    if (brackets.empty()) {
        return std::nullopt;
    }
    std::vector<double> roots;
    roots.reserve(brackets.size());
    for (const auto& bracket : brackets) {
        roots.push_back(bracket.lo == bracket.hi
                            ? bracket.lo
                            : bisect_intersection(root, bracket.lo, bracket.hi,
                                                   bracket.flo, bracket.fhi));
    }
    const auto best = std::min_element(
        roots.begin(), roots.end(), [&](double lhs, double rhs) {
            return root.polar(lhs).first < root.polar(rhs).first;
        });
    const double phi = *best;
    const auto [radius, angle] = root.polar(phi);
    const auto [unused_point, roll] = root.nominal_involute_at_radius(radius);
    (void)unused_point;
    return StartOfInvolute{radius, angle, roll, phi, true, root.point(phi)};
}

std::optional<RackRootEnvelope> rack_root_envelope(
    double reference_radius, double base_radius, double root_radius,
    double psi0, double half_pitch, double cutter_tip_depth,
    double rack_root_radius, int count)
{
    const auto root = rack_generated_root(
        reference_radius, base_radius, root_radius, psi0, half_pitch,
        cutter_tip_depth, rack_root_radius);
    if (!root.has_value()) {
        return std::nullopt;
    }
    const auto soi = solve_start_of_involute(*root);
    if (!soi.has_value()) {
        return std::nullopt;
    }
    return RackRootEnvelope{root->sample_to(soi->trochoid_parameter, count),
                            soi->start_of_involute_r};
}

std::optional<FilletResult> root_fillet(const std::vector<Point2>& flank,
                                        double root_radius, double fillet_radius,
                                        int arc_points, bool internal)
{
    if (fillet_radius <= 0.0 || flank.empty() || arc_points < 2) {
        return std::nullopt;
    }
    const double phi_flank = angle_of(flank.front());
    if (phi_flank <= 0.0) {
        return std::nullopt;
    }
    const double centre_radius =
        internal ? root_radius - fillet_radius : root_radius + fillet_radius;
    if (centre_radius <= 0.0) {
        return std::nullopt;
    }
    const auto gap = [&](double phi) {
        return distance_to_polyline(polar(centre_radius, phi), flank).distance -
               fillet_radius;
    };
    double lo = 0.0;
    double hi = phi_flank;
    if (gap(lo) <= 0.0) {
        return std::nullopt;
    }
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (gap(mid) > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const double phi_c = 0.5 * (lo + hi);
    const Point2 centre = polar(centre_radius, phi_c);
    const auto closest = distance_to_polyline(centre, flank);
    const Point2 root_touch = polar(root_radius, phi_c);
    double a0 = angle_of(root_touch - centre);
    double a1 = angle_of(closest.point - centre);
    while (a1 - a0 > kPi) {
        a1 -= kTau;
    }
    while (a0 - a1 > kPi) {
        a1 += kTau;
    }
    std::vector<Point2> arc;
    arc.reserve(static_cast<std::size_t>(arc_points));
    for (int i = 0; i < arc_points; ++i) {
        const double angle = a0 + (a1 - a0) * i / (arc_points - 1.0);
        arc.push_back({centre.x + fillet_radius * std::cos(angle),
                       centre.y + fillet_radius * std::sin(angle)});
    }
    std::vector<Point2> trimmed{closest.point};
    if (closest.segment + 1 < static_cast<int>(flank.size())) {
        trimmed.insert(trimmed.end(), flank.begin() + closest.segment + 1,
                       flank.end());
    }
    return FilletResult{std::move(trimmed), std::move(arc)};
}

const std::vector<Point2>* ToothSpaceLoop::find_segment(
    const std::string& name) const noexcept
{
    for (const auto& segment : segments) {
        if (segment.name == name) {
            return &segment.points;
        }
    }
    return nullptr;
}

ToothSpaceLoop tooth_space_loop(
    double base_radius, double root_radius, double tip_radius,
    double cap_radius, double psi0, double half_pitch, double fillet_radius,
    int flank_count, bool split_cap, bool internal,
    const std::optional<RackRootEnvelope>& rack_root)
{
    if (flank_count < 2) {
        throw std::invalid_argument("at least two flank samples are required");
    }
    std::vector<Point2> generated_root;
    std::vector<Point2> flank;
    std::optional<FilletResult> fillet;
    if (internal) {
        flank = internal_flank_points(base_radius, root_radius, tip_radius, psi0,
                                      flank_count);
        std::reverse(flank.begin(), flank.end());
        fillet = root_fillet(flank, root_radius, fillet_radius, 9, true);
    } else if (rack_root.has_value()) {
        generated_root = rack_root->points;
        if (tip_radius <= rack_root->transition_radius + 1e-10) {
            generated_root.clear();
            flank = flank_points(base_radius, root_radius, tip_radius, psi0,
                                 half_pitch, flank_count);
            fillet = root_fillet(flank, root_radius, fillet_radius);
        } else {
            flank = flank_points(base_radius, rack_root->transition_radius,
                                 tip_radius, psi0, half_pitch, flank_count);
            if (flank.empty() ||
                distance(generated_root.back(), flank.front()) > 1e-8) {
                generated_root.clear();
                fillet = root_fillet(flank, root_radius, fillet_radius);
            } else {
                generated_root.back() = flank.front();
            }
        }
    } else {
        flank = flank_points(base_radius, root_radius, tip_radius, psi0,
                             half_pitch, flank_count);
        fillet = root_fillet(flank, root_radius, fillet_radius);
    }

    std::vector<Point2> arc;
    if (fillet.has_value()) {
        flank = std::move(fillet->flank);
        arc = std::move(fillet->arc);
    } else {
        arc = generated_root;
    }
    const double phi_root = angle_of(!arc.empty() ? arc.front() : flank.front());
    const double phi_tip = angle_of(flank.back());
    std::vector<Point2> root_arc;
    root_arc.reserve(9);
    for (int i = 0; i < 9; ++i) {
        root_arc.push_back(
            polar(root_radius, -phi_root + 2.0 * phi_root * i / 8.0));
    }
    std::vector<Point2> cap;
    cap.reserve(5);
    for (int i = 0; i < 5; ++i) {
        cap.push_back(polar(cap_radius, -phi_tip + 2.0 * phi_tip * i / 4.0));
    }
    const auto mirrored = [](const std::vector<Point2>& points) {
        std::vector<Point2> result;
        result.reserve(points.size());
        for (const auto point : points) {
            result.push_back(mirror(point));
        }
        return result;
    };
    const auto flank_neg = mirrored(flank);
    const auto arc_neg = mirrored(arc);
    std::vector<NamedSegment> segments{
        {"fillet_neg", arc_neg},
        {"generated_root_neg", mirrored(generated_root)},
        {"flank_neg", flank_neg},
        {"riser_neg", {flank_neg.back(), cap.front()}},
        {"cap", cap},
        {"riser_pos", {cap.back(), flank.back()}},
        {"flank_pos", std::vector<Point2>(flank.rbegin(), flank.rend())},
        {"fillet_pos", std::vector<Point2>(arc.rbegin(), arc.rend())},
        {"generated_root_pos",
         std::vector<Point2>(generated_root.rbegin(), generated_root.rend())},
        {"root", std::vector<Point2>(root_arc.rbegin(), root_arc.rend())},
    };
    if (split_cap) {
        const auto cap_index = std::find_if(
            segments.begin(), segments.end(), [](const NamedSegment& segment) {
                return segment.name == "cap";
            });
        const int mid = static_cast<int>(cap.size() / 2);
        std::vector<Point2> cap_neg(cap.begin(), cap.begin() + mid + 1);
        std::vector<Point2> cap_pos(cap.begin() + mid, cap.end());
        *cap_index = {"cap_neg", std::move(cap_neg)};
        segments.insert(cap_index + 1,
                        NamedSegment{"cap_pos", std::move(cap_pos)});
    }
    const std::vector<std::string> order = split_cap
                                                ? std::vector<std::string>{
                                                      "fillet_neg", "flank_neg",
                                                      "riser_neg", "cap_neg",
                                                      "cap_pos", "riser_pos",
                                                      "flank_pos", "fillet_pos",
                                                      "root"}
                                                : std::vector<std::string>{
                                                      "fillet_neg", "flank_neg",
                                                      "riser_neg", "cap",
                                                      "riser_pos", "flank_pos",
                                                      "fillet_pos", "root"};
    std::vector<Point2> loop;
    for (const auto& name : order) {
        const auto segment = std::find_if(
            segments.begin(), segments.end(), [&](const NamedSegment& item) {
                return item.name == name;
            });
        if (segment == segments.end()) {
            continue;
        }
        for (const auto point : segment->points) {
            if (loop.empty() || distance(loop.back(), point) > 1e-9) {
                loop.push_back(point);
            }
        }
    }
    if (loop.size() > 1 && distance(loop.front(), loop.back()) < 1e-9) {
        loop.pop_back();
    }
    return {std::move(segments), std::move(loop), !arc.empty()};
}

} // namespace geargen::core::involute
