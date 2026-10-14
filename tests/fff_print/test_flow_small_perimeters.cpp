#include <catch2/catch_all.hpp>

#include "test_helpers.hpp"

#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <map>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// Total extruded length (sum of dist_E) over lines whose per-line comment
// contains `role` (requires gcode_comments, forced on by init_print()).
// Same scan pattern as test_helpers' layers_with_role()/role_passes(), summing
// extrusion instead of collecting layers/passes.
double extruded_length_for_role(const std::string &gcode, const std::string &role)
{
    double total = 0.0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.comment().find(role) != std::string_view::npos)
            total += line.dist_E(self);
    });
    return total;
}

// Total extruded length for the feature whose ";TYPE:<type_name>" section header
// (a comment-only, non-extruding line) contains `type_name`. Needed to distinguish
// "Outer wall" from "Inner wall": both share the same generic per-line "perimeter"
// comment used by extruded_length_for_role(), so only the TYPE header tells them
// apart. The header line itself isn't an extruding line, so this tracks state
// across lines instead of matching a single line's own comment.
//
// Only lines with actual XY movement are counted: a bare deretraction (G1 E... with
// no X/Y) still satisfies line.extruding(), but its E amount is a fixed per-loop
// retraction-compensation value that doesn't scale with any flow ratio. Including it
// skews the ratio away from the true per-segment flow value, especially on this small
// test geometry where fixed retraction volume is a large fraction of the real extrusion.
double extruded_length_for_type(const std::string &gcode, const std::string &type_name)
{
    double total = 0.0;
    bool in_type = false;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind("TYPE:", 0) == 0) {
            in_type = comment.find(type_name) != std::string_view::npos;
            return;
        }
        if (in_type && line.extruding(self) && line.dist_XY(self) > 0)
            total += line.dist_E(self);
    });
    return total;
}

// A thin, short cylinder: its wall length is well under small_perimeter_threshold
// below, so its entire "perimeter" extrusion goes through the small-perimeter branch.
std::string slice_small_cylinder(double flow_small_perimeters_ratio)
{
    return slice({ make_cylinder(7.5, 3.0) }, {
        { "small_perimeter_threshold", 10 },
        { "set_other_flow_ratios",     true },
        { "flow_small_perimeters",     flow_small_perimeters_ratio },
        { "skirt_loops",               0 },
        { "brim_type",                 "no_brim" },
    });
}

} // namespace

TEST_CASE("flow_small_perimeters overrides wall flow only for small perimeters", "[FlowSmallPerimeters]")
{
    const std::string gcode_ratio_1    = slice_small_cylinder(1.0);
    const std::string gcode_ratio_half = slice_small_cylinder(0.5);

    const double perimeter_e_ratio_1    = extruded_length_for_role(gcode_ratio_1, "perimeter");
    const double perimeter_e_ratio_half = extruded_length_for_role(gcode_ratio_half, "perimeter");

    // Sanity check: the cylinder actually produced perimeter extrusion to compare.
    REQUIRE(perimeter_e_ratio_1 > 0);

    // The whole wall is a "small perimeter" here, so halving the ratio should
    // halve the total extruded length on this role (same relationship verified
    // manually on real G-code before writing this test).
    CHECK(perimeter_e_ratio_half == Catch::Approx(perimeter_e_ratio_1 * 0.5).epsilon(0.02));

    // Control: an unrelated role (infill) must be unaffected by this ratio.
    const double infill_e_ratio_1    = extruded_length_for_role(gcode_ratio_1, "infill");
    const double infill_e_ratio_half = extruded_length_for_role(gcode_ratio_half, "infill");
    CHECK(infill_e_ratio_half == Catch::Approx(infill_e_ratio_1).epsilon(0.001));
}

TEST_CASE("flow_small_perimeters does not affect normal-sized perimeters", "[FlowSmallPerimeters]")
{
    // A cube wide enough that its walls are well above the (near-zero) threshold:
    // the small-perimeter branch should never trigger, so the ratio must be a no-op.
    auto gcode_for_ratio = [](double ratio) {
        return slice({ cube(40) }, {
            { "small_perimeter_threshold", 1 },
            { "set_other_flow_ratios",     true },
            { "flow_small_perimeters",     ratio },
            { "skirt_loops",               0 },
            { "brim_type",                 "no_brim" },
        });
    };

    const double perimeter_e_ratio_1    = extruded_length_for_role(gcode_for_ratio(1.0), "perimeter");
    const double perimeter_e_ratio_half = extruded_length_for_role(gcode_for_ratio(0.5), "perimeter");

    REQUIRE(perimeter_e_ratio_1 > 0);
    CHECK(perimeter_e_ratio_half == Catch::Approx(perimeter_e_ratio_1).epsilon(0.001));
}

TEST_CASE("small_perimeters_flow_outer_wall_only restricts the override to the outer wall", "[FlowSmallPerimeters]")
{
    // Two walls so the cylinder actually has a distinct "Outer wall" and "Inner wall".
    auto gcode_for = [](double ratio, bool outer_wall_only) {
        return slice({ make_cylinder(7.5, 3.0) }, {
            { "small_perimeter_threshold",            10 },
            { "set_other_flow_ratios",                true },
            { "flow_small_perimeters",                ratio },
            { "small_perimeters_flow_outer_wall_only", outer_wall_only },
            { "wall_loops",                            2 },
            { "skirt_loops",                           0 },
            { "brim_type",                             "no_brim" },
        });
    };

    // Reference at ratio=1: the checkbox is irrelevant here (no override to restrict).
    const std::string gcode_baseline = gcode_for(1.0, false);
    const double outer_baseline = extruded_length_for_type(gcode_baseline, "Outer wall");
    const double inner_baseline = extruded_length_for_type(gcode_baseline, "Inner wall");
    REQUIRE(outer_baseline > 0);
    REQUIRE(inner_baseline > 0);

    // Checkbox off (default): the 0.5 ratio applies to both walls.
    const std::string gcode_both_walls = gcode_for(0.5, false);
    CHECK(extruded_length_for_type(gcode_both_walls, "Outer wall") == Catch::Approx(outer_baseline * 0.5).epsilon(0.02));
    CHECK(extruded_length_for_type(gcode_both_walls, "Inner wall") == Catch::Approx(inner_baseline * 0.5).epsilon(0.02));

    // Checkbox on: only the outer wall keeps the 0.5 override; the inner wall falls
    // back to the normal inner_wall_flow_ratio (default 1), so it stays at baseline.
    const std::string gcode_outer_only = gcode_for(0.5, true);
    CHECK(extruded_length_for_type(gcode_outer_only, "Outer wall") == Catch::Approx(outer_baseline * 0.5).epsilon(0.02));
    CHECK(extruded_length_for_type(gcode_outer_only, "Inner wall") == Catch::Approx(inner_baseline).epsilon(0.02));
}
