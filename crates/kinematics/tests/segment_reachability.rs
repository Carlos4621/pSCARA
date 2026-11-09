use kinematics::{Point, Scara};

fn point(x: f64, y: f64) -> Point {
    Point::new(x, y).expect("test coordinates must be finite")
}

fn robot(first_arm: f64, second_arm: f64) -> Scara {
    Scara::new(first_arm, second_arm).expect("test arm lengths must be valid")
}

#[test]
fn chord_wholly_inside_the_annulus_is_reachable_in_either_direction() {
    let scara = robot(100.0, 60.0); // Reachable radii: 40 through 160.
    let start = point(100.0, 0.0);
    let end = point(0.0, 100.0);
    // The chord's nearest point has radius sqrt(5_000), greater than 40.

    assert_eq!(scara.is_segment_reachable(start, end), Ok(true));
    assert_eq!(scara.is_segment_reachable(end, start), Ok(true));
}

#[test]
fn chord_crossing_the_inner_hole_is_unreachable_even_with_outer_boundary_endpoints() {
    let scara = robot(100.0, 60.0);
    let start = point(160.0, 0.0);
    let end = point(-160.0, 0.0);
    // Both endpoints are reachable, but the midpoint lies at the origin.

    assert_eq!(scara.is_segment_reachable(start, end), Ok(false));
    assert_eq!(scara.is_segment_reachable(end, start), Ok(false));
}

#[test]
fn tangent_to_inner_boundary_is_reachable_but_nearby_crossing_is_not() {
    let scara = robot(100.0, 60.0);

    assert_eq!(
        scara.is_segment_reachable(point(40.0, -100.0), point(40.0, 100.0)),
        Ok(true)
    );
    assert_eq!(
        scara.is_segment_reachable(point(39.0, -100.0), point(39.0, 100.0)),
        Ok(false)
    );
}

#[test]
fn outer_boundary_endpoints_do_not_require_the_chord_to_stay_on_the_boundary() {
    let scara = robot(100.0, 60.0);
    // Its midpoint has radius sqrt(12_800), safely outside the inner hole.

    assert_eq!(
        scara.is_segment_reachable(point(160.0, 0.0), point(0.0, 160.0)),
        Ok(true)
    );
}

#[test]
fn an_unreachable_endpoint_makes_the_whole_segment_unreachable() {
    let scara = robot(100.0, 60.0);

    for outside in [point(39.0, 0.0), point(161.0, 0.0)] {
        assert_eq!(
            scara.is_segment_reachable(outside, point(100.0, 0.0)),
            Ok(false)
        );
        assert_eq!(
            scara.is_segment_reachable(point(100.0, 0.0), outside),
            Ok(false)
        );
    }
}

#[test]
fn zero_length_segment_has_the_reachability_of_its_endpoint() {
    let scara = robot(100.0, 60.0);

    for reachable in [point(40.0, 0.0), point(100.0, 0.0), point(160.0, 0.0)] {
        assert_eq!(scara.is_segment_reachable(reachable, reachable), Ok(true));
    }
    assert_eq!(
        scara.is_segment_reachable(point(0.0, 0.0), point(0.0, 0.0)),
        Ok(false)
    );
}

#[test]
fn equal_arms_have_no_inner_hole_so_a_diameter_is_reachable() {
    let scara = robot(100.0, 100.0);

    assert_eq!(
        scara.is_segment_reachable(point(100.0, 0.0), point(-100.0, 0.0)),
        Ok(true)
    );
}

#[test]
fn normalized_chord_is_reachable_when_raw_squared_coordinates_underflow() {
    let scale = 1e-200;
    let scara = robot(scale, scale);

    assert_eq!(
        scara.is_segment_reachable(point(scale, 0.0), point(0.0, scale)),
        Ok(true)
    );
}

#[test]
fn normalized_chord_is_reachable_when_raw_squared_coordinates_overflow() {
    let scale = 1e200;
    let scara = robot(scale, scale);

    assert_eq!(
        scara.is_segment_reachable(point(scale, 0.0), point(0.0, scale)),
        Ok(true)
    );
}
