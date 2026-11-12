use kinematics::{ElbowConfiguration, JointAngles, KinematicsError, Point, Scara};
use trajectory::LineSegment;

fn segment(start: Point, end: Point) -> LineSegment {
    LineSegment::new(start, end).expect("test segment must have a finite length")
}

fn point(x: f64, y: f64) -> Point {
    Point::new(x, y).unwrap_or_else(|_| panic!("invalid test point ({x}, {y})"))
}

fn angles(first_theta: f64, second_theta: f64) -> JointAngles {
    JointAngles::new(first_theta, second_theta)
        .unwrap_or_else(|_| panic!("invalid test angles ({first_theta}, {second_theta})"))
}

fn forward(robot: &Scara, angles: JointAngles) -> Point {
    robot
        .forward(angles)
        .unwrap_or_else(|_| panic!("valid forward kinematics rejected"))
}

fn configurations() -> [ElbowConfiguration; 2] {
    [ElbowConfiguration::Positive, ElbowConfiguration::Negative]
}

fn trajectory_points(line: &LineSegment, max_point_distance: f64) -> Vec<Point> {
    let points = match line.points(max_point_distance) {
        Ok(points) => points,
        Err(_) => panic!("valid trajectory unexpectedly rejected"),
    };

    points
        .map(|point| match point {
            Ok(point) => point,
            Err(_) => panic!("valid trajectory point unexpectedly rejected"),
        })
        .collect()
}

fn assert_point_close(actual: Point, expected: Point, characteristic_scale: f64) {
    let tolerance = characteristic_scale * 1e-10;
    assert!(
        (actual.x() - expected.x()).abs() <= tolerance
            && (actual.y() - expected.y()).abs() <= tolerance,
        "expected {expected:?}, got {actual:?}, tolerance {tolerance}"
    );
}

fn assert_inverse_forward_round_trip(robot: &Scara, point: Point, characteristic_scale: f64) {
    for configuration in configurations() {
        let angles = robot
            .inverse(point, configuration)
            .unwrap_or_else(|_| panic!("trajectory point {point:?} should be reachable"));
        assert!(angles.first_theta().is_finite(), "{point:?}: {angles:?}");
        assert!(angles.second_theta().is_finite(), "{point:?}: {angles:?}");
        assert_point_close(forward(robot, angles), point, characteristic_scale);
    }
}

#[test]
fn reachable_trajectories_round_trip_through_both_elbow_configurations() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let cases = [
        // Radial path spanning both workspace boundaries.
        (segment(point(40.0, 0.0), point(160.0, 0.0)), 7.0),
        // Chord between points on the outer boundary that stays outside the inner hole.
        (segment(point(160.0, 0.0), point(0.0, 160.0)), 13.0),
        // Interior path with changing x and y coordinates.
        (segment(point(60.0, -60.0), point(120.0, 60.0)), 9.0),
    ];

    for (line, maximum) in cases {
        let points = trajectory_points(&line, maximum);
        assert!(points.len() >= 2);
        assert_eq!(points[0].x(), line.start().x());
        assert_eq!(points[0].y(), line.start().y());
        assert_eq!(points.last().unwrap().x(), line.end().x());
        assert_eq!(points.last().unwrap().y(), line.end().y());
        for point in points {
            assert_inverse_forward_round_trip(&robot, point, 160.0);
        }
    }
}

#[test]
fn forward_trajectory_inverse_forward_composition_covers_a_joint_angle_grid() {
    // Equal arms have no inner unreachable disk, so every chord between generated
    // endpoints remains inside the workspace.
    let robot = Scara::new(75.0, 75.0).unwrap();
    let angle_pairs = [
        (-2.4, -1.2),
        (-1.0, 0.7),
        (0.0, 0.0),
        (0.8, -2.0),
        (2.2, 1.4),
    ];

    for (start_first, start_second) in angle_pairs {
        for (end_first, end_second) in angle_pairs {
            let start = forward(&robot, angles(start_first, start_second));
            let end = forward(&robot, angles(end_first, end_second));
            let line = segment(start, end);

            for point in trajectory_points(&line, 5.0) {
                assert_inverse_forward_round_trip(&robot, point, 150.0);
            }
        }
    }
}

#[test]
fn kinematics_rejects_only_the_trajectory_points_inside_the_inner_workspace_hole() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let line = segment(point(-160.0, 0.0), point(160.0, 0.0));
    let points = trajectory_points(&line, 20.0);
    assert_eq!(points.len(), 17);

    let mut unreachable_points = 0;
    for point in points {
        let should_be_reachable = point.distance_from_origin().unwrap() >= 40.0;
        if !should_be_reachable {
            unreachable_points += 1;
        }

        for configuration in configurations() {
            let result = robot.inverse(point, configuration);
            if should_be_reachable {
                let angles = result.unwrap_or_else(|_| panic!("{point:?} should be reachable"));
                assert_point_close(forward(&robot, angles), point, 160.0);
            } else {
                assert!(matches!(result, Err(KinematicsError::UnreachablePoint)));
            }
        }
    }

    assert_eq!(unreachable_points, 3);
}

#[test]
fn degenerate_trajectory_emits_two_solvable_endpoint_samples() {
    let robot = Scara::new(100.0, 60.0).unwrap();
    let point = forward(&robot, angles(0.7, -1.2));
    let line = segment(point, point);
    let points = trajectory_points(&line, f64::MAX);

    assert_eq!(points.len(), 2);
    for point in points {
        assert_inverse_forward_round_trip(&robot, point, 160.0);
    }
}

#[test]
fn trajectory_and_kinematics_composition_is_stable_across_finite_scales() {
    for scale in [1e-200, 1e-100, 1e-6, 1.0, 1e6, 1e100, 1e200] {
        let robot = Scara::new(scale, scale).unwrap();
        let line = segment(point(scale, 0.0), point(0.0, scale));
        let points = trajectory_points(&line, scale / 5.0);

        assert!(points.len() >= 2);
        for point in points {
            assert_inverse_forward_round_trip(&robot, point, scale);
        }
    }
}
