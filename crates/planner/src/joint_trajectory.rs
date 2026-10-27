use kinematics::{ElbowConfiguration, JointAngles, Scara};
use trajectory::{LineSegment, TrajectoryPoints};

use crate::PlannerError;

#[derive(Debug)]
pub struct JointTrajectory<'a> {
    scara: &'a Scara,
    trajectory_points: TrajectoryPoints<'a>,
    configuration: ElbowConfiguration,
    is_finished: bool,
}

impl<'a> JointTrajectory<'a> {
    pub fn new(
        scara: &'a Scara,
        line: &'a LineSegment,
        max_point_distance: f64,
        configuration: ElbowConfiguration,
    ) -> Result<Self, PlannerError> {
        let trajectory_points = line
            .points(max_point_distance)
            .map_err(PlannerError::Trajectory);

        match trajectory_points {
            Ok(trajectory_points) => Ok(Self {
                scara,
                trajectory_points,
                configuration,
                is_finished: false,
            }),
            Err(err) => Err(err),
        }
    }
}

impl<'a> Iterator for JointTrajectory<'a> {
    type Item = Result<JointAngles, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.is_finished {
            return None;
        }

        let Some(point) = self.trajectory_points.next() else {
            self.is_finished = true;
            return None;
        };

        let result = point.map_err(PlannerError::Trajectory).and_then(|point| {
            self.scara.inverse(point, self.configuration).map_err(PlannerError::Kinematics)
        });

        if result.is_err() {
            self.is_finished = true;
            return None;
        }

        Some(result)
    }
}
