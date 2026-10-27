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
        let trajectory_points = line.points(max_point_distance).map_err(PlannerError::Trajectory);

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

        let point = self.trajectory_points.next();

        let point = match point {
            Some(point) => match point.map_err(PlannerError::Trajectory) {
                Ok(ok) => ok,
                Err(err) => {
                    self.is_finished = true;
                    return Some(Err(err));
                }
            },
            None => {
                self.is_finished = true;
                return None;
            }
        };

        let angles = self.scara.inverse(point, self.configuration).map_err(PlannerError::Kinematics);

        match angles {
            Ok(ok) => Some(Ok(ok)),
            Err(err) => {
                self.is_finished = true;
                Some(Err(err))
            }
        }
    }
}
