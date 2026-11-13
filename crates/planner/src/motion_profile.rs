use kinematics::JointAngles;

use crate::{
    JointAccelerationLimits, JointVelocityLimits, PlannerError,
    profile_math::{product_ratio, sqrt_product_ratio},
};

enum MotionPhase {
    Acceleration,
    Cruise,
    Deceleration,
}

#[derive(Debug)]
pub struct MotionProfile {
    start_angle: f64,
    end_angle: f64,

    angular_displacement: f64,
    displacement_direction: f64,

    initial_speed: f64,
    final_speed: f64,
    peak_velocity: f64,

    max_acceleration: f64,

    acceleration_time: f64,
    cruise_time: f64,
    deceleration_time: f64,

    total_time: f64,
}

impl MotionProfile {
    pub fn new(
        start_angle: f64,
        end_angle: f64,
        max_acceleration: f64,
        max_velocity: f64,
    ) -> Result<Self, PlannerError> {
        Self::with_boundary_speeds(
            start_angle,
            end_angle,
            max_acceleration,
            max_velocity,
            0.0,
            0.0,
        )
    }

    /// Builds a minimum-time, monotone profile with the given boundary speeds.
    /// Speeds are finite, nonnegative magnitudes, bounded by `max_velocity`;
    /// the angle displacement determines the sign of the returned velocity.
    /// Equal angles require equal speeds and produce a zero-duration profile.
    /// With no displacement, the instantaneous velocity is the supplied speed.
    pub fn with_boundary_speeds(
        start_angle: f64,
        end_angle: f64,
        max_acceleration: f64,
        max_velocity: f64,
        initial_speed: f64,
        final_speed: f64,
    ) -> Result<Self, PlannerError> {
        utilities::validate_finite(start_angle).map_err(|_| PlannerError::AngleNonFinite)?;
        utilities::validate_finite(end_angle).map_err(|_| PlannerError::AngleNonFinite)?;
        utilities::validate_finite_positive(initial_speed)
            .map_err(|_| PlannerError::InvalidInitialSpeed)?;
        utilities::validate_finite_positive(final_speed)
            .map_err(|_| PlannerError::InvalidFinalSpeed)?;

        utilities::validate_non_zero_finite_positive(max_acceleration)
            .map_err(|_| PlannerError::InvalidMaxAcceleration)?;
        utilities::validate_non_zero_finite_positive(max_velocity)
            .map_err(|_| PlannerError::InvalidMaxVelocity)?;

        if initial_speed > max_velocity || final_speed > max_velocity {
            return Err(PlannerError::InvalidBoundarySpeed);
        }

        let delta = end_angle - start_angle;
        utilities::validate_finite(delta).map_err(|_| PlannerError::NumericalFailure)?;
        let angular_displacement = delta.abs();
        let displacement_direction = delta.signum();

        let higher_speed = initial_speed.max(final_speed);
        let lower_speed = initial_speed.min(final_speed);
        // |vf² - vi²| / (2a), without squaring or adding large speeds.
        let minimum_displacement =
            Self::speed_change_distance(higher_speed, lower_speed, max_acceleration);

        if angular_displacement < minimum_displacement
            || (angular_displacement == 0.0 && initial_speed != final_speed)
        {
            return Err(PlannerError::InfeasibleMotion);
        }

        let acceleration_distance =
            Self::speed_change_distance(max_velocity, initial_speed, max_acceleration);
        let deceleration_distance =
            Self::speed_change_distance(max_velocity, final_speed, max_acceleration);
        let critical_distance = acceleration_distance + deceleration_distance;

        let (peak_velocity, acceleration_time, cruise_time, deceleration_time) =
            if angular_displacement == 0.0 {
                (initial_speed, 0.0, 0.0, 0.0)
            } else if angular_displacement <= critical_distance {
                let remaining_distance = angular_displacement - minimum_displacement;
                // vp² = a*d + (vi² + vf²)/2 = a*(d - d_min) + max(vi,vf)².
                let peak = libm::hypot(
                    higher_speed,
                    sqrt_product_ratio(&[max_acceleration, remaining_distance], &[]),
                )
                .min(max_velocity);
                // Rationalize (vp - max(vi,vf))/a to retain small ramps when
                // the peak rounds to a boundary speed. Rest-to-rest also keeps
                // its duration when a*d underflows before the square root.
                let shorter_ramp = if higher_speed == 0.0 {
                    sqrt_product_ratio(&[angular_displacement], &[max_acceleration])
                } else {
                    product_ratio(&[remaining_distance], &[peak, 1.0 + higher_speed / peak])
                };
                let longer_ramp = shorter_ramp + (higher_speed - lower_speed) / max_acceleration;
                if initial_speed <= final_speed {
                    (peak, longer_ramp, 0.0, shorter_ramp)
                } else {
                    (peak, shorter_ramp, 0.0, longer_ramp)
                }
            } else {
                let cruise_distance =
                    (angular_displacement - acceleration_distance - deceleration_distance).max(0.0);
                (
                    max_velocity,
                    (max_velocity - initial_speed) / max_acceleration,
                    cruise_distance / max_velocity,
                    (max_velocity - final_speed) / max_acceleration,
                )
            };

        for value in [
            peak_velocity,
            acceleration_time,
            cruise_time,
            deceleration_time,
        ] {
            utilities::validate_finite_positive(value)
                .map_err(|_| PlannerError::NumericalFailure)?;
        }

        let total_time = acceleration_time + cruise_time + deceleration_time;

        utilities::validate_finite_positive(total_time)
            .map_err(|_| PlannerError::NumericalFailure)?;
        if angular_displacement > 0.0 && total_time == 0.0 {
            return Err(PlannerError::NumericalFailure);
        }

        Ok(Self {
            start_angle,
            end_angle,
            angular_displacement,
            displacement_direction,
            initial_speed,
            final_speed,
            peak_velocity,
            max_acceleration,
            acceleration_time,
            cruise_time,
            deceleration_time,
            total_time,
        })
    }

    pub fn total_time(&self) -> f64 {
        self.total_time
    }

    pub fn position_at(&self, time: f64) -> Result<f64, PlannerError> {
        self.position_at_scaled(time, self.total_time)
    }

    pub(crate) fn position_at_scaled(&self, time: f64, duration: f64) -> Result<f64, PlannerError> {
        Self::validate_duration_time(time, duration)?;

        if time == duration {
            return Ok(self.end_angle);
        }
        if time == 0.0 || self.angular_displacement == 0.0 {
            return Ok(self.start_angle);
        }

        let angle = match self.phase_at(time, duration) {
            MotionPhase::Acceleration => self.ramp_displacement(time, duration, self.initial_speed),
            MotionPhase::Cruise => {
                let ramp_distance = self.ramp_displacement(
                    self.acceleration_time,
                    self.total_time,
                    self.initial_speed,
                );
                ramp_distance
                    + product_ratio(
                        &[
                            self.peak_velocity,
                            time - self.scaled_ramp_time(duration),
                            self.total_time,
                        ],
                        &[duration],
                    )
            }
            MotionPhase::Deceleration => {
                self.angular_displacement
                    - self.ramp_displacement(duration - time, duration, self.final_speed)
            }
        };

        let result = self.start_angle
            + self.displacement_direction * angle.clamp(0.0, self.angular_displacement);

        utilities::validate_finite(result).map_err(|_| PlannerError::NumericalFailure)?;

        Ok(result)
    }

    pub fn velocity_at(&self, time: f64) -> Result<f64, PlannerError> {
        self.velocity_at_scaled(time, self.total_time)
    }

    pub(crate) fn velocity_at_scaled(&self, time: f64, duration: f64) -> Result<f64, PlannerError> {
        Self::validate_duration_time(time, duration)?;

        if self.total_time == 0.0 {
            return Ok(if duration == 0.0 {
                self.initial_speed
            } else {
                0.0
            });
        }
        if self.angular_displacement == 0.0 {
            return Ok(0.0);
        }

        if time == 0.0 || time == duration {
            let speed = if time == 0.0 {
                self.initial_speed
            } else {
                self.final_speed
            };
            return Ok(
                product_ratio(&[speed, self.total_time], &[duration]) * self.displacement_direction
            );
        }

        // Synchronization only stretches time, so rounding must not raise a limit.
        let peak = product_ratio(&[self.peak_velocity, self.total_time], &[duration])
            .min(self.peak_velocity);
        let velocity_magnitude = match self.phase_at(time, duration) {
            MotionPhase::Acceleration => self.ramp_velocity(time, duration, self.initial_speed),
            MotionPhase::Cruise => peak,
            MotionPhase::Deceleration => {
                self.ramp_velocity(duration - time, duration, self.final_speed)
            }
        };

        let result = velocity_magnitude.min(peak) * self.displacement_direction;

        utilities::validate_finite(result).map_err(|_| PlannerError::NumericalFailure)?;

        Ok(result)
    }

    pub fn acceleration_at(&self, time: f64) -> Result<f64, PlannerError> {
        self.acceleration_at_scaled(time, self.total_time)
    }

    pub(crate) fn acceleration_at_scaled(
        &self,
        time: f64,
        duration: f64,
    ) -> Result<f64, PlannerError> {
        Self::validate_duration_time(time, duration)?;

        if self.start_angle == self.end_angle {
            return Ok(0.0);
        }

        let acceleration = product_ratio(
            &[self.max_acceleration, self.total_time, self.total_time],
            &[duration, duration],
        )
        .min(self.max_acceleration);
        Ok(match self.phase_at(time, duration) {
            MotionPhase::Acceleration => self.displacement_direction * acceleration,
            MotionPhase::Cruise => 0.0,
            MotionPhase::Deceleration => -self.displacement_direction * acceleration,
        })
    }

    pub fn validate_time(&self, time: f64) -> Result<(), PlannerError> {
        Self::validate_duration_time(time, self.total_time)
    }

    pub fn calculate_progress_limits(
        start: JointAngles,
        end: JointAngles,
        velocity_limits: &JointVelocityLimits,
        acceleration_limits: &JointAccelerationLimits,
    ) -> Result<(f64, f64), PlannerError> {
        let first_displacement = (start.first_theta() - end.first_theta()).abs();
        let second_displacement = (start.second_theta() - end.second_theta()).abs();

        utilities::validate_finite_positive(first_displacement)
            .map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_finite_positive(second_displacement)
            .map_err(|_| PlannerError::NumericalFailure)?;

        let mut max_progress_velocity = f64::INFINITY;
        let mut max_progress_acceleration = f64::INFINITY;

        if first_displacement > 0.0 {
            max_progress_velocity = max_progress_velocity.min(product_ratio(
                &[velocity_limits.first_joint_max_velocity()],
                &[first_displacement],
            ));
            max_progress_acceleration = max_progress_acceleration.min(product_ratio(
                &[acceleration_limits.first_joint_max_acceleration()],
                &[first_displacement],
            ));
        }

        if second_displacement > 0.0 {
            max_progress_velocity = max_progress_velocity.min(product_ratio(
                &[velocity_limits.second_joint_max_velocity()],
                &[second_displacement],
            ));

            max_progress_acceleration = max_progress_acceleration.min(product_ratio(
                &[acceleration_limits.second_joint_max_acceleration()],
                &[second_displacement],
            ));
        }

        utilities::validate_non_zero_finite_positive(max_progress_velocity)
            .map_err(|_| PlannerError::NumericalFailure)?;
        utilities::validate_non_zero_finite_positive(max_progress_acceleration)
            .map_err(|_| PlannerError::NumericalFailure)?;

        Ok((max_progress_velocity, max_progress_acceleration))
    }

    fn validate_duration_time(time: f64, duration: f64) -> Result<(), PlannerError> {
        utilities::validate_finite_positive(time).map_err(|_| PlannerError::InvalidTime)?;

        if time > duration {
            return Err(PlannerError::InvalidTime);
        }

        Ok(())
    }

    fn scaled_ramp_time(&self, duration: f64) -> f64 {
        product_ratio(&[self.acceleration_time, duration], &[self.total_time])
    }

    fn phase_at(&self, time: f64, duration: f64) -> MotionPhase {
        if time == duration && self.deceleration_time > 0.0 {
            return MotionPhase::Deceleration;
        }
        let ramp = self.scaled_ramp_time(duration);
        if self.acceleration_time > 0.0 && time <= ramp {
            return MotionPhase::Acceleration;
        }
        if self.cruise_time > 0.0
            && time
                <= duration - product_ratio(&[self.deceleration_time, duration], &[self.total_time])
        {
            return MotionPhase::Cruise;
        }
        if self.deceleration_time > 0.0 {
            MotionPhase::Deceleration
        } else if self.cruise_time > 0.0 {
            MotionPhase::Cruise
        } else {
            MotionPhase::Acceleration
        }
    }

    fn ramp_displacement(&self, time: f64, duration: f64, boundary_speed: f64) -> f64 {
        product_ratio(&[boundary_speed, time, self.total_time], &[duration])
            + product_ratio(
                &[
                    self.max_acceleration,
                    time,
                    time,
                    self.total_time,
                    self.total_time,
                ],
                &[2.0, duration, duration],
            )
    }

    fn ramp_velocity(&self, time: f64, duration: f64, boundary_speed: f64) -> f64 {
        product_ratio(&[boundary_speed, self.total_time], &[duration])
            + product_ratio(
                &[
                    self.max_acceleration,
                    time,
                    self.total_time,
                    self.total_time,
                ],
                &[duration, duration],
            )
    }

    fn speed_change_distance(higher: f64, lower: f64, acceleration: f64) -> f64 {
        if higher == lower {
            return 0.0;
        }
        product_ratio(
            &[higher - lower, higher, 1.0 + lower / higher],
            &[2.0, acceleration],
        )
    }
}

#[cfg(test)]
#[path = "../tests/unit/motion_profile.rs"]
mod tests;
