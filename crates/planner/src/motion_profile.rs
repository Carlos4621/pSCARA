use crate::{
    PlannerError,
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
    peak_velocity: f64,
    max_acceleration: f64,
    acceleration_time: f64,
    total_time: f64,
}

impl MotionProfile {
    pub fn new(
        start_angle: f64,
        end_angle: f64,
        max_acceleration: f64,
        max_velocity: f64,
    ) -> Result<Self, PlannerError> {
        utilities::validate_finite(start_angle).map_err(|_| PlannerError::AngleNonFinite)?;
        utilities::validate_finite(end_angle).map_err(|_| PlannerError::AngleNonFinite)?;

        utilities::validate_non_zero_finite_positive(max_acceleration)
            .map_err(|_| PlannerError::InvalidMaxAcceleration)?;
        utilities::validate_non_zero_finite_positive(max_velocity)
            .map_err(|_| PlannerError::InvalidMaxVelocity)?;

        let delta = end_angle - start_angle;
        utilities::validate_finite(delta).map_err(|_| PlannerError::NumericalFailure)?;
        let angular_displacement = delta.abs();
        let displacement_direction = delta.signum();

        let (peak_velocity, acceleration_time, total_time) = if angular_displacement == 0.0 {
            (0.0, 0.0, 0.0)
        } else {
            let unconstrained_peak =
                sqrt_product_ratio(&[max_acceleration, angular_displacement], &[]);
            let triangular = unconstrained_peak <= max_velocity;
            let peak = unconstrained_peak.min(max_velocity);
            let ramp = if triangular {
                sqrt_product_ratio(&[angular_displacement], &[max_acceleration])
            } else {
                max_velocity / max_acceleration
            };
            let duration = if triangular {
                2.0 * ramp
            } else {
                angular_displacement / max_velocity + ramp
            };
            utilities::validate_non_zero_finite_positive(duration)
                .map_err(|_| PlannerError::NumericalFailure)?;
            (peak, ramp, duration)
        };

        Ok(Self {
            start_angle,
            end_angle,
            angular_displacement,
            displacement_direction,
            peak_velocity,
            max_acceleration,
            acceleration_time,
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
            MotionPhase::Acceleration => self.ramp_displacement(time, duration),
            MotionPhase::Cruise => {
                let ramp_distance = self.ramp_displacement(self.acceleration_time, self.total_time);
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
                self.angular_displacement - self.ramp_displacement(duration - time, duration)
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

        if time == 0.0 || time == duration || self.angular_displacement == 0.0 {
            return Ok(0.0);
        }

        // Synchronization only stretches time, so rounding must not raise a limit.
        let peak = product_ratio(&[self.peak_velocity, self.total_time], &[duration])
            .min(self.peak_velocity);
        let velocity_magnitude = match self.phase_at(time, duration) {
            MotionPhase::Acceleration => self.ramp_velocity(time, duration),
            MotionPhase::Cruise => peak,
            MotionPhase::Deceleration => self.ramp_velocity(duration - time, duration),
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
        if time == duration {
            return MotionPhase::Deceleration;
        }
        let ramp = self.scaled_ramp_time(duration);
        if time <= ramp {
            return MotionPhase::Acceleration;
        }
        if time <= duration - ramp {
            return MotionPhase::Cruise;
        }

        MotionPhase::Deceleration
    }

    fn ramp_displacement(&self, time: f64, duration: f64) -> f64 {
        product_ratio(
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

    fn ramp_velocity(&self, time: f64, duration: f64) -> f64 {
        product_ratio(
            &[
                self.max_acceleration,
                time,
                self.total_time,
                self.total_time,
            ],
            &[duration, duration],
        )
    }
}
