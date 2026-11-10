use crate::PlannerError;

enum MotionPhase {
    Acceletration,
    Cruise,
    Desacceleration,
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
    cruise_time: f64,
    total_time: f64,
}

impl MotionProfile {
    pub fn new(
        start_angle: f64,
        end_angle: f64,
        max_acceleration: f64,
        max_velocity: f64,
    ) -> Result<Self, PlannerError> {
        if !start_angle.is_finite() || !end_angle.is_finite() {
            return Err(PlannerError::AngleNonFinite);
        }

        if !max_acceleration.is_finite() || max_acceleration <= 0.0 {
            return Err(PlannerError::InvalidMaxAcceleration);
        }

        if !max_velocity.is_finite() || max_velocity <= 0.0 {
            return Err(PlannerError::InvalidMaxVelocity);
        }

        let critic_distance = max_velocity * max_velocity / max_acceleration;
        let angular_displacement = (end_angle - start_angle).abs();
        let displacement_direction = (end_angle - start_angle).signum();

        let is_triangular = angular_displacement < critic_distance;

        let peak_velocity = if is_triangular {
            libm::sqrt(max_acceleration * angular_displacement)
        } else {
            max_velocity
        };

        if !peak_velocity.is_finite() {
            return Err(PlannerError::NumericalFailure);
        }

        let acceleration_time = peak_velocity / max_acceleration;

        if !acceleration_time.is_finite() {
            return Err(PlannerError::NumericalFailure);
        }

        let cruise_time = if angular_displacement == 0.0 || is_triangular {
            0.0
        } else {
            (angular_displacement - peak_velocity * peak_velocity / max_acceleration)
                / peak_velocity
        };

        if !cruise_time.is_finite() {
            return Err(PlannerError::NumericalFailure);
        }

        let total_time = 2.0 * acceleration_time + cruise_time;

        if !total_time.is_finite() {
            return Err(PlannerError::NumericalFailure);
        }

        Ok(Self {
            start_angle,
            end_angle,
            angular_displacement,
            displacement_direction,
            peak_velocity,
            max_acceleration,
            acceleration_time,
            cruise_time,
            total_time,
        })
    }

    pub fn total_time(&self) -> f64 {
        self.total_time
    }

    pub fn position_at(&self, time: f64) -> Result<f64, PlannerError> {
        self.validate_time(time)?;

        if time == self.total_time {
            return Ok(self.end_angle);
        }

        let angle = match self.get_phase_on(time) {
            MotionPhase::Acceletration => self.position_on_acceleration_phase(time),
            MotionPhase::Cruise => self.position_on_cruise_phase(time),
            MotionPhase::Desacceleration => self.position_on_desacceleration_phase(time),
        };

        Ok(self.start_angle + self.displacement_direction * angle)
    }

    pub fn velocity_at(&self, time: f64) -> Result<f64, PlannerError> {
        self.validate_time(time)?;

        let velocity_magnitude = match self.get_phase_on(time) {
            MotionPhase::Acceletration => self.velocity_on_acceleration_phase(time),
            MotionPhase::Cruise => self.peak_velocity,
            MotionPhase::Desacceleration => self.velocity_on_desacceleration_phase(time),
        };

        Ok(velocity_magnitude * self.displacement_direction)
    }

    pub fn validate_time(&self, time: f64) -> Result<(), PlannerError> {
        if !time.is_finite() || time < 0.0 || time > self.total_time {
            return Err(PlannerError::InvalidTime);
        }

        Ok(())
    }

    fn get_phase_on(&self, time: f64) -> MotionPhase {
        if 0.0 <= time && time <= self.acceleration_time {
            return MotionPhase::Acceletration;
        }

        if self.acceleration_time < time && time <= self.acceleration_time + self.cruise_time {
            return MotionPhase::Cruise;
        }

        MotionPhase::Desacceleration
    }

    fn position_on_acceleration_phase(&self, time: f64) -> f64 {
        self.max_acceleration * time * time / 2.0
    }

    fn position_on_cruise_phase(&self, time: f64) -> f64 {
        self.position_on_acceleration_phase(self.acceleration_time)
            + self.peak_velocity * (time - self.acceleration_time)
    }

    fn position_on_desacceleration_phase(&self, time: f64) -> f64 {
        self.angular_displacement - self.position_on_acceleration_phase(self.total_time - time)
    }

    fn velocity_on_acceleration_phase(&self, time: f64) -> f64 {
        self.max_acceleration * time
    }

    fn velocity_on_desacceleration_phase(&self, time: f64) -> f64 {
        self.velocity_on_acceleration_phase(self.total_time - time)
    }
}
