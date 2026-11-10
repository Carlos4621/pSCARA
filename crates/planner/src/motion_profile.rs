use crate::PlannerError;

#[derive(Debug)]
pub struct MotionProfile {
    start_angle: f64,
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

        let peak_velocity = if angular_displacement < critic_distance {
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

        let cruise_time = if angular_displacement > 0.0 {
            (angular_displacement - peak_velocity * peak_velocity / max_acceleration)
                / peak_velocity
        } else {
            0.0
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
        if !time.is_finite() || time < 0.0 || time > self.total_time {
            return Err(PlannerError::InvalidTime);
        }

        let angle = if 0.0 <= time && time <= self.acceleration_time{
            self.calcule_acceleration_phase(time)
        } 
        else if self.acceleration_time < time && time <= self.acceleration_time + self.cruise_time {
            self.calcule_cruise_phase(time)
        }
        else {
            self.calcule_desacceleration_phase(time)
        };

        Ok(self.start_angle + self.displacement_direction * angle)
    }

    fn calcule_acceleration_phase(&self, time: f64) -> f64 {
        self.max_acceleration * time * time / 2.0
    }

    fn calcule_cruise_phase(&self, time: f64) -> f64 {
        self.calcule_acceleration_phase(self.acceleration_time) + self.peak_velocity * (time - self.acceleration_time)
    }

    fn calcule_desacceleration_phase(&self, time: f64) -> f64 {
        self.angular_displacement - self.calcule_acceleration_phase(self.total_time - time)
    }
}
