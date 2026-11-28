use kinematics::JointAngles;

#[cfg(feature = "transition-hinted")]
use crate::{JointToStepsConverter, MotorSteps, StepTimeBase};
use crate::{PlannerError, TimedCoordinatedJointPathProfiles, TimedCoordinatedJointProfile};

#[derive(Debug, Clone, Copy)]
pub struct CoordinatedJointSample {
    pub position: JointAngles,
    pub velocity: (f64, f64),
    pub acceleration: (f64, f64),
}

#[derive(Debug, Clone, Copy)]
pub struct CoordinatedJointTrajectory<'a> {
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    end_time: f64,
}

impl<'a> CoordinatedJointTrajectory<'a> {
    pub fn new(profiles: TimedCoordinatedJointPathProfiles<'a>) -> Result<Self, PlannerError> {
        let mut end_time = 0.0;

        for profile in profiles {
            end_time = profile?.end_time()?;
        }

        Ok(Self { profiles, end_time })
    }

    pub fn position_at(&self, global_time: f64) -> Result<JointAngles, PlannerError> {
        self.find_profile(global_time)?.position_at(global_time)
    }

    /// Devuelve la posición final para tiempos finitos posteriores al final.
    pub fn position_at_clamped(&self, global_time: f64) -> Result<JointAngles, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;
        self.position_at(global_time.min(self.end_time))
    }

    pub fn acceleration_at(&self, global_time: f64) -> Result<(f64, f64), PlannerError> {
        self.find_profile(global_time)?.acceleration_at(global_time)
    }

    pub fn velocity_at(&self, global_time: f64) -> Result<(f64, f64), PlannerError> {
        self.find_profile(global_time)?.velocity_at(global_time)
    }

    pub fn sample_at(&self, global_time: f64) -> Result<CoordinatedJointSample, PlannerError> {
        let profile = self.find_profile(global_time)?;
        sample_profile_at(&profile, global_time)
    }

    fn find_profile(&self, global_time: f64) -> Result<TimedCoordinatedJointProfile, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;

        if global_time > self.end_time {
            return Err(PlannerError::InvalidTime);
        }

        for profile in self.profiles {
            let profile = profile?;
            if profile.start_time() <= global_time && profile.end_time()? >= global_time {
                return Ok(profile);
            }
        }

        Err(PlannerError::ProfileInstanceNotFound)
    }
}

/// Muestrea posiciones en cualquier orden y conserva el último perfil consultado.
#[derive(Debug)]
pub struct CachedTrajectoryPositionSampler<'a> {
    trajectory: CoordinatedJointTrajectory<'a>,
    cached_profile: Option<TimedCoordinatedJointProfile>,
    cached_end_time: f64,
}

impl<'a> CachedTrajectoryPositionSampler<'a> {
    pub fn new(trajectory: CoordinatedJointTrajectory<'a>) -> Self {
        Self {
            trajectory,
            cached_profile: None,
            cached_end_time: 0.0,
        }
    }

    #[cfg(feature = "transition-hinted")]
    pub(crate) fn estimate_transition_tick(
        &self,
        start_tick: u64,
        end_tick: u64,
        position: MotorSteps,
        converter: &JointToStepsConverter,
        timebase: StepTimeBase,
    ) -> Option<u64> {
        let profile = self.cached_profile.as_ref()?;
        let start_time = timebase
            .ticks_to_seconds(start_tick)
            .min(self.trajectory.end_time);
        let end_time = timebase
            .ticks_to_seconds(end_tick)
            .min(self.trajectory.end_time);
        if start_time < profile.start_time() || end_time > self.cached_end_time {
            return None;
        }
        profile.estimate_next_step_tick(converter, position, timebase)
    }

    /// Limita los tiempos finitos posteriores al final al último instante de la trayectoria.
    pub fn position_at_clamped(&mut self, global_time: f64) -> Result<JointAngles, PlannerError> {
        utilities::validate_finite_positive(global_time).map_err(|_| PlannerError::InvalidTime)?;
        let time = global_time.min(self.trajectory.end_time);

        // En una frontera compartida, la búsqueda original elige el perfil anterior.
        if let Some(profile) = &self.cached_profile
            && time > profile.start_time()
            && time <= self.cached_end_time
        {
            return profile.position_at(time);
        }

        let profile = self.trajectory.find_profile(time)?;
        let end_time = profile.end_time()?;
        let position = profile.position_at(time)?;
        self.cached_profile = Some(profile);
        self.cached_end_time = end_time;
        Ok(position)
    }
}

fn sample_profile_at(
    profile: &TimedCoordinatedJointProfile,
    global_time: f64,
) -> Result<CoordinatedJointSample, PlannerError> {
    Ok(CoordinatedJointSample {
        position: profile.position_at(global_time)?,
        velocity: profile.velocity_at(global_time)?,
        acceleration: profile.acceleration_at(global_time)?,
    })
}

/// Cursor para consultar una trayectoria con tiempos no decrecientes.
///
/// Mantiene el perfil actual y avanza por los pendientes solo cuando hace
/// falta. Un instante compartido entre dos perfiles pertenece al anterior,
/// salvo si se inicia desde un sufijo cuya primera frontera es ese instante.
///
/// Una consulta fallida no cambia el cursor: se puede reintentar o consultar
/// otro instante que no sea anterior a la ultima consulta exitosa.
#[derive(Debug)]
pub struct CoordinatedJointTrajectoryCursor<'a> {
    profiles: TimedCoordinatedJointPathProfiles<'a>,
    current_profile: Option<TimedCoordinatedJointProfile>,
    end_time: f64,
    last_time: Option<f64>,
}

impl<'a> CoordinatedJointTrajectoryCursor<'a> {
    pub fn new(trajectory: CoordinatedJointTrajectory<'a>) -> Self {
        Self {
            profiles: trajectory.profiles,
            current_profile: None,
            end_time: trajectory.end_time,
            last_time: None,
        }
    }

    pub fn end_time(&self) -> f64 {
        self.end_time
    }

    pub fn sample_at(
        &mut self,
        global_time: f64,
    ) -> Result<CoordinatedJointSample, PlannerError> {
        utilities::validate_finite_positive(global_time)
            .map_err(|_| PlannerError::InvalidTime)?;

        if global_time > self.end_time {
            return Err(PlannerError::InvalidTime);
        }

        if self.last_time.is_some_and(|last_time| global_time < last_time) {
            return Err(PlannerError::TimeWentBackwards);
        }

        if let Some(profile) = self.current_profile.as_ref() {
            if global_time < profile.start_time() {
                return Err(PlannerError::ProfileInstanceNotFound);
            }

            if global_time <= profile.end_time()? {
                let sample = sample_profile_at(profile, global_time)?;
                self.last_time = Some(global_time);
                return Ok(sample);
            }
        }

        let mut remaining = self.profiles;
        while let Some(next_profile) = remaining.next() {
            let profile = next_profile?;

            if global_time < profile.start_time() {
                return Err(PlannerError::ProfileInstanceNotFound);
            }

            if global_time <= profile.end_time()? {
                let sample = sample_profile_at(&profile, global_time)?;
                self.profiles = remaining;
                self.current_profile = Some(profile);
                self.last_time = Some(global_time);
                return Ok(sample);
            }
        }

        Err(PlannerError::ProfileInstanceNotFound)
    }
}

#[cfg(test)]
#[path = "../tests/unit/cached_trajectory_position_sampler.rs"]
mod cached_sampler_tests;
