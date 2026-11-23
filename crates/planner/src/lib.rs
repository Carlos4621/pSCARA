#![no_std]

mod profile_math;

mod planner_error;
pub use planner_error::PlannerError;

mod joint_trajectory;
pub use joint_trajectory::JointTrajectory;

mod joint_range;
pub use joint_range::JointRange;

mod joint_limits;
pub use joint_limits::JointLimits;

mod validated_trajectory;
pub use validated_trajectory::ValidatedTrajectory;

mod joint_velocity_limits;
pub use joint_velocity_limits::JointVelocityLimits;

mod timed_joint_trajectory;
pub use timed_joint_trajectory::{TimedJointSample, TimedJointTrajectory};

mod validated_timed_trajectory;
pub use validated_timed_trajectory::ValidatedTimedTrajectory;

mod joint_acceleration_limits;
pub use joint_acceleration_limits::JointAccelerationLimits;

mod motion_profile;
pub use motion_profile::MotionProfile;

mod joint_motion_profile;
pub use joint_motion_profile::JointMotionProfile;

mod coordinated_joint_motion_profile;
pub use coordinated_joint_motion_profile::CoordinatedJointMotionProfile;

mod joint_junction;
pub use joint_junction::calculate_max_junction_progress_speeds;

mod joint_path_velocity_planner;
pub use joint_path_velocity_planner::plan_joint_path_speeds;

mod coordinated_joint_path_profiles;
pub use coordinated_joint_path_profiles::CoordinatedJointPathProfiles;

mod timed_coordinated_joint_profile;
pub use timed_coordinated_joint_profile::TimedCoordinatedJointProfile;

mod timed_coordinated_joint_path_profiles;
pub use timed_coordinated_joint_path_profiles::TimedCoordinatedJointPathProfiles;

mod coordinated_joint_trajectory;
pub use coordinated_joint_trajectory::{
    CoordinatedJointSample, CoordinatedJointTrajectory, CoordinatedJointTrajectoryCursor,
};

mod joint_to_steps_converter;
pub use joint_to_steps_converter::{JointToStepsConverter, MotorSteps};

mod motor_step_move;
pub use motor_step_move::{AxisStepMove, MotorStepMove, StepDirection};

mod step_dir_event;
pub use step_dir_event::{StepDirEvent, into_step_dir_events};

mod step_dir_pin_command;
pub use step_dir_pin_command::{StepDirPinCommand, StepDirState};

mod step_dir_timing;
pub use step_dir_timing::StepDirTiming;

mod step_dir_interval_validator;
pub use step_dir_interval_validator::validate_step_dir_intervals;

mod step_dir_change_validator;
pub use step_dir_change_validator::validate_step_dir_changes;

mod coordinated_step_interpolator;
pub use coordinated_step_interpolator::{CoordinatedStepInterpolator, StepEvent};

mod step_time_base;
pub use step_time_base::StepTimeBase;

mod motor_position_sampler;
pub use motor_position_sampler::MotorPositionSampler;

mod motor_step_interval;
pub use motor_step_interval::MotorStepInterval;

mod timed_step_interpolator;
pub use timed_step_interpolator::{TimedStepEvent, TimedStepInterpolator};

mod step_transition_detector;
pub use step_transition_detector::StepTransitionDetector;

mod transition_finder;
pub use transition_finder::find_first_transition_tick;

mod trajectory_step_event_iterator;
pub use trajectory_step_event_iterator::TrajectoryStepEventIterator;

mod segmented_trajectory_step_event_iterator;
pub use segmented_trajectory_step_event_iterator::SegmentedTrajectoryStepEventIterator;

mod segment_boundaries;
pub use segment_boundaries::{generate_guarded_segment_boundaries, generate_segment_boundaries};

mod segmented_step_events_builder;
pub use segmented_step_events_builder::build_segmented_step_events;

mod build_trajectory_step_events;
pub use build_trajectory_step_events::{
    build_interval_verified_step_dir_events, build_trajectory_step_events,
    build_verified_trajectory_step_events, preflight_trajectory_step_events,
};
