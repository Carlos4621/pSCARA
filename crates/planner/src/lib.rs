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