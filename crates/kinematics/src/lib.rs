#![no_std]

mod point;
pub use point::Point;

mod kinematics_error;
pub use kinematics_error::KinematicsError;

mod scara;
pub use scara::{Scara, ElbowConfiguration, JointAngles};
