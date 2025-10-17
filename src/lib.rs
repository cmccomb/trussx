#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

//! Tools for building and analysing simple pin-jointed truss structures.

mod errors;
mod geometry;
mod truss;

pub use errors::{AnalysisError, MemberPropertyError, TrussEditError};
pub use geometry::{displacement, force, point, Displacement, Force, Point};
pub use truss::Truss;
