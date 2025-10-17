//! Error types produced while editing or analysing trusses.

use petgraph::graph::{EdgeIndex, NodeIndex};
use thiserror::Error;

/// Error returned when a truss analysis fails.
#[derive(Debug, Error, PartialEq)]
pub enum AnalysisError {
    /// Returned when a member is missing required material properties.
    #[error("member {0:?} is missing material properties")]
    MissingProperties(EdgeIndex),
    /// Returned when a member spans zero distance.
    #[error("member {0:?} has zero length")]
    ZeroLengthMember(EdgeIndex),
    /// Returned when the supplied properties for a member are not physically meaningful.
    #[error("member {member:?} has invalid properties: {source}")]
    InvalidMemberProperties {
        /// Identifier of the offending member.
        member: EdgeIndex,
        /// Description of the invalid property.
        #[source]
        source: MemberPropertyError,
    },
    /// Returned when the stiffness matrix cannot be inverted.
    #[error("stiffness matrix is singular; check supports and connectivity")]
    SingularStiffness,
}

/// Error returned when updating material properties for a truss member.
///
/// The variants describe the reason the supplied value is rejected so callers can
/// present actionable feedback to users.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MemberPropertyError {
    /// Returned when the cross-sectional area is zero or negative.
    #[error("area must be positive (received {area})")]
    NonPositiveArea {
        /// Identifier of the affected member.
        member: EdgeIndex,
        /// Rejected cross-sectional area in square metres.
        area: f64,
    },
    /// Returned when the elastic modulus is zero or negative.
    #[error("elastic modulus must be positive (received {elastic_modulus})")]
    NonPositiveElasticModulus {
        /// Identifier of the affected member.
        member: EdgeIndex,
        /// Rejected elastic modulus in pascals.
        elastic_modulus: f64,
    },
}

/// Error returned when editing a [`Truss`](crate::Truss) with invalid indices.
///
/// Attempting to mutate the structure with a joint or member that is not part of the
/// current graph returns a descriptive variant so callers can decide how to recover.
///
/// # Examples
///
/// ```
/// use petgraph::graph::EdgeIndex;
/// use trussx::{Truss, TrussEditError};
///
/// let mut truss = Truss::new();
/// let invalid_member = EdgeIndex::new(42);
/// let error = truss
///     .set_member_properties(invalid_member, 0.01, 200.0e9)
///     .expect_err("unknown member is rejected");
/// assert_eq!(error, TrussEditError::UnknownMember(invalid_member));
/// ```
#[derive(Debug, Error, PartialEq)]
pub enum TrussEditError {
    /// Returned when a joint cannot be found in the truss.
    #[error("joint {0:?} does not exist in this truss")]
    UnknownJoint(NodeIndex),
    /// Returned when a member cannot be found in the truss.
    #[error("member {0:?} does not exist in this truss")]
    UnknownMember(EdgeIndex),
    /// Returned when the supplied member properties are invalid.
    #[error("{0}")]
    InvalidMemberProperties(MemberPropertyError),
}
