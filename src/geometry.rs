use petgraph::graph::{EdgeIndex, NodeIndex};
use trussx::{point, Truss};

/// Describes the topology of the cantilever example used by the CLI.
#[derive(Debug, Clone, Copy)]
pub struct CantileverGeometry {
    /// Index of the fixed joint at the support.
    pub fixed_joint: NodeIndex,
    /// Index of the joint where the load is applied.
    pub loaded_joint: NodeIndex,
    /// Member that connects the fixed and loaded joints.
    pub member: EdgeIndex,
}

/// Build the bare cantilever truss without any boundary conditions.
///
/// The geometry mirrors the textbook single-bar cantilever often used to
/// demonstrate axial deformation. See
/// <https://en.wikipedia.org/wiki/Cantilever> for background.
///
/// # Examples
/// ```
/// use crate::geometry::{build_cantilever_truss, CantileverGeometry};
///
/// let (truss, CantileverGeometry { fixed_joint, loaded_joint, member }) =
///     build_cantilever_truss();
/// assert_eq!(truss.joint_count(), 2);
/// assert_eq!(truss.member_count(), 1);
/// assert_eq!(fixed_joint.index(), 0);
/// assert_eq!(loaded_joint.index(), 1);
/// assert_eq!(member.index(), 0);
/// ```
#[must_use]
pub fn build_cantilever_truss() -> (Truss, CantileverGeometry) {
    let mut truss = Truss::new();

    // Add the support joint at the global origin. This mirrors the typical
    // textbook diagram where the fixed support is drawn on the left-hand side.
    let fixed_joint = truss.add_joint(point(0.0, 0.0, 0.0));

    // Create the free joint one metre away from the support. This distance is
    // arbitrary but keeps the numbers simple so the resulting displacement is
    // easy to verify by hand.
    let loaded_joint = truss.add_joint(point(1.0, 0.0, 0.0));

    // Connect the joints with a single axial member. Because we only have one
    // bar, the internal force path is easy to understand: the load travels
    // directly through this member back to the support.
    let member = truss.add_member(fixed_joint, loaded_joint);

    (
        truss,
        CantileverGeometry {
            fixed_joint,
            loaded_joint,
            member,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_expected_topology() {
        let (truss, geometry) = build_cantilever_truss();
        assert_eq!(truss.joint_count(), 2);
        assert_eq!(truss.member_count(), 1);
        assert_eq!(geometry.fixed_joint.index(), 0);
        assert_eq!(geometry.loaded_joint.index(), 1);
        assert_eq!(geometry.member.index(), 0);
    }
}
