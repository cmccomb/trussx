use crate::geometry::CantileverGeometry;
use trussx::{force, Truss, TrussEditError};

/// Physical properties used for the cantilever demonstration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CantileverProperties {
    /// Cross-sectional area in square metres.
    pub area: f64,
    /// Elastic modulus in pascals.
    pub elastic_modulus: f64,
    /// Yield strength in pascals used for the factor of safety.
    pub yield_strength: f64,
    /// Axial load in newtons applied at the free end.
    pub axial_load: f64,
}

impl Default for CantileverProperties {
    fn default() -> Self {
        Self {
            area: 0.01,
            elastic_modulus: 200.0e9,
            yield_strength: 250.0e6,
            axial_load: -1_000.0,
        }
    }
}

/// Apply supports, loads and material properties to the cantilever example.
///
/// Each modification is broken out explicitly so readers can map the code onto
/// the physical description of axial bars from
/// <https://en.wikipedia.org/wiki/Truss#Analysis>.
pub fn apply_cantilever_conditions(
    truss: &mut Truss,
    geometry: &CantileverGeometry,
) -> Result<CantileverProperties, TrussEditError> {
    let properties = CantileverProperties::default();

    // Fix all translational degrees of freedom at the left-hand joint to model a
    // rigid support. This mirrors a clamped boundary condition where no
    // displacement is permitted (see
    // https://en.wikipedia.org/wiki/Boundary_value_problem).
    truss.set_support(geometry.fixed_joint, [true, true, true])?;

    // Restrain vertical and out-of-plane movement at the free joint while
    // leaving the axial direction unrestrained. This converts the structure into
    // a simple cantilever where only the x-translation carries the load.
    truss.set_support(geometry.loaded_joint, [false, true, true])?;

    // Apply the external axial load. The negative sign indicates the force is
    // pulling to the left, matching the direction of the fixed support.
    truss.set_load(
        geometry.loaded_joint,
        force(properties.axial_load, 0.0, 0.0),
    )?;

    // Assign material and geometric properties for the single member. The area
    // and elastic modulus correspond to Hooke's law (stress = E * strain) and
    // directly influence the stiffness of the bar.
    truss.set_member_properties(geometry.member, properties.area, properties.elastic_modulus)?;

    // Provide a yield strength so we can compute a factor of safety after the
    // analysis completes. Using absolute values aligns with engineering practice
    // for axial members (see https://en.wikipedia.org/wiki/Factor_of_safety).
    truss.set_member_yield_strength(geometry.member, properties.yield_strength)?;

    Ok(properties)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::build_cantilever_truss;

    #[test]
    fn applies_expected_conditions() {
        let (mut truss, geometry) = build_cantilever_truss();
        let properties =
            apply_cantilever_conditions(&mut truss, &geometry).expect("conditions apply cleanly");
        assert_eq!(properties, CantileverProperties::default());
        // Run the analysis to ensure the configuration is mechanically stable.
        truss.evaluate().expect("analysis succeeds");
        let displacement = truss
            .joint_displacement(geometry.loaded_joint)
            .expect("displacement available");
        assert!(displacement.x.is_sign_negative());
    }
}
