use crate::conditions::CantileverProperties;
use crate::geometry::CantileverGeometry;
use trussx::{AnalysisError, Displacement, Truss};

/// Summary of the results from the cantilever analysis.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalysisSummary {
    /// Displacement of the loaded joint.
    pub displacement: Displacement,
    /// Axial force in the single member.
    pub axial_force: f64,
    /// Axial stress in the member.
    pub axial_stress: f64,
    /// Factor of safety computed from the stored yield strength.
    pub factor_of_safety: Option<f64>,
    /// Properties that generated the result, exposed for reporting.
    pub properties: CantileverProperties,
}

/// Execute the linear analysis for the cantilever demonstration and extract key
/// response metrics.
///
/// The truss is solved using the stiffness method described at
/// <https://en.wikipedia.org/wiki/Direct_stiffness_method>.
pub fn run_analysis(
    truss: &mut Truss,
    geometry: &CantileverGeometry,
) -> Result<AnalysisSummary, AnalysisError> {
    truss.evaluate()?;

    let displacement = truss
        .joint_displacement(geometry.loaded_joint)
        .expect("displacement computed during evaluation");
    let axial_force = truss
        .member_axial_force(geometry.member)
        .expect("axial force computed during evaluation");
    let axial_stress = truss
        .member_stress(geometry.member)
        .expect("stress computed during evaluation");
    let factor_of_safety = truss.member_factor_of_safety(geometry.member);

    Ok(AnalysisSummary {
        displacement,
        axial_force,
        axial_stress,
        factor_of_safety,
        properties: CantileverProperties::default(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conditions::apply_cantilever_conditions;
    use crate::geometry::build_cantilever_truss;

    #[test]
    fn produces_expected_results() {
        let (mut truss, geometry) = build_cantilever_truss();
        apply_cantilever_conditions(&mut truss, &geometry).expect("valid setup");
        let summary = run_analysis(&mut truss, &geometry).expect("analysis runs");

        // The closed-form solution for an axially loaded bar is FL/AE.
        let expected_displacement = -5.0e-7;
        assert!((summary.displacement.x - expected_displacement).abs() < 1.0e-12);
        assert!(summary.displacement.y.abs() < f64::EPSILON);
        assert!(summary.displacement.z.abs() < f64::EPSILON);

        // Axial stress equals force divided by area.
        let expected_force = -1_000.0;
        let expected_stress = expected_force / summary.properties.area;
        assert!((summary.axial_force - expected_force).abs() < 1.0e-9);
        assert!((summary.axial_stress - expected_stress).abs() < 1.0e-6);

        let fos = summary
            .factor_of_safety
            .expect("factor of safety available");
        assert!((fos - 2_500.0).abs() < 1.0e-6);
    }
}
