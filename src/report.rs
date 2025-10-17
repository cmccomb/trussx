use crate::analysis::AnalysisSummary;
use std::fmt::Write;

/// Render a textual summary of the cantilever analysis.
///
/// The formatted report walks through the key numbers so developers who are new
/// to structural analysis can cross-check the output with references such as
/// <https://en.wikipedia.org/wiki/Young%27s_modulus>.
#[must_use]
pub fn render_summary(summary: &AnalysisSummary) -> String {
    let mut output = String::new();

    // Explain the problem statement first so the reader knows what the numbers
    // represent before we dive into them.
    writeln!(
        &mut output,
        "Cantilever bar analysis (axial load = {:.1} N)",
        summary.properties.axial_load
    )
    .expect("writing to string cannot fail");

    // Report the displacement. Keeping scientific notation mirrors what
    // engineers expect from hand calculations.
    writeln!(
        &mut output,
        "Displacement at free end: ux = {:+.3e} m, uy = {:+.3e} m, uz = {:+.3e} m",
        summary.displacement.x, summary.displacement.y, summary.displacement.z
    )
    .expect("writing to string cannot fail");

    // Provide the internal force and stress so readers can verify equilibrium
    // (sum of forces) and compatibility (strain = displacement / length).
    writeln!(
        &mut output,
        "Member response: axial force = {:+.1} N, stress = {:+.3e} Pa",
        summary.axial_force, summary.axial_stress
    )
    .expect("writing to string cannot fail");

    // The factor of safety gives a simple go/no-go check. We mention that it
    // uses absolute values so the sign convention does not confuse readers.
    if let Some(fos) = summary.factor_of_safety {
        writeln!(
            &mut output,
            "Factor of safety (|yield| / |stress|): {fos:.1}"
        )
        .expect("writing to string cannot fail");
    } else {
        output.push_str("Factor of safety: not available (missing yield strength)\n");
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::AnalysisSummary;
    use crate::conditions::CantileverProperties;
    use trussx::Displacement;

    #[test]
    fn formats_human_readable_report() {
        let summary = AnalysisSummary {
            displacement: Displacement::new(-5.0e-7, 0.0, 0.0),
            axial_force: -1_000.0,
            axial_stress: -1.0e5,
            factor_of_safety: Some(2_500.0),
            properties: CantileverProperties::default(),
        };
        let report = render_summary(&summary);
        assert!(report.contains("Cantilever bar analysis"));
        assert!(report.contains("ux = -5.000e-7 m"));
        assert!(report.contains("2500.0"));
    }
}
