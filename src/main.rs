mod analysis;
mod conditions;
mod geometry;
mod report;

use analysis::run_analysis;
use conditions::apply_cantilever_conditions;
use geometry::build_cantilever_truss;
use report::render_summary;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Build the bare truss geometry before we think about loads or materials.
    // A truss is a structure made from straight members connected at joints.
    // See: https://en.wikipedia.org/wiki/Truss
    let (mut truss, geometry) = build_cantilever_truss();

    // Apply the boundary conditions (supports) and the external load. These
    // steps reflect static equilibrium requirements described in
    // https://en.wikipedia.org/wiki/Statics.
    apply_cantilever_conditions(&mut truss, &geometry)?;

    // Run the finite element analysis for the configured truss and collect a
    // lightweight summary of the results. The method relies on linear
    // elasticity as described in https://en.wikipedia.org/wiki/Hooke%27s_law.
    let summary = run_analysis(&mut truss, &geometry)?;

    // Render a human-friendly report that explains the structural response and
    // print it to standard output for the CLI user.
    let report = render_summary(&summary);
    println!("{report}");

    Ok(())
}
