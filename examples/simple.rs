use nalgebra::Vector3;
use trussx::{point, Truss};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut truss = Truss::new();
    let a = truss.add_joint(point(0.0, 0.0, 0.0));
    let b = truss.add_joint(point(1.0, 0.0, 0.0));
    truss.set_support(a, [true, true, true])?;
    truss.set_support(b, [false, true, true])?;
    truss.set_load(b, Vector3::new(-1000.0, 0.0, 0.0))?;
    let ab = truss.add_member(a, b);
    truss.set_member_properties(ab, 0.01, 200.0e9)?;

    truss.evaluate()?;

    if let Some(displacement) = truss.joint_displacement(b) {
        println!("ux = {:.3e} m", displacement.x);
    }

    Ok(())
}
