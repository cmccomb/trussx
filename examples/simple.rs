use trussx::{force, point, Truss};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a simple truss with two joints and one member
    let mut truss = Truss::new();

    // Define joints
    let a = truss.add_joint(point(0.0, 0.0, 0.0));
    let b = truss.add_joint(point(1.0, 0.0, 0.0));

    // Define supports and loads
    truss.set_support(a, [true, true, true])?;
    truss.set_support(b, [false, true, true])?;
    truss.set_load(b, force(-1000.0, 0.0, 0.0))?;

    // Define member properties
    let ab = truss.add_member(a, b);
    truss.set_member_properties(ab, 0.01, 200.0e9)?;

    // Evaluate the truss and unwrap the result
    truss.evaluate()?;

    // Retrieve and print the displacement at joint B
    if let Some(displacement) = truss.joint_displacement(b) {
        println!("ux = {:.3e} m", displacement.x);
    }

    // All done
    Ok(())
}
