use trussx::{force, point, Truss};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a simple truss with two joints and one member
    let mut truss = Truss::new();

    // Define joints
    let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
    let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));

    // Define supports and loads
    truss.set_support(joint_a, [true, true, true])?;
    truss.set_support(joint_b, [false, true, true])?;
    truss.set_load(joint_b, force(-5_000.0, 0.0, 0.0))?;

    // Define member properties
    let member_ab = truss.add_member(joint_a, joint_b);
    truss.set_member_properties(member_ab, 0.005, 210.0e9)?;
    truss.set_member_yield_strength(member_ab, 250.0e6)?;

    // Evaluate the truss and unwrap the result
    truss.evaluate()?;

    // Retrieve and print the factor of safety for member AB
    if let Some(factor_of_safety) = truss.member_factor_of_safety(member_ab) {
        println!("Member AB factor of safety: {factor_of_safety:.2}");
    } else {
        println!("Factor of safety is unavailable for member AB.");
    }

    // All done
    Ok(())
}
