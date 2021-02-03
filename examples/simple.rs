use trussx::{StructuralShape, Truss};

fn main() {
    let mut x = Truss::new();
    let a = x.add_joint(0.0, 0.0, 0.0);
    let b = x.add_joint(3.0, 0.0, 0.0);
    let ab = x.add_edge(
        a,
        b,
        StructuralShape::Pipe {
            radius: 1.0,
            thickness: 0.1,
        },
        0.0,
        0.0,
    );
}
