use trussx::{StructuralShape, Truss, point};

fn main() {
    let mut x = Truss::new();
    let a = x.add_joint(point(0.0, 0.0, 0.0));
    let b = x.add_joint(point(3.0, 0.0, 0.0));
    let ab = x.add_edge(a, b);
}
