use trussx::{Truss, point};

fn main() {
    let mut x = Truss::new();
    let a = x.add_joint(point(0.0, 0.0, 0.0));
    let b = x.add_joint(point(3.0, 0.0, 0.0));
    let _ab = x.add_edge(a, b);
}
