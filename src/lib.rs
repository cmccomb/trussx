enum StructuralShape {
    Pipe,

}

struct Node {
    id: usize,
    x: f64,
    y: f64,
    z: f64,
}

struct Member {
    id: usize,
    beginning_node: usize,
    ending_node: usize,
    cross_section: StructuralShape,
}

struct Load {

}

struct Truss {
    nodes: Vec<Node>,
    members: Vec<Member>,
    loads: Vec<Load>,
}