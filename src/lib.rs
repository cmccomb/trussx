//! This package provides utilities for designing and analyzing truss structures

// We use petgraph to make things easier
use petgraph::Graph;

/// This enum contains different structural shapes
pub enum StructuralShape {
    Pipe {
        radius: f64,
        thickness: f64,
    },
    IBeam {
        width: f64,
        height: f64,
        web_thickness: f64,
        flange_thickness: f64,
    },
    BoxBeam {
        width: f64,
        height: f64,
        thickness: f64,
    },
}

struct Joint {
    x: f64,
    y: f64,
    z: f64,
    reaction: Vec<bool>,
    load: Vec<f64>,
}

struct Member {
    cross_section: StructuralShape,
    elastic_modulus: f64,
    yield_strength: f64,
}

/// This is the truss object that contains all of the necessary information about trusses
pub struct Truss {
    graph: petgraph::Graph<Joint, Member>,
}

impl Truss {
    /// This function instantiates an empty truss
    pub fn new() -> Truss {
        Truss {
            graph: petgraph::Graph::new(),
        }
    }

    /// This function creates a new joint
    pub fn add_joint(&mut self, x: f64, y: f64, z: f64) -> petgraph::graph::NodeIndex {
        self.graph.add_node(Joint {
            x,
            y,
            z,
            reaction: vec![false, false, false],
            load: vec![0.0; 3],
        })
    }

    /// This function creates a new member to connect two joints
    pub fn add_edge(
        &mut self,
        a: petgraph::graph::NodeIndex,
        b: petgraph::graph::NodeIndex,
        cross_section: StructuralShape,
        elastic_modulus: f64,
        yield_strength: f64,
    ) -> petgraph::graph::EdgeIndex {
        self.graph.add_edge(
            a,
            b,
            Member {
                cross_section,
                elastic_modulus,
                yield_strength,
            },
        )
    }

    /// This function moves a joint
    pub fn move_joint(&mut self, a: petgraph::graph::NodeIndex) {
        unimplemented!()
    }

    /// This function deletes a joint
    pub fn delete_joint(&mut self, a: petgraph::graph::NodeIndex) {
        unimplemented!()
    }

    /// This function deletes a member
    pub fn delete_member(&mut self, ab: petgraph::graph::EdgeIndex) {
        unimplemented!()
    }

    fn calculate_member_forces(&mut self) {
        unimplemented!()
    }

    fn calculate_member_stress(&mut self) {
        unimplemented!()
    }

    /// This function calculates the forces in each member and outputs a report
    pub fn evaluate(&mut self) {
        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
    use crate::*;
    #[test]
    fn it_works() {
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
}
