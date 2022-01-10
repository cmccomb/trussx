#![warn(clippy::all)]
#![warn(missing_docs)]
#![warn(rustdoc::missing_doc_code_examples)]
#![warn(clippy::missing_docs_in_private_items)]
#![doc = include_str!("../README.md")]

use ndarray::Array2;
pub use structural_shapes::StructuralShape;
use uom::si::{
    f64::{Force, Length, Pressure, Ratio},
    force::newton,
    length::{meter, Conversion as LengthConversion, Unit as LengthUnit},
    pressure::{pascal, Conversion as PressureConversion, Unit as PressureUnit},
    ratio::ratio,
};

/// A joint in the truss
#[derive(Clone, Copy, Debug)]
struct Joint {
    /// The position of the joint
    position: [Length; 3],
    /// The reactions applied to the joint
    reaction: [bool; 3],
    /// The loads applied to the joint
    load: [Force; 3],
    /// The deflections at
    deflection: [Length; 3],
}

impl Default for Joint {
    fn default() -> Joint {
        Joint {
            position: [Length::new::<meter>(0.0); 3],
            reaction: [false, false, false],
            load: [Force::new::<newton>(0.0); 3],
            deflection: [Length::new::<meter>(0.0); 3],
        }
    }
}

/// A member in the truss
#[derive(Clone, Copy, Debug)]
struct Member {
    /// The cross-sectional shape
    cross_section: StructuralShape,
    /// The elastic modulus
    elastic_modulus: Pressure,
    /// The yield strength
    yield_strength: Pressure,
    /// The force in the member
    force: Force,
    /// The stress in the member
    stress: Pressure,
    /// The factor of safety for the member
    fos: Ratio,
}

impl Default for Member {
    fn default() -> Member {
        Member {
            cross_section: StructuralShape::Pipe {
                outer_radius: Length::new::<meter>(0.0),
                thickness: Length::new::<meter>(0.0),
                center_of_gravity: (Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
            },
            elastic_modulus: Pressure::new::<pascal>(0.0),
            yield_strength: Pressure::new::<pascal>(0.0),
            force: Force::new::<newton>(0.0),
            stress: Pressure::new::<pascal>(0.0),
            fos: Ratio::new::<ratio>(0.0),
        }
    }
}

/// This is the truss object that contains all of the necessary information about trusses
#[derive(Clone, Debug)]
pub struct Truss {
    /// A graph structure containing most of the information about the truss
    graph: petgraph::Graph<Joint, Member>,
    /// A bool indicating whether or not results are current
    results: bool,
}

impl Default for Truss {
    fn default() -> Truss {
        Truss::new()
    }
}

impl Truss {
    /// This function instantiates an empty truss
    pub fn new() -> Truss {
        Truss {
            graph: petgraph::Graph::new(),
            results: false,
        }
    }

    /// This function creates a new joint
    pub fn add_joint<T: LengthUnit + LengthConversion<f64>>(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
    ) -> petgraph::graph::NodeIndex {
        self.clear();
        self.graph.add_node(Joint {
            position: [
                Length::new::<T>(x),
                Length::new::<T>(y),
                Length::new::<T>(z),
            ],
            ..Joint::default()
        })
    }

    /// This function creates a new member to connect two joints
    pub fn add_edge(
        &mut self,
        a: petgraph::graph::NodeIndex,
        b: petgraph::graph::NodeIndex,
    ) -> petgraph::graph::EdgeIndex {
        self.clear();
        self.graph.add_edge(a, b, Member::default())
    }

    /// This function moves a joint
    pub fn move_joint<T: LengthUnit + LengthConversion<f64>>(
        &mut self,
        a: petgraph::graph::NodeIndex,
        x: f64,
        y: f64,
        z: f64,
    ) {
        self.clear();
        let joint = self.graph.node_weight_mut(a);
        match joint {
            None => {
                panic!("This joint does not exist");
            }
            Some(joint) => {
                joint.position = [
                    Length::new::<T>(x),
                    Length::new::<T>(y),
                    Length::new::<T>(z),
                ];
            }
        }
    }

    /// This function deletes a joint
    pub fn delete_joint(&mut self, a: petgraph::graph::NodeIndex) {
        self.clear();
        self.graph.remove_node(a);
    }

    /// This function deletes a member
    pub fn delete_member(&mut self, ab: petgraph::graph::EdgeIndex) {
        self.clear();
        self.graph.remove_edge(ab);
    }

    /// Set reaction forces available at a joint
    pub fn set_reactions(&mut self, a: petgraph::graph::NodeIndex, reaction: [bool; 3]) {
        self.clear();
        let joint = self.graph.node_weight_mut(a);
        match joint {
            None => {
                panic!("This joint does not exist");
            }
            Some(joint) => {
                joint.reaction = reaction;
            }
        }
    }

    /// Set loads at a joint
    pub fn set_loads(&mut self, a: petgraph::graph::NodeIndex, load: [Force; 3]) {
        self.clear();
        let joint = self.graph.node_weight_mut(a);
        match joint {
            None => {
                panic!("This joint does not exist");
            }
            Some(joint) => {
                joint.load = load;
            }
        }
    }

    /// Set material for a member
    pub fn set_material<T: PressureUnit + PressureConversion<f64>>(
        &mut self,
        ab: petgraph::graph::EdgeIndex,
        elastic_modulus: f64,
        yield_strength: f64,
    ) {
        self.clear();
        let member = self.graph.edge_weight_mut(ab);
        match member {
            None => {
                panic!("This joint does not exist");
            }
            Some(member) => {
                member.elastic_modulus = Pressure::new::<T>(elastic_modulus);
                member.yield_strength = Pressure::new::<T>(yield_strength);
            }
        }
    }

    /// Set shape for a member
    pub fn set_shape(&mut self, ab: petgraph::graph::EdgeIndex, shape: StructuralShape) {
        self.clear();
        let member = self.graph.edge_weight_mut(ab);
        match member {
            None => {
                panic!("This joint does not exist");
            }
            Some(member) => {
                member.cross_section = shape;
            }
        }
    }

    /// Set material for all members
    pub fn set_material_for_all<T: PressureUnit + PressureConversion<f64>>(
        &mut self,
        elastic_modulus: f64,
        yield_strength: f64,
    ) {
        self.clear();
        for mut member in self.graph.edge_weights_mut() {
            member.elastic_modulus = Pressure::new::<T>(elastic_modulus);
            member.yield_strength = Pressure::new::<T>(yield_strength);
        }
    }

    /// Set material for all members
    pub fn set_shape_for_all(&mut self, shape: StructuralShape) {
        self.clear();
        for mut member in self.graph.edge_weights_mut() {
            member.cross_section = shape;
        }
    }

    /// Clear results after a change
    fn clear(&mut self) {
        if self.results {
            self.results = false;
            for mut member in self.graph.edge_weights_mut() {
                member.force = Force::new::<newton>(0.0);
                member.stress = Pressure::new::<pascal>(0.0);
            }

            for mut joint in self.graph.node_weights_mut() {
                joint.deflection = [Length::new::<meter>(0.0); 3];
            }
        }
    }

    /// Calculate forces in the members
    fn calculate_member_forces(&mut self) {
        let n = self.graph.node_count();

        // Initialize some stuff
        let _stiffness_matrix = Array2::<f64>::zeros((n * 3, n * 3));
        let mut deflections = Array2::<bool>::from_elem((3, n), false);
        let mut loads = Array2::<Force>::zeros((3, n));

        // Fill some stuff up
        for (idx, member) in self.graph.node_indices().enumerate() {
            let node_reactions = self.graph.node_weight_mut(member).unwrap().reaction;
            let node_loads = self.graph.node_weight_mut(member).unwrap().load;
            for jdx in 1..3 {
                deflections[[jdx, idx]] = !node_reactions[jdx];
                loads[[jdx, idx]] = node_loads[jdx];
            }
        }

        // Find out which joints can deflect
        let mut ff: Vec<usize> = vec![0; 0];
        let mut ff_load: Vec<Force> = vec![Force::new::<newton>(0.0); 0];
        let mut counter: usize = 0;
        for i in 0..n {
            for j in 0..3 {
                if deflections[[j, i]] {
                    ff.push(counter);
                    ff_load.push(loads[[j, i]])
                }
                counter += 1;
            }
        }

        // Build the global stiffess matrix
        // int idx1, idx2, key1, key2;
        // long double ux, uy, uz;
        // std::vector<int> ee(6, 0);
        // std::vector<long double> uu(6, 0.0);
        // for (std::map<int, Edge>::iterator it = edges.begin(); it != edges.end(); it++) {
        //     int k = (it->first);
        //     key1 = edges[k].initial_node;
        //     key2 = edges[k].terminal_node;
        //     idx1 = node_id_map[key1];
        //     idx2 = node_id_map[key2];
        //     ux = (nodes[key1].parameters["x"] - nodes[key2].parameters["x"]) / edges[k].parameters["L"];
        //     uy = (nodes[key1].parameters["y"] - nodes[key2].parameters["y"]) / edges[k].parameters["L"];
        //     uz = (nodes[key1].parameters["z"] - nodes[key2].parameters["z"]) / edges[k].parameters["L"];
        //     long double EAL = E * edges[k].parameters["A"] / edges[k].parameters["L"];
        //     edges[k].parameters["kx"] = EAL*ux;
        //     edges[k].parameters["ky"] = EAL*uy;
        //     edges[k].parameters["kz"] = EAL*uz;
        //     uu = {ux, uy, uz, -ux, -uy, -uz};
        //     ee = {3 * idx1, 3 * idx1 + 1, 3 * idx1 + 2, 3 * idx2, 3 * idx2 + 1, 3 * idx2 + 2};
        //     for (int i = 0; i < 6; i++) {
        //         for (int j = 0; j < 6; j++) {
        //             K[ee[i]][ee[j]] += EAL * uu[i] * uu[j];
        //         }
        //     }
        // }

        // Solve for displacements
        // int ffs = static_cast<int>(ff.size());
        // std::vector<std::vector<long double> > Kff(ff.size(), std::vector<long double>(ff.size() + 1, 0.0));
        // for (int i = 0; i < ffs; i++) {
        //     for (int j = 0; j < ffs; j++) {
        //         Kff[i][j] = K[ff[i]][ff[j]];
        //     }
        //     Kff[i][ffs] = loads_ff[i];
        // }
        //
        // std::vector<long double> deflections_compact = gauss(Kff);
        //
        // // Compute the condition number
        // for(int i=0; i<ffs; i++){
        //     Kff[i][ffs] = deflections_compact[i];
        // }
        // std::vector<long double> backed_out_loads = matrix_vector_mult(Kff);
        // cond = 0;
        // for(int i=0; i<ffs; i++){
        //     cond += std::abs(loads_ff[i] - backed_out_loads[i]);
        // }
        //
        //
        // // Fit the compacted deflection matrix back into the original
        // counter = 0;
        // for (int i = 0; i < number_of_nodes; i++) {
        //     for (int j = 0; j < 3; j++) {
        //         if (deflections[j][i] == 1) {
        //             deflections[j][i] = deflections_compact[counter];
        //             counter++;
        //         }
        //     }
        // }
        //
        // // From displacements, solve for forces
        // for (std::map<int, Edge>::iterator it = edges.begin(); it != edges.end(); it++) {
        //     // Define a few things
        //     int k = (it->first);
        //     idx1 = node_id_map[edges[k].initial_node];
        //     idx2 = node_id_map[edges[k].terminal_node];
        //
        //     // Define the force
        //     edges[k].parameters["F"] =   edges[k].parameters["kx"] * (deflections[0][idx1] - deflections[0][idx2])
        //                                + edges[k].parameters["ky"] * (deflections[1][idx1] - deflections[1][idx2])
        //                                + edges[k].parameters["kz"] * (deflections[2][idx1] - deflections[2][idx2]);
        //
        //     // Calculate factor of safety against yielding
        //     edges[k].parameters["FOS_y"] = std::abs((Fy*edges[k].parameters["A"])/edges[k].parameters["F"]);
        //
        //     // Calculate factor of safety against buckling
        //     if (edges[k].parameters["F"] < 0) {
        //         edges[k].parameters["FOS_b"] = -(std::pow(M_PI, 2) * E * edges[k].parameters["I"]/std::pow(edges[k].parameters["L"], 2))/edges[k].parameters["F"];
        //     } else {
        //         edges[k].parameters["FOS_b"] = 1000;
        //     }
        //
        //     // Save the limiting factor of safety
        //     if(edges[k].parameters["FOS_b"] < edges[k].parameters["FOS_y"]){
        //         edges[k].parameters["FOS_lim"] = edges[k].parameters["FOS_b"];
        //     } else {
        //         edges[k].parameters["FOS_lim"] = edges[k].parameters["FOS_y"];
        //     }
        // }
    }

    /// Calculate member stresses from forces
    fn calculate_member_stress(&mut self) {
        for mut member in self.graph.edge_weights_mut() {
            member.stress = member.force / member.cross_section.area();
            member.fos = member.yield_strength / member.stress;
        }
    }

    /// This function calculates the forces in each member and outputs a report
    pub fn evaluate(&mut self) {
        self.results = true;
        self.calculate_member_forces();
        self.calculate_member_stress();
    }

    /// Get FOS for all members in truss
    pub fn get_fos_tuple(&mut self) -> Vec<(petgraph::graph::EdgeIndex, Ratio)> {
        let mut fos = vec![];
        for member in self.graph.edge_indices() {
            fos.push((member, self.graph.edge_weight(member).unwrap().fos.abs()));
        }
        fos
    }

    /// Get FOS for all members in truss
    pub fn get_fos(&mut self) -> Vec<Ratio> {
        let mut fos = vec![];
        for member in self.graph.edge_indices() {
            fos.push(self.graph.edge_weight(member).unwrap().fos.abs());
        }
        fos
    }

    /// Find the member with minimum fos
    pub fn min_fos_member(&mut self) -> petgraph::graph::EdgeIndex {
        let mut fos: Ratio;
        let mut min_fos = Ratio::new::<ratio>(f64::INFINITY);
        let mut min_fos_member = petgraph::graph::EdgeIndex::default();
        for member in self.graph.edge_indices() {
            fos = self.graph.edge_weight(member).unwrap().fos.abs();
            if fos < min_fos {
                min_fos_member = member;
                min_fos = fos;
            }
        }
        min_fos_member
    }

    /// Find the member with maximum stress
    pub fn max_stress_member(&mut self) -> petgraph::graph::EdgeIndex {
        let mut stress: Pressure;
        let mut max_stress = Pressure::new::<pascal>(0.0);
        let mut max_stress_member = petgraph::graph::EdgeIndex::default();
        for member in self.graph.edge_indices() {
            stress = self.graph.edge_weight(member).unwrap().stress.abs();
            if stress > max_stress {
                max_stress_member = member;
                max_stress = stress;
            }
        }
        max_stress_member
    }
}

#[cfg(test)]
mod tests {
    use crate::*;
    use uom::si::length::meter;
    use uom::si::pressure::gigapascal;

    #[test]
    fn it_works() {
        let mut x = Truss::new();
        let a = x.add_joint::<meter>(0.0, 0.0, 0.0);
        let b = x.add_joint::<meter>(3.0, 0.0, 0.0);
        let c = x.add_joint::<meter>(1.5, 1.5, 0.0);
        let _ab = x.add_edge(a, b);
        let _bc = x.add_edge(b, c);
        let _ac = x.add_edge(a, c);
        x.set_material_for_all::<gigapascal>(125.0, 25.0);
    }
}
