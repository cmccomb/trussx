#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

//! Tools for building and analysing simple pin-jointed truss structures.

use std::collections::HashMap;

use nalgebra::{DMatrix, DVector, SMatrix, Vector3};
use petgraph::graph::{EdgeIndex, Graph, NodeIndex};
use thiserror::Error;

/// Three dimensional vector of `f64` values.
pub type Point3 = Vector3<f64>;

/// Error returned when a truss analysis fails.
#[derive(Debug, Error, PartialEq)]
pub enum AnalysisError {
    /// Returned when a member is missing required material properties.
    #[error("member {0:?} is missing material properties")]
    MissingProperties(EdgeIndex),
    /// Returned when a member spans zero distance.
    #[error("member {0:?} has zero length")]
    ZeroLengthMember(EdgeIndex),
    /// Returned when the stiffness matrix cannot be inverted.
    #[error("stiffness matrix is singular; check supports and connectivity")]
    SingularStiffness,
}

#[derive(Clone, Debug)]
struct Joint {
    /// Position of the joint in metres.
    position: Point3,
    /// True if the corresponding translational degree of freedom is restrained.
    support: [bool; 3],
    /// External load applied to the joint in newtons.
    load: Vector3<f64>,
    /// Resulting displacement after analysis in metres.
    displacement: Vector3<f64>,
}

impl Joint {
    /// Create a new joint at the supplied position.
    fn new(position: Point3) -> Self {
        Self {
            position,
            support: [false, false, false],
            load: Vector3::zeros(),
            displacement: Vector3::zeros(),
        }
    }
}

/// Internal representation of a truss member.
#[derive(Clone, Debug)]
struct Member {
    /// Cross-sectional area in square metres.
    area: Option<f64>,
    /// Elastic modulus in pascals.
    elastic_modulus: Option<f64>,
    /// Optional yield strength in pascals.
    yield_strength: Option<f64>,
    /// Axial force after analysis in newtons.
    axial_force: f64,
    /// Axial stress after analysis in pascals.
    stress: f64,
    /// Optional factor of safety against yielding.
    factor_of_safety: Option<f64>,
}

impl Member {
    /// Create a member with no assigned properties.
    fn new() -> Self {
        Self {
            area: None,
            elastic_modulus: None,
            yield_strength: None,
            axial_force: 0.0,
            stress: 0.0,
            factor_of_safety: None,
        }
    }

    /// Return the area and elastic modulus when both are available.
    fn properties(&self) -> Option<(f64, f64)> {
        Some((self.area?, self.elastic_modulus?))
    }
}

/// Container for a pin-jointed truss model.
#[derive(Debug, Default)]
pub struct Truss {
    /// Underlying storage for joints and members.
    graph: Graph<Joint, Member>,
    /// Indicates whether the cached analysis data is current.
    analysis_valid: bool,
}

impl Truss {
    /// Create an empty truss.
    ///
    /// # Examples
    /// ```
    /// use trussx::Truss;
    ///
    /// let truss = Truss::new();
    /// assert_eq!(truss.joint_count(), 0);
    /// ```
    #[must_use]
    pub fn new() -> Self {
        Self {
            graph: Graph::new(),
            analysis_valid: false,
        }
    }

    /// Return the number of joints in the truss.
    #[must_use]
    pub fn joint_count(&self) -> usize {
        self.graph.node_count()
    }

    /// Return the number of members in the truss.
    #[must_use]
    pub fn member_count(&self) -> usize {
        self.graph.edge_count()
    }

    /// Add a new joint to the truss.
    ///
    /// # Examples
    /// ```
    /// use nalgebra::Vector3;
    /// use trussx::Truss;
    ///
    /// let mut truss = Truss::new();
    /// let joint = truss.add_joint(Vector3::new(0.0, 0.0, 0.0));
    /// assert_eq!(truss.joint_count(), 1);
    /// assert_eq!(joint.index(), 0);
    /// ```
    pub fn add_joint(&mut self, position: Point3) -> NodeIndex {
        self.invalidate();
        self.graph.add_node(Joint::new(position))
    }

    /// Update the position of an existing joint.
    pub fn move_joint(&mut self, joint: NodeIndex, position: Point3) {
        self.invalidate();
        if let Some(node) = self.graph.node_weight_mut(joint) {
            node.position = position;
        }
    }

    /// Remove a joint and all connected members from the truss.
    pub fn remove_joint(&mut self, joint: NodeIndex) {
        self.invalidate();
        self.graph.remove_node(joint);
    }

    /// Connect two joints with a new member.
    pub fn add_member(&mut self, start: NodeIndex, end: NodeIndex) -> EdgeIndex {
        self.invalidate();
        self.graph.add_edge(start, end, Member::new())
    }

    /// Remove a member from the truss.
    pub fn remove_member(&mut self, member: EdgeIndex) {
        self.invalidate();
        self.graph.remove_edge(member);
    }

    /// Set the restraint state for a joint.
    ///
    /// Each entry in `support` corresponds to the X, Y and Z directions respectively. A
    /// value of `true` indicates that the degree of freedom is fixed.
    pub fn set_support(&mut self, joint: NodeIndex, support: [bool; 3]) {
        self.invalidate();
        if let Some(node) = self.graph.node_weight_mut(joint) {
            node.support = support;
        }
    }

    /// Apply a point load to a joint.
    pub fn set_load(&mut self, joint: NodeIndex, load: Vector3<f64>) {
        self.invalidate();
        if let Some(node) = self.graph.node_weight_mut(joint) {
            node.load = load;
        }
    }

    /// Set the axial properties for a member.
    pub fn set_member_properties(&mut self, member: EdgeIndex, area: f64, elastic_modulus: f64) {
        self.invalidate();
        if let Some(edge) = self.graph.edge_weight_mut(member) {
            edge.area = Some(area);
            edge.elastic_modulus = Some(elastic_modulus);
        }
    }

    /// Set the yield strength for a member.
    pub fn set_member_yield_strength(&mut self, member: EdgeIndex, yield_strength: f64) {
        self.invalidate();
        if let Some(edge) = self.graph.edge_weight_mut(member) {
            edge.yield_strength = Some(yield_strength);
        }
    }

    /// Reset stored analysis results.
    fn invalidate(&mut self) {
        if self.analysis_valid {
            for joint in self.graph.node_weights_mut() {
                joint.displacement = Vector3::zeros();
            }
            for member in self.graph.edge_weights_mut() {
                member.axial_force = 0.0;
                member.stress = 0.0;
                member.factor_of_safety = None;
            }
            self.analysis_valid = false;
        }
    }

    /// Construct a mapping from graph node indices to contiguous identifiers.
    fn node_index_map(&self) -> HashMap<NodeIndex, usize> {
        self.graph
            .node_indices()
            .enumerate()
            .map(|(idx, node)| (node, idx))
            .collect()
    }

    /// Assemble the global stiffness matrix for the current configuration.
    fn build_stiffness_matrix(
        &self,
        index_map: &HashMap<NodeIndex, usize>,
    ) -> Result<DMatrix<f64>, AnalysisError> {
        let dof = self.joint_count() * 3;
        let mut matrix = DMatrix::zeros(dof, dof);
        for edge in self.graph.edge_indices() {
            let (start, end) = self.graph.edge_endpoints(edge).expect("valid edge");
            let start_joint = &self.graph[start];
            let end_joint = &self.graph[end];
            let properties = self.graph[edge]
                .properties()
                .ok_or(AnalysisError::MissingProperties(edge))?;
            let (area, elastic_modulus) = properties;
            let delta = end_joint.position - start_joint.position;
            let length = delta.norm();
            if length == 0.0 {
                return Err(AnalysisError::ZeroLengthMember(edge));
            }
            let direction = delta / length;
            let ea_over_l = elastic_modulus * area / length;

            let l = direction[0];
            let m = direction[1];
            let n = direction[2];

            let local: SMatrix<f64, 6, 6> = ea_over_l
                * SMatrix::<f64, 6, 6>::from_row_slice(&[
                    l * l,
                    l * m,
                    l * n,
                    -l * l,
                    -l * m,
                    -l * n,
                    l * m,
                    m * m,
                    m * n,
                    -l * m,
                    -m * m,
                    -m * n,
                    l * n,
                    m * n,
                    n * n,
                    -l * n,
                    -m * n,
                    -n * n,
                    -l * l,
                    -l * m,
                    -l * n,
                    l * l,
                    l * m,
                    l * n,
                    -l * m,
                    -m * m,
                    -m * n,
                    l * m,
                    m * m,
                    m * n,
                    -l * n,
                    -m * n,
                    -n * n,
                    l * n,
                    m * n,
                    n * n,
                ]);

            let start_idx = index_map[&start] * 3;
            let end_idx = index_map[&end] * 3;
            let dof_map = [
                start_idx,
                start_idx + 1,
                start_idx + 2,
                end_idx,
                end_idx + 1,
                end_idx + 2,
            ];

            for (row_local, global_row) in dof_map.iter().enumerate() {
                for (col_local, global_col) in dof_map.iter().enumerate() {
                    matrix[(*global_row, *global_col)] += local[(row_local, col_local)];
                }
            }
        }
        Ok(matrix)
    }

    /// Assemble the nodal load vector.
    fn build_load_vector(&self, index_map: &HashMap<NodeIndex, usize>) -> DVector<f64> {
        let dof = self.joint_count() * 3;
        let mut load = DVector::zeros(dof);
        for node in self.graph.node_indices() {
            let joint = &self.graph[node];
            let base = index_map[&node] * 3;
            load[base] = joint.load.x;
            load[base + 1] = joint.load.y;
            load[base + 2] = joint.load.z;
        }
        load
    }

    /// Determine the free degrees of freedom in the structure.
    fn collect_free_dofs(&self, index_map: &HashMap<NodeIndex, usize>) -> Vec<usize> {
        let mut free = Vec::new();
        for node in self.graph.node_indices() {
            let joint = &self.graph[node];
            let base = index_map[&node] * 3;
            for axis in 0..3 {
                if !joint.support[axis] {
                    free.push(base + axis);
                }
            }
        }
        free
    }

    /// Solve for the nodal displacement vector.
    fn solve_displacements(
        stiffness: &DMatrix<f64>,
        load: &DVector<f64>,
        free_dofs: &[usize],
    ) -> Result<DVector<f64>, AnalysisError> {
        let mut displacements = DVector::zeros(load.len());
        let free_len = free_dofs.len();
        if free_len == 0 {
            return Ok(displacements);
        }
        let mut k_ff = DMatrix::zeros(free_len, free_len);
        let mut f_f = DVector::zeros(free_len);
        for (row_idx, &row) in free_dofs.iter().enumerate() {
            f_f[row_idx] = load[row];
            for (col_idx, &col) in free_dofs.iter().enumerate() {
                k_ff[(row_idx, col_idx)] = stiffness[(row, col)];
            }
        }
        let solution = k_ff
            .lu()
            .solve(&f_f)
            .ok_or(AnalysisError::SingularStiffness)?;
        for (idx, &dof) in free_dofs.iter().enumerate() {
            displacements[dof] = solution[idx];
        }
        Ok(displacements)
    }

    /// Persist nodal displacements back to the graph.
    fn store_joint_displacements(
        &mut self,
        index_map: &HashMap<NodeIndex, usize>,
        displacements: &DVector<f64>,
    ) {
        for node in self.graph.node_indices() {
            let joint = self.graph.node_weight_mut(node).expect("valid node");
            let base = index_map[&node] * 3;
            joint.displacement.x = displacements[base];
            joint.displacement.y = displacements[base + 1];
            joint.displacement.z = displacements[base + 2];
        }
    }

    /// Update member forces and stresses using the solved displacements.
    fn update_member_forces(
        &mut self,
        index_map: &HashMap<NodeIndex, usize>,
        displacements: &DVector<f64>,
    ) {
        for edge in self.graph.edge_indices() {
            let (start, end) = self.graph.edge_endpoints(edge).expect("valid edge");
            let start_joint = &self.graph[start];
            let end_joint = &self.graph[end];
            let properties = {
                let member = self.graph.edge_weight(edge).expect("valid member");
                (member.area, member.elastic_modulus, member.yield_strength)
            };
            let (Some(area), Some(elastic_modulus), yield_strength) = properties else {
                if let Some(member) = self.graph.edge_weight_mut(edge) {
                    member.axial_force = 0.0;
                    member.stress = 0.0;
                    member.factor_of_safety = None;
                }
                continue;
            };
            let delta = end_joint.position - start_joint.position;
            let length = delta.norm();
            if length == 0.0 {
                if let Some(member) = self.graph.edge_weight_mut(edge) {
                    member.axial_force = 0.0;
                    member.stress = 0.0;
                    member.factor_of_safety = None;
                }
                continue;
            }
            let direction = delta / length;
            let start_idx = index_map[&start] * 3;
            let end_idx = index_map[&end] * 3;
            let start_disp = Vector3::new(
                displacements[start_idx],
                displacements[start_idx + 1],
                displacements[start_idx + 2],
            );
            let end_disp = Vector3::new(
                displacements[end_idx],
                displacements[end_idx + 1],
                displacements[end_idx + 2],
            );
            let axial_extension = direction.dot(&(end_disp - start_disp));
            let axial_force = elastic_modulus * area * axial_extension / length;
            let stress = axial_force / area;
            if let Some(member) = self.graph.edge_weight_mut(edge) {
                member.axial_force = axial_force;
                member.stress = stress;
                member.factor_of_safety = yield_strength
                    .and_then(|yield_strength| {
                        if stress.abs() > f64::EPSILON {
                            Some(yield_strength.abs() / stress.abs())
                        } else {
                            None
                        }
                    })
                    .filter(|value| value.is_finite());
            }
        }
    }

    /// Run a linear elastic analysis of the truss.
    ///
    /// # Errors
    ///
    /// Returns an error when material properties are missing, a member has zero length,
    /// or the stiffness matrix is singular because the structure is unstable.
    pub fn evaluate(&mut self) -> Result<(), AnalysisError> {
        let index_map = self.node_index_map();
        let stiffness = self.build_stiffness_matrix(&index_map)?;
        let load = self.build_load_vector(&index_map);
        let free_dofs = self.collect_free_dofs(&index_map);
        let displacements = Self::solve_displacements(&stiffness, &load, &free_dofs)?;
        self.store_joint_displacements(&index_map, &displacements);
        self.update_member_forces(&index_map, &displacements);
        self.analysis_valid = true;
        Ok(())
    }

    /// Retrieve the displacement vector for a joint.
    #[must_use]
    pub fn joint_displacement(&self, joint: NodeIndex) -> Option<Vector3<f64>> {
        self.graph
            .node_weight(joint)
            .map(|joint| joint.displacement)
    }

    /// Retrieve the axial force for a member.
    #[must_use]
    pub fn member_axial_force(&self, member: EdgeIndex) -> Option<f64> {
        self.graph
            .edge_weight(member)
            .map(|member| member.axial_force)
    }

    /// Retrieve the axial stress for a member.
    #[must_use]
    pub fn member_stress(&self, member: EdgeIndex) -> Option<f64> {
        self.graph.edge_weight(member).map(|member| member.stress)
    }

    /// Retrieve the factor of safety for a member, if available.
    #[must_use]
    pub fn member_factor_of_safety(&self, member: EdgeIndex) -> Option<f64> {
        self.graph
            .edge_weight(member)
            .and_then(|member| member.factor_of_safety)
    }
}

/// Convenience helper for creating position vectors.
///
/// # Examples
/// ```
/// use trussx::point;
///
/// let origin = point(0.0, 0.0, 0.0);
/// assert_eq!(origin.x, 0.0);
/// ```
#[must_use]
pub fn point(x: f64, y: f64, z: f64) -> Point3 {
    Vector3::new(x, y, z)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn single_bar_in_tension() {
        let mut truss = Truss::new();
        let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
        let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
        truss.set_support(joint_a, [true, true, true]);
        truss.set_support(joint_b, [false, true, true]);
        truss.set_load(joint_b, Vector3::new(-1000.0, 0.0, 0.0));
        let member_ab = truss.add_member(joint_a, joint_b);
        truss.set_member_properties(member_ab, 0.01, 200.0e9);
        truss.set_member_yield_strength(member_ab, 250.0e6);

        truss.evaluate().expect("analysis succeeds");

        let displacement = truss
            .joint_displacement(joint_b)
            .expect("joint displacement available");
        assert_relative_eq!(displacement.x, -5.0e-7, epsilon = 1.0e-10);
        let force = truss
            .member_axial_force(member_ab)
            .expect("member force available");
        assert_relative_eq!(force, -1000.0, epsilon = 1.0e-6);
        let stress = truss.member_stress(member_ab).expect("member stress");
        assert_relative_eq!(stress, -100_000.0, epsilon = 1.0e-3);
        let fos = truss
            .member_factor_of_safety(member_ab)
            .expect("factor of safety computed");
        assert_relative_eq!(fos, 2500.0, epsilon = 1.0e-6);
    }

    #[test]
    fn missing_properties_causes_error() {
        let mut truss = Truss::new();
        let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
        let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
        truss.set_support(joint_a, [true, true, true]);
        truss.set_support(joint_b, [false, true, true]);
        truss.set_load(joint_b, Vector3::new(-1000.0, 0.0, 0.0));
        let member_ab = truss.add_member(joint_a, joint_b);
        let error = truss.evaluate().expect_err("missing properties");
        assert_eq!(error, AnalysisError::MissingProperties(member_ab));
    }

    #[test]
    fn triangular_truss_matches_reference_solution() {
        let mut truss = Truss::new();
        let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
        let joint_b = truss.add_joint(point(6.0, 0.0, 0.0));
        let joint_c = truss.add_joint(point(3.0, 4.0, 0.0));

        truss.set_support(joint_a, [true, true, true]);
        truss.set_support(joint_b, [false, true, true]);
        truss.set_support(joint_c, [false, false, true]);
        truss.set_load(joint_c, Vector3::new(0.0, -10_000.0, 0.0));

        let bottom_member = truss.add_member(joint_a, joint_b);
        let left_diagonal = truss.add_member(joint_a, joint_c);
        let right_diagonal = truss.add_member(joint_b, joint_c);

        for member in [bottom_member, left_diagonal, right_diagonal] {
            truss.set_member_properties(member, 0.003, 200.0e9);
        }

        truss.evaluate().expect("analysis succeeds");

        let displacement = truss
            .joint_displacement(joint_c)
            .expect("joint displacement available");
        assert_relative_eq!(displacement.x, 1.875e-5, epsilon = 1.0e-12);
        assert_relative_eq!(displacement.y, -7.916_666_666_666_665e-5, epsilon = 1.0e-12);

        let bottom_axial = truss
            .member_axial_force(bottom_member)
            .expect("member force available");
        assert_relative_eq!(bottom_axial, 3750.0, epsilon = 1.0e-6);
        let left_axial = truss
            .member_axial_force(left_diagonal)
            .expect("member force available");
        assert_relative_eq!(left_axial, -6250.0, epsilon = 1.0e-6);
        let right_axial = truss
            .member_axial_force(right_diagonal)
            .expect("member force available");
        assert_relative_eq!(right_axial, -6250.0, epsilon = 1.0e-6);
    }

    #[test]
    fn three_panel_warren_truss_distributes_roof_load() {
        let mut truss = Truss::new();
        let left_support = truss.add_joint(point(0.0, 0.0, 0.0));
        let lower_panel_b = truss.add_joint(point(5.0, 0.0, 0.0));
        let lower_panel_c = truss.add_joint(point(10.0, 0.0, 0.0));
        let right_support = truss.add_joint(point(15.0, 0.0, 0.0));
        let left_apex = truss.add_joint(point(2.5, 3.0, 0.0));
        let mid_apex = truss.add_joint(point(7.5, 3.0, 0.0));
        let right_apex = truss.add_joint(point(12.5, 3.0, 0.0));

        truss.set_support(left_support, [true, true, true]);
        truss.set_support(right_support, [false, true, true]);
        for joint in [
            lower_panel_b,
            lower_panel_c,
            left_apex,
            mid_apex,
            right_apex,
        ] {
            truss.set_support(joint, [false, false, true]);
        }

        let roof_load = Vector3::new(0.0, -15_000.0, 0.0);
        for joint in [left_apex, mid_apex, right_apex] {
            truss.set_load(joint, roof_load);
        }

        let bottom_left_panel = truss.add_member(left_support, lower_panel_b);
        let bottom_middle_panel = truss.add_member(lower_panel_b, lower_panel_c);
        let bottom_right_panel = truss.add_member(lower_panel_c, right_support);
        let top_left_panel = truss.add_member(left_apex, mid_apex);
        let top_right_panel = truss.add_member(mid_apex, right_apex);
        let left_raker = truss.add_member(left_support, left_apex);
        let left_post = truss.add_member(left_apex, lower_panel_b);
        let left_diagonal = truss.add_member(lower_panel_b, mid_apex);
        let right_diagonal = truss.add_member(mid_apex, lower_panel_c);
        let right_post = truss.add_member(lower_panel_c, right_apex);
        let right_raker = truss.add_member(right_apex, right_support);

        for member in [
            bottom_left_panel,
            bottom_middle_panel,
            bottom_right_panel,
            top_left_panel,
            top_right_panel,
            left_raker,
            left_post,
            left_diagonal,
            right_diagonal,
            right_post,
            right_raker,
        ] {
            truss.set_member_properties(member, 0.004, 210.0e9);
        }

        truss.evaluate().expect("analysis succeeds");

        let displacement = truss
            .joint_displacement(mid_apex)
            .expect("midspan deflection available");
        assert_relative_eq!(displacement.x, 2.046_130_952_380_952e-4, epsilon = 1.0e-12);
        assert_relative_eq!(displacement.y, -8.689_392_548_551_007e-4, epsilon = 1.0e-12);

        let bf_force = truss
            .member_axial_force(left_diagonal)
            .expect("diagonal force available");
        assert_relative_eq!(bf_force, -9_762.812_094_883_302, epsilon = 1.0e-9);
        let fc_force = truss
            .member_axial_force(right_diagonal)
            .expect("diagonal force available");
        assert_relative_eq!(fc_force, -9_762.812_094_883_333, epsilon = 1.0e-9);
        let ef_force = truss
            .member_axial_force(top_left_panel)
            .expect("top chord force available");
        assert_relative_eq!(ef_force, -25_000.0, epsilon = 1.0e-6);
    }
}
