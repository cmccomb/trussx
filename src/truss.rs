//! Core data structures and algorithms for truss analysis.

use std::collections::HashMap;

use nalgebra::{DMatrix, DVector, SMatrix};
use petgraph::graph::{EdgeIndex, Graph, NodeIndex};

use crate::errors::{AnalysisError, MemberPropertyError, TrussEditError};
use crate::geometry::{Displacement, Force, Point};

/// Internal representation of a truss joint.
#[derive(Clone, Debug)]
struct Joint {
    /// Position of the joint in metres.
    position: Point,
    /// Indicator for each translational degree of freedom that is restrained.
    support: [bool; 3],
    /// External load applied to the joint in newtons.
    load: Force,
    /// Solved displacement vector for the joint in metres.
    displacement: Displacement,
}

impl Joint {
    /// Create a joint with the supplied position and default state.
    fn new(position: Point) -> Self {
        Self {
            position,
            support: [false, false, false],
            load: Force::default(),
            displacement: Displacement::default(),
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

    /// Return the area and elastic modulus when both are present.
    fn properties(&self) -> Option<(f64, f64)> {
        Some((self.area?, self.elastic_modulus?))
    }
}

/// Container for a pin-jointed truss model.
#[derive(Debug, Default)]
pub struct Truss {
    /// Underlying graph storage for joints and members.
    graph: Graph<Joint, Member>,
    /// Indicates whether the cached analysis results are current.
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
    /// use trussx::{point, Truss};
    ///
    /// let mut truss = Truss::new();
    /// let joint = truss.add_joint(point(0.0, 0.0, 0.0));
    /// assert_eq!(truss.joint_count(), 1);
    /// assert_eq!(joint.index(), 0);
    /// ```
    pub fn add_joint(&mut self, position: Point) -> NodeIndex {
        self.invalidate();
        self.graph.add_node(Joint::new(position))
    }

    /// Update the position of an existing joint.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownJoint`] when `joint` is not part of this truss.
    pub fn move_joint(&mut self, joint: NodeIndex, position: Point) -> Result<(), TrussEditError> {
        if self.graph.node_weight(joint).is_none() {
            return Err(TrussEditError::UnknownJoint(joint));
        }
        self.invalidate();
        if let Some(node) = self.graph.node_weight_mut(joint) {
            node.position = position;
            Ok(())
        } else {
            Err(TrussEditError::UnknownJoint(joint))
        }
    }

    /// Remove a joint and all connected members from the truss.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownJoint`] when `joint` is not part of this truss.
    pub fn remove_joint(&mut self, joint: NodeIndex) -> Result<(), TrussEditError> {
        if self.graph.node_weight(joint).is_none() {
            return Err(TrussEditError::UnknownJoint(joint));
        }
        self.invalidate();
        if self.graph.remove_node(joint).is_some() {
            Ok(())
        } else {
            Err(TrussEditError::UnknownJoint(joint))
        }
    }

    /// Connect two joints with a new member.
    pub fn add_member(&mut self, start: NodeIndex, end: NodeIndex) -> EdgeIndex {
        self.invalidate();
        self.graph.add_edge(start, end, Member::new())
    }

    /// Remove a member from the truss.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownMember`] when `member` is not part of this truss.
    pub fn remove_member(&mut self, member: EdgeIndex) -> Result<(), TrussEditError> {
        if self.graph.edge_weight(member).is_none() {
            return Err(TrussEditError::UnknownMember(member));
        }
        self.invalidate();
        if self.graph.remove_edge(member).is_some() {
            Ok(())
        } else {
            Err(TrussEditError::UnknownMember(member))
        }
    }

    /// Set the restraint state for a joint.
    ///
    /// Each entry in `support` corresponds to the X, Y and Z directions respectively. A
    /// value of `true` indicates that the degree of freedom is fixed.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownJoint`] when `joint` is not part of this truss.
    pub fn set_support(
        &mut self,
        joint: NodeIndex,
        support: [bool; 3],
    ) -> Result<(), TrussEditError> {
        if self.graph.node_weight(joint).is_none() {
            return Err(TrussEditError::UnknownJoint(joint));
        }
        self.invalidate();
        if let Some(node) = self.graph.node_weight_mut(joint) {
            node.support = support;
            Ok(())
        } else {
            Err(TrussEditError::UnknownJoint(joint))
        }
    }

    /// Apply a point load to a joint.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownJoint`] when `joint` is not part of this truss.
    pub fn set_load(&mut self, joint: NodeIndex, load: Force) -> Result<(), TrussEditError> {
        if self.graph.node_weight(joint).is_none() {
            return Err(TrussEditError::UnknownJoint(joint));
        }
        self.invalidate();
        if let Some(node) = self.graph.node_weight_mut(joint) {
            node.load = load;
            Ok(())
        } else {
            Err(TrussEditError::UnknownJoint(joint))
        }
    }

    /// Set the axial properties for a member.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownMember`] when `member` is not part of this truss and
    /// [`TrussEditError::InvalidMemberProperties`] when either `area` or `elastic_modulus`
    /// is not strictly positive.
    ///
    /// # Examples
    /// ```
    /// use trussx::{point, Truss, TrussEditError};
    ///
    /// let mut truss = Truss::new();
    /// let a = truss.add_joint(point(0.0, 0.0, 0.0));
    /// let b = truss.add_joint(point(1.0, 0.0, 0.0));
    /// let member = truss.add_member(a, b);
    ///
    /// let error = truss
    ///     .set_member_properties(member, 0.0, 200.0e9)
    ///     .expect_err("invalid area rejected");
    /// match error {
    ///     TrussEditError::InvalidMemberProperties(_) => (),
    ///     other => panic!("unexpected error: {other:?}"),
    /// }
    /// ```
    pub fn set_member_properties(
        &mut self,
        member: EdgeIndex,
        area: f64,
        elastic_modulus: f64,
    ) -> Result<(), TrussEditError> {
        if self.graph.edge_weight(member).is_none() {
            return Err(TrussEditError::UnknownMember(member));
        }
        if area <= 0.0 {
            return Err(TrussEditError::InvalidMemberProperties(
                MemberPropertyError::NonPositiveArea { member, area },
            ));
        }
        if elastic_modulus <= 0.0 {
            return Err(TrussEditError::InvalidMemberProperties(
                MemberPropertyError::NonPositiveElasticModulus {
                    member,
                    elastic_modulus,
                },
            ));
        }
        self.invalidate();
        if let Some(edge) = self.graph.edge_weight_mut(member) {
            edge.area = Some(area);
            edge.elastic_modulus = Some(elastic_modulus);
        }
        Ok(())
    }

    /// Assign a yield strength to a member in pascals.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownMember`] when `member` is not part of this truss.
    pub fn set_member_yield_strength(
        &mut self,
        member: EdgeIndex,
        yield_strength: f64,
    ) -> Result<(), TrussEditError> {
        if self.graph.edge_weight(member).is_none() {
            return Err(TrussEditError::UnknownMember(member));
        }
        self.invalidate();
        if let Some(edge) = self.graph.edge_weight_mut(member) {
            edge.yield_strength = Some(yield_strength);
        }
        Ok(())
    }

    /// Retrieve the displacement of a joint after analysis.
    #[must_use]
    pub fn joint_displacement(&self, joint: NodeIndex) -> Option<Displacement> {
        self.graph
            .node_weight(joint)
            .map(|joint| joint.displacement)
    }

    /// Retrieve the axial force in a member after analysis.
    #[must_use]
    pub fn member_axial_force(&self, member: EdgeIndex) -> Option<f64> {
        self.graph
            .edge_weight(member)
            .map(|member| member.axial_force)
    }

    /// Retrieve the axial stress in a member after analysis.
    #[must_use]
    pub fn member_stress(&self, member: EdgeIndex) -> Option<f64> {
        self.graph.edge_weight(member).map(|member| member.stress)
    }

    /// Retrieve the factor of safety against yielding for a member.
    #[must_use]
    pub fn member_factor_of_safety(&self, member: EdgeIndex) -> Option<f64> {
        self.graph
            .edge_weight(member)
            .and_then(|member| member.factor_of_safety)
    }

    /// Analyse the truss under the configured loads.
    ///
    /// # Errors
    ///
    /// Returns [`AnalysisError`] when the structure cannot be solved because of
    /// invalid member properties or insufficient constraints.
    pub fn evaluate(&mut self) -> Result<(), AnalysisError> {
        if self.analysis_valid {
            return Ok(());
        }
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

    /// Reset cached analysis results when the topology or properties change.
    fn invalidate(&mut self) {
        if self.analysis_valid {
            for joint in self.graph.node_weights_mut() {
                joint.displacement = Displacement::default();
            }
            for member in self.graph.edge_weights_mut() {
                member.axial_force = 0.0;
                member.stress = 0.0;
                member.factor_of_safety = None;
            }
            self.analysis_valid = false;
        }
    }

    /// Construct a mapping from graph indices to contiguous degree-of-freedom indices.
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
            if area <= 0.0 {
                return Err(AnalysisError::InvalidMemberProperties {
                    member: edge,
                    source: MemberPropertyError::NonPositiveArea { member: edge, area },
                });
            }
            if elastic_modulus <= 0.0 {
                return Err(AnalysisError::InvalidMemberProperties {
                    member: edge,
                    source: MemberPropertyError::NonPositiveElasticModulus {
                        member: edge,
                        elastic_modulus,
                    },
                });
            }
            let delta = end_joint.position.to_vector() - start_joint.position.to_vector();
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

    /// Assemble the global nodal load vector.
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

    /// Determine the indices corresponding to unconstrained degrees of freedom.
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

    /// Solve for joint displacements using the reduced stiffness matrix.
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

    /// Persist solved joint displacements back to the graph representation.
    fn store_joint_displacements(
        &mut self,
        index_map: &HashMap<NodeIndex, usize>,
        displacements: &DVector<f64>,
    ) {
        for node in self.graph.node_indices() {
            let joint = self.graph.node_weight_mut(node).expect("valid node");
            let base = index_map[&node] * 3;
            joint.displacement = Displacement::new(
                displacements[base],
                displacements[base + 1],
                displacements[base + 2],
            );
        }
    }

    /// Compute member axial forces, stresses and factors of safety.
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
            let delta = end_joint.position.to_vector() - start_joint.position.to_vector();
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
            let start_disp = Displacement::new(
                displacements[start_idx],
                displacements[start_idx + 1],
                displacements[start_idx + 2],
            );
            let end_disp = Displacement::new(
                displacements[end_idx],
                displacements[end_idx + 1],
                displacements[end_idx + 2],
            );
            let relative = end_disp.to_vector() - start_disp.to_vector();
            let axial_displacement = direction.dot(&relative);
            let axial_force = elastic_modulus * area / length * axial_displacement;
            let stress = axial_force / area;

            if let Some(member) = self.graph.edge_weight_mut(edge) {
                member.axial_force = axial_force;
                member.stress = stress;
                member.factor_of_safety = yield_strength.map(|yield_strength| {
                    if stress == 0.0 {
                        f64::INFINITY
                    } else {
                        yield_strength / stress.abs()
                    }
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::geometry::{force, point};

    #[test]
    fn joint_mutators_return_error_for_unknown_indices() {
        let mut truss = Truss::new();
        let stale_joint = truss.add_joint(point(0.0, 0.0, 0.0));
        truss
            .remove_joint(stale_joint)
            .expect("initial joint removal succeeds");

        let mut other = Truss::new();
        let foreign_joint = other.add_joint(point(1.0, 0.0, 0.0));

        for joint in [stale_joint, foreign_joint] {
            let move_error = truss
                .move_joint(joint, point(2.0, 0.0, 0.0))
                .expect_err("unknown joint rejected");
            assert_eq!(move_error, TrussEditError::UnknownJoint(joint));

            let support_error = truss
                .set_support(joint, [true, false, false])
                .expect_err("unknown joint rejected");
            assert_eq!(support_error, TrussEditError::UnknownJoint(joint));

            let load_error = truss
                .set_load(joint, force(0.0, 0.0, 0.0))
                .expect_err("unknown joint rejected");
            assert_eq!(load_error, TrussEditError::UnknownJoint(joint));
        }

        let stale_remove_error = truss
            .remove_joint(stale_joint)
            .expect_err("stale joint rejected");
        assert_eq!(
            stale_remove_error,
            TrussEditError::UnknownJoint(stale_joint)
        );

        let foreign_remove_error = truss
            .remove_joint(foreign_joint)
            .expect_err("foreign joint rejected");
        assert_eq!(
            foreign_remove_error,
            TrussEditError::UnknownJoint(foreign_joint)
        );
    }

    #[test]
    fn member_mutators_return_error_for_unknown_indices() {
        let mut truss = Truss::new();
        let a = truss.add_joint(point(0.0, 0.0, 0.0));
        let b = truss.add_joint(point(1.0, 0.0, 0.0));
        let stale_member = truss.add_member(a, b);
        truss
            .remove_member(stale_member)
            .expect("initial member removal succeeds");

        let mut other = Truss::new();
        let start = other.add_joint(point(0.0, 0.0, 0.0));
        let end = other.add_joint(point(1.0, 0.0, 0.0));
        let foreign_member = other.add_member(start, end);

        for member in [stale_member, foreign_member] {
            let properties_error = truss
                .set_member_properties(member, 0.01, 200.0e9)
                .expect_err("unknown member rejected");
            assert_eq!(properties_error, TrussEditError::UnknownMember(member));

            let yield_error = truss
                .set_member_yield_strength(member, 250.0e6)
                .expect_err("unknown member rejected");
            assert_eq!(yield_error, TrussEditError::UnknownMember(member));

            let remove_error = truss
                .remove_member(member)
                .expect_err("unknown member rejected");
            assert_eq!(remove_error, TrussEditError::UnknownMember(member));
        }
    }

    #[test]
    fn invalid_member_properties_are_rejected() {
        let mut truss = Truss::new();
        let a = truss.add_joint(point(0.0, 0.0, 0.0));
        let b = truss.add_joint(point(1.0, 0.0, 0.0));
        let member = truss.add_member(a, b);

        let area_error = truss
            .set_member_properties(member, 0.0, 200.0e9)
            .expect_err("zero area rejected");
        assert!(matches!(
            area_error,
            TrussEditError::InvalidMemberProperties(MemberPropertyError::NonPositiveArea { .. })
        ));

        let modulus_error = truss
            .set_member_properties(member, 0.01, 0.0)
            .expect_err("zero modulus rejected");
        assert!(matches!(
            modulus_error,
            TrussEditError::InvalidMemberProperties(
                MemberPropertyError::NonPositiveElasticModulus { .. }
            )
        ));
    }

    #[test]
    fn analysis_requires_member_properties() {
        let mut truss = Truss::new();
        let a = truss.add_joint(point(0.0, 0.0, 0.0));
        let b = truss.add_joint(point(1.0, 0.0, 0.0));
        let member = truss.add_member(a, b);

        let error = truss.evaluate().expect_err("missing properties detected");
        assert_eq!(error, AnalysisError::MissingProperties(member));
    }

    #[test]
    fn zero_length_member_is_rejected() {
        let mut truss = Truss::new();
        let a = truss.add_joint(point(0.0, 0.0, 0.0));
        let b = truss.add_joint(point(0.0, 0.0, 0.0));
        let member = truss.add_member(a, b);
        truss
            .set_member_properties(member, 0.01, 200.0e9)
            .expect("properties accepted");

        let error = truss.evaluate().expect_err("zero length detected");
        assert_eq!(error, AnalysisError::ZeroLengthMember(member));
    }

    #[test]
    fn cantilever_analysis_matches_expected_displacements() {
        let mut truss = Truss::new();
        let support = truss.add_joint(point(0.0, 0.0, 0.0));
        truss
            .set_support(support, [true, true, true])
            .expect("support applied");

        let free = truss.add_joint(point(1.0, 0.0, 0.0));
        truss
            .set_support(free, [false, true, true])
            .expect("support applied");
        truss
            .set_load(free, force(-1_000.0, 0.0, 0.0))
            .expect("load applied");

        let member = truss.add_member(support, free);
        truss
            .set_member_properties(member, 0.01, 200.0e9)
            .expect("properties accepted");

        truss.evaluate().expect("analysis succeeds");

        let displacement = truss
            .joint_displacement(free)
            .expect("displacement available");
        let expected_displacement = -1_000.0 * 1.0 / (0.01 * 200.0e9);
        assert_relative_eq!(displacement.x, expected_displacement, epsilon = 1.0e-12);
        assert_relative_eq!(displacement.y, 0.0, epsilon = 1.0e-9);
        assert_relative_eq!(displacement.z, 0.0, epsilon = 1.0e-9);

        let force = truss.member_axial_force(member).expect("force available");
        assert_relative_eq!(force, -1_000.0, epsilon = 1.0e-6);

        let stress = truss.member_stress(member).expect("stress available");
        assert_relative_eq!(stress, -100_000.0, epsilon = 1.0e-6);
    }

    #[test]
    fn factor_of_safety_is_reported_when_yield_strength_present() {
        let mut truss = Truss::new();
        let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
        let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
        truss
            .set_support(joint_a, [true, true, true])
            .expect("support applied");
        truss
            .set_support(joint_b, [false, true, true])
            .expect("support applied");
        truss
            .set_load(joint_b, force(-1_000.0, 0.0, 0.0))
            .expect("load applied");

        let member_ab = truss.add_member(joint_a, joint_b);
        truss
            .set_member_properties(member_ab, 0.01, 200.0e9)
            .expect("properties accepted");
        truss
            .set_member_yield_strength(member_ab, 250.0e6)
            .expect("yield strength accepted");

        truss.evaluate().expect("analysis succeeds");

        let fos = truss
            .member_factor_of_safety(member_ab)
            .expect("factor of safety available");
        assert!((fos - 2_500.0).abs() < 1.0e-6);
    }
}
