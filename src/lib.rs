#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

//! Tools for building and analysing simple pin-jointed truss structures.

use std::collections::HashMap;

use nalgebra::{DMatrix, DVector, SMatrix, Vector3};
use petgraph::graph::{EdgeIndex, Graph, NodeIndex};
use thiserror::Error;

/// Position in three dimensional space measured in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    /// Distance along the global X axis.
    pub x: f64,
    /// Distance along the global Y axis.
    pub y: f64,
    /// Distance along the global Z axis.
    pub z: f64,
}

impl Point {
    /// Create a [`Point`] with explicit coordinates.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Convert the point into an algebraic vector.
    #[must_use]
    fn to_vector(self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl From<Vector3<f64>> for Point {
    fn from(value: Vector3<f64>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Point> for Vector3<f64> {
    fn from(value: Point) -> Self {
        value.to_vector()
    }
}

/// Cartesian vector representing a three dimensional force in newtons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Force {
    /// Force component acting along the global X axis.
    pub x: f64,
    /// Force component acting along the global Y axis.
    pub y: f64,
    /// Force component acting along the global Z axis.
    pub z: f64,
}

impl Force {
    /// Create a [`Force`] with explicit components.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Convert the force into an algebraic vector.
    #[must_use]
    fn to_vector(self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl Default for Force {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl From<Vector3<f64>> for Force {
    fn from(value: Vector3<f64>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Force> for Vector3<f64> {
    fn from(value: Force) -> Self {
        value.to_vector()
    }
}

/// Translation vector describing joint displacement in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Displacement {
    /// Displacement component along the global X axis.
    pub x: f64,
    /// Displacement component along the global Y axis.
    pub y: f64,
    /// Displacement component along the global Z axis.
    pub z: f64,
}

impl Displacement {
    /// Create a [`Displacement`] with explicit components.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Convert the displacement into an algebraic vector.
    #[must_use]
    fn to_vector(self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl Default for Displacement {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl From<Vector3<f64>> for Displacement {
    fn from(value: Vector3<f64>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Displacement> for Vector3<f64> {
    fn from(value: Displacement) -> Self {
        value.to_vector()
    }
}

/// Error returned when a truss analysis fails.
#[derive(Debug, Error, PartialEq)]
pub enum AnalysisError {
    /// Returned when a member is missing required material properties.
    #[error("member {0:?} is missing material properties")]
    MissingProperties(EdgeIndex),
    /// Returned when a member spans zero distance.
    #[error("member {0:?} has zero length")]
    ZeroLengthMember(EdgeIndex),
    /// Returned when the supplied properties for a member are not physically meaningful.
    #[error("member {member:?} has invalid properties: {source}")]
    InvalidMemberProperties {
        /// Identifier of the offending member.
        member: EdgeIndex,
        /// Description of the invalid property.
        #[source]
        source: MemberPropertyError,
    },
    /// Returned when the stiffness matrix cannot be inverted.
    #[error("stiffness matrix is singular; check supports and connectivity")]
    SingularStiffness,
}

/// Error returned when updating material properties for a truss member.
///
/// The variants describe the reason the supplied value is rejected so callers can
/// present actionable feedback to users.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MemberPropertyError {
    /// Returned when the cross-sectional area is zero or negative.
    #[error("area must be positive (received {area})")]
    NonPositiveArea {
        /// Identifier of the affected member.
        member: EdgeIndex,
        /// Rejected cross-sectional area in square metres.
        area: f64,
    },
    /// Returned when the elastic modulus is zero or negative.
    #[error("elastic modulus must be positive (received {elastic_modulus})")]
    NonPositiveElasticModulus {
        /// Identifier of the affected member.
        member: EdgeIndex,
        /// Rejected elastic modulus in pascals.
        elastic_modulus: f64,
    },
}

/// Error returned when editing a [`Truss`] with invalid indices.
///
/// Attempting to mutate the structure with a joint or member that is not part of the
/// current graph returns a descriptive variant so callers can decide how to recover.
///
/// # Examples
///
/// ```
/// use petgraph::graph::EdgeIndex;
/// use trussx::{Truss, TrussEditError};
///
/// let mut truss = Truss::new();
/// let invalid_member = EdgeIndex::new(42);
/// let error = truss
///     .set_member_properties(invalid_member, 0.01, 200.0e9)
///     .expect_err("unknown member is rejected");
/// assert_eq!(error, TrussEditError::UnknownMember(invalid_member));
/// ```
#[derive(Debug, Error, PartialEq)]
pub enum TrussEditError {
    /// Returned when a joint cannot be found in the truss.
    #[error("joint {0:?} does not exist in this truss")]
    UnknownJoint(NodeIndex),
    /// Returned when a member cannot be found in the truss.
    #[error("member {0:?} does not exist in this truss")]
    UnknownMember(EdgeIndex),
    /// Returned when the supplied member properties are invalid.
    #[error("{0}")]
    InvalidMemberProperties(MemberPropertyError),
}

#[derive(Clone, Debug)]
struct Joint {
    /// Position of the joint in metres.
    position: Point,
    /// True if the corresponding translational degree of freedom is restrained.
    support: [bool; 3],
    /// External load applied to the joint in newtons.
    load: Force,
    /// Resulting displacement after analysis in metres.
    displacement: Displacement,
}

impl Joint {
    /// Create a new joint at the supplied position.
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
    /// let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
    /// let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
    /// let member = truss.add_member(joint_a, joint_b);
    ///
    /// let error = truss
    ///     .set_member_properties(member, 0.0, 200.0e9)
    ///     .expect_err("zero area rejected");
    /// assert!(matches!(
    ///     error,
    ///     TrussEditError::InvalidMemberProperties(
    ///         trussx::MemberPropertyError::NonPositiveArea { .. }
    ///     )
    /// ));
    ///
    /// truss
    ///     .set_member_properties(member, 0.01, 200.0e9)
    ///     .expect("positive properties accepted");
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
            Ok(())
        } else {
            Err(TrussEditError::UnknownMember(member))
        }
    }

    /// Set the yield strength for a member.
    ///
    /// The supplied value is stored in pascals and used during [`Truss::evaluate`] to compute
    /// a factor of safety against yielding. Call [`Truss::member_factor_of_safety`] after an
    /// analysis to retrieve the resulting value.
    ///
    /// # Errors
    ///
    /// Returns [`TrussEditError::UnknownMember`] when `member` is not part of this truss.
    pub fn set_member_yield_strength(
        &mut self,
        member: EdgeIndex,
        yield_strength: f64,
    ) -> Result<(), TrussEditError> {
        self.invalidate();
        if let Some(edge) = self.graph.edge_weight_mut(member) {
            edge.yield_strength = Some(yield_strength);
            Ok(())
        } else {
            Err(TrussEditError::UnknownMember(member))
        }
    }

    /// Reset stored analysis results.
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
            joint.displacement = Displacement::new(
                displacements[base],
                displacements[base + 1],
                displacements[base + 2],
            );
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
            let axial_extension = direction.dot(&(end_disp.to_vector() - start_disp.to_vector()));
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
    pub fn joint_displacement(&self, joint: NodeIndex) -> Option<Displacement> {
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
    ///
    /// The factor of safety is computed as `|yield_strength| / |stress|`. The result is `None`
    /// when no yield strength has been assigned, [`Truss::evaluate`] has not yet been called, the
    /// computed stress is effectively zero, or the ratio is not finite.
    ///
    /// # Examples
    /// ```
    /// use trussx::{force, point, Truss};
    ///
    /// let mut truss = Truss::new();
    /// let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
    /// let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
    /// truss.set_support(joint_a, [true, true, true]);
    /// truss.set_support(joint_b, [false, true, true]);
    /// truss.set_load(joint_b, force(-1_000.0, 0.0, 0.0));
    ///
    /// let member_ab = truss.add_member(joint_a, joint_b);
    /// truss.set_member_properties(member_ab, 0.01, 200.0e9);
    /// truss.set_member_yield_strength(member_ab, 250.0e6);
    ///
    /// truss.evaluate().unwrap();
    ///
    /// let fos = truss.member_factor_of_safety(member_ab).unwrap();
    /// assert!((fos - 2500.0).abs() < 1.0e-6);
    /// ```
    #[must_use]
    pub fn member_factor_of_safety(&self, member: EdgeIndex) -> Option<f64> {
        self.graph
            .edge_weight(member)
            .and_then(|member| member.factor_of_safety)
    }
}

/// Convenience helper for creating [`Point`] instances.
///
/// # Examples
/// ```
/// use trussx::point;
///
/// let origin = point(0.0, 0.0, 0.0);
/// assert_eq!(origin.x, 0.0);
/// ```
#[must_use]
pub const fn point(x: f64, y: f64, z: f64) -> Point {
    Point::new(x, y, z)
}

/// Convenience helper for creating [`Force`] instances.
///
/// # Examples
/// ```
/// use trussx::force;
///
/// let load = force(1.0, 0.0, -5.0);
/// assert_eq!(load.z, -5.0);
/// ```
#[must_use]
pub const fn force(x: f64, y: f64, z: f64) -> Force {
    Force::new(x, y, z)
}

/// Convenience helper for creating [`Displacement`] instances.
///
/// # Examples
/// ```
/// use trussx::displacement;
///
/// let delta = displacement(0.001, 0.0, 0.0);
/// assert_eq!(delta.x, 0.001);
/// ```
#[must_use]
pub const fn displacement(x: f64, y: f64, z: f64) -> Displacement {
    Displacement::new(x, y, z)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

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
            let props_error = truss
                .set_member_properties(member, 0.01, 200.0e9)
                .expect_err("unknown member rejected");
            assert_eq!(props_error, TrussEditError::UnknownMember(member));

            let yield_error = truss
                .set_member_yield_strength(member, 250.0e6)
                .expect_err("unknown member rejected");
            assert_eq!(yield_error, TrussEditError::UnknownMember(member));
        }

        let stale_remove_error = truss
            .remove_member(stale_member)
            .expect_err("stale member rejected");
        assert_eq!(
            stale_remove_error,
            TrussEditError::UnknownMember(stale_member)
        );

        let foreign_remove_error = truss
            .remove_member(foreign_member)
            .expect_err("foreign member rejected");
        assert_eq!(
            foreign_remove_error,
            TrussEditError::UnknownMember(foreign_member)
        );
    }

    #[test]
    fn member_properties_require_positive_values() {
        let mut truss = Truss::new();
        let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
        let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
        let member = truss.add_member(joint_a, joint_b);

        let zero_area_error = truss
            .set_member_properties(member, 0.0, 200.0e9)
            .expect_err("zero area rejected");
        assert_eq!(
            zero_area_error,
            TrussEditError::InvalidMemberProperties(MemberPropertyError::NonPositiveArea {
                member,
                area: 0.0,
            },)
        );

        let negative_modulus_error = truss
            .set_member_properties(member, 0.01, -200.0e9)
            .expect_err("negative modulus rejected");
        assert_eq!(
            negative_modulus_error,
            TrussEditError::InvalidMemberProperties(
                MemberPropertyError::NonPositiveElasticModulus {
                    member,
                    elastic_modulus: -200.0e9,
                },
            )
        );

        let zero_modulus_error = truss
            .set_member_properties(member, 0.02, 0.0)
            .expect_err("zero modulus rejected");
        assert_eq!(
            zero_modulus_error,
            TrussEditError::InvalidMemberProperties(
                MemberPropertyError::NonPositiveElasticModulus {
                    member,
                    elastic_modulus: 0.0,
                },
            )
        );

        truss
            .set_member_properties(member, 0.02, 210.0e9)
            .expect("positive properties accepted");
    }

    #[test]
    fn single_bar_in_tension() {
        let mut truss = Truss::new();
        let joint_a = truss.add_joint(point(0.0, 0.0, 0.0));
        let joint_b = truss.add_joint(point(1.0, 0.0, 0.0));
        truss
            .set_support(joint_a, [true, true, true])
            .expect("joint A exists");
        truss
            .set_support(joint_b, [false, true, true])
            .expect("joint B exists");
        truss
            .set_load(joint_b, force(-1000.0, 0.0, 0.0))
            .expect("joint B exists");
        let member_ab = truss.add_member(joint_a, joint_b);
        truss
            .set_member_properties(member_ab, 0.01, 200.0e9)
            .expect("member AB exists");
        truss
            .set_member_yield_strength(member_ab, 250.0e6)
            .expect("member AB exists");

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
        truss
            .set_support(joint_a, [true, true, true])
            .expect("joint A exists");
        truss
            .set_support(joint_b, [false, true, true])
            .expect("joint B exists");
        truss
            .set_load(joint_b, force(-1000.0, 0.0, 0.0))
            .expect("joint B exists");
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

        truss
            .set_support(joint_a, [true, true, true])
            .expect("joint A exists");
        truss
            .set_support(joint_b, [false, true, true])
            .expect("joint B exists");
        truss
            .set_support(joint_c, [false, false, true])
            .expect("joint C exists");
        truss
            .set_load(joint_c, force(0.0, -10_000.0, 0.0))
            .expect("joint C exists");

        let bottom_member = truss.add_member(joint_a, joint_b);
        let left_diagonal = truss.add_member(joint_a, joint_c);
        let right_diagonal = truss.add_member(joint_b, joint_c);

        for member in [bottom_member, left_diagonal, right_diagonal] {
            truss
                .set_member_properties(member, 0.003, 200.0e9)
                .expect("member exists");
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

        truss
            .set_support(left_support, [true, true, true])
            .expect("left support exists");
        truss
            .set_support(right_support, [false, true, true])
            .expect("right support exists");
        for joint in [
            lower_panel_b,
            lower_panel_c,
            left_apex,
            mid_apex,
            right_apex,
        ] {
            truss
                .set_support(joint, [false, false, true])
                .expect("joint exists");
        }

        let roof_load = force(0.0, -15_000.0, 0.0);
        for joint in [left_apex, mid_apex, right_apex] {
            truss.set_load(joint, roof_load).expect("joint exists");
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
            truss
                .set_member_properties(member, 0.004, 210.0e9)
                .expect("member exists");
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
