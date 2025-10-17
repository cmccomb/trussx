#![warn(clippy::pedantic)]

use petgraph::graph::{EdgeIndex, NodeIndex};
use trussx::{force, point, Truss};

#[derive(Debug, Clone, Copy)]
struct CantileverGeometry {
    fixed_joint: NodeIndex,
    loaded_joint: NodeIndex,
    member: EdgeIndex,
}

#[derive(Debug, Clone, Copy)]
struct CantileverProperties {
    area: f64,
    elastic_modulus: f64,
    yield_strength: f64,
    axial_load: f64,
}

impl Default for CantileverProperties {
    fn default() -> Self {
        Self {
            area: 0.01,
            elastic_modulus: 200.0e9,
            yield_strength: 250.0e6,
            axial_load: -1_000.0,
        }
    }
}

fn build_cantilever_truss() -> (Truss, CantileverGeometry) {
    let mut truss = Truss::new();
    let fixed_joint = truss.add_joint(point(0.0, 0.0, 0.0));
    let loaded_joint = truss.add_joint(point(1.0, 0.0, 0.0));
    let member = truss.add_member(fixed_joint, loaded_joint);

    (
        truss,
        CantileverGeometry {
            fixed_joint,
            loaded_joint,
            member,
        },
    )
}

fn apply_cantilever_conditions(
    truss: &mut Truss,
    geometry: &CantileverGeometry,
) -> CantileverProperties {
    let properties = CantileverProperties::default();

    truss
        .set_support(geometry.fixed_joint, [true, true, true])
        .expect("fixed joint support assignment succeeds");
    truss
        .set_support(geometry.loaded_joint, [false, true, true])
        .expect("loaded joint support assignment succeeds");
    truss
        .set_load(
            geometry.loaded_joint,
            force(properties.axial_load, 0.0, 0.0),
        )
        .expect("axial load assignment succeeds");
    truss
        .set_member_properties(geometry.member, properties.area, properties.elastic_modulus)
        .expect("member properties assignment succeeds");
    truss
        .set_member_yield_strength(geometry.member, properties.yield_strength)
        .expect("member yield strength assignment succeeds");

    properties
}

#[test]
fn builds_expected_topology() {
    let (truss, geometry) = build_cantilever_truss();

    assert_eq!(truss.joint_count(), 2);
    assert_eq!(truss.member_count(), 1);
    assert_eq!(geometry.fixed_joint.index(), 0);
    assert_eq!(geometry.loaded_joint.index(), 1);
    assert_eq!(geometry.member.index(), 0);
}

#[test]
fn applies_conditions_and_evaluates() {
    let (mut truss, geometry) = build_cantilever_truss();
    let properties = apply_cantilever_conditions(&mut truss, &geometry);

    assert!((properties.area - 0.01).abs() < f64::EPSILON);
    truss.evaluate().expect("cantilever analysis succeeds");
}

#[test]
fn cantilever_response_matches_closed_form_solution() {
    let (mut truss, geometry) = build_cantilever_truss();
    let properties = apply_cantilever_conditions(&mut truss, &geometry);

    truss
        .evaluate()
        .expect("cantilever analysis produces results");

    let displacement = truss
        .joint_displacement(geometry.loaded_joint)
        .expect("cantilever displacement available");
    let axial_force = truss
        .member_axial_force(geometry.member)
        .expect("cantilever axial force available");
    let axial_stress = truss
        .member_stress(geometry.member)
        .expect("cantilever axial stress available");
    let factor_of_safety = truss
        .member_factor_of_safety(geometry.member)
        .expect("cantilever factor of safety available");

    let expected_displacement =
        properties.axial_load * 1.0 / (properties.area * properties.elastic_modulus);

    assert!((displacement.x - expected_displacement).abs() < 1.0e-12);
    assert!(displacement.y.abs() < f64::EPSILON);
    assert!(displacement.z.abs() < f64::EPSILON);

    assert!((axial_force - properties.axial_load).abs() < 1.0e-9);
    assert!((axial_stress - (properties.axial_load / properties.area)).abs() < 1.0e-6);
    assert!(
        (factor_of_safety - (properties.yield_strength.abs() / axial_stress.abs())).abs() < 1.0e-6
    );
}
