//! Fundamental geometric types for truss modelling.

use nalgebra::Vector3;

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
    pub fn to_vector(self) -> Vector3<f64> {
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
    pub fn to_vector(self) -> Vector3<f64> {
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
    pub fn to_vector(self) -> Vector3<f64> {
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
    use super::*;

    #[test]
    fn point_to_vector_roundtrip() {
        let origin = Point::new(1.0, 2.0, 3.0);
        let vector: Vector3<f64> = origin.into();
        assert_eq!(vector, Vector3::new(1.0, 2.0, 3.0));
        let point = Point::from(vector);
        assert_eq!(point, origin);
    }

    #[test]
    fn force_defaults_to_zero() {
        assert_eq!(Force::default(), Force::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn displacement_from_vector() {
        let vector = Vector3::new(0.1, -0.2, 0.3);
        let displacement = Displacement::from(vector);
        assert_eq!(displacement, Displacement::new(0.1, -0.2, 0.3));
    }
}
