[![Crates.io](https://img.shields.io/crates/v/trussx.svg)](https://crates.io/crates/trussx)
[![docs.rs](https://docs.rs/trussx/badge.svg)](https://docs.rs/trussx)

# trussx

Utilities for building and analysing pin-jointed truss structures. The crate provides a
small, strongly-typed API for creating joints and members, applying loads and supports,
and running a linear elastic analysis in three dimensions.

## Installation

Add the dependency to your `Cargo.toml`:

```toml
[dependencies]
trussx = "0.1"
```

## Usage

```rust
use nalgebra::Vector3;
use trussx::{point, Truss};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut truss = Truss::new();
    let a = truss.add_joint(point(0.0, 0.0, 0.0));
    let b = truss.add_joint(point(1.0, 0.0, 0.0));
    truss.set_support(a, [true, true, true]);
    truss.set_support(b, [false, true, true]);
    truss.set_load(b, Vector3::new(-1000.0, 0.0, 0.0));

    let ab = truss.add_member(a, b);
    truss.set_member_properties(ab, 0.01, 200.0e9);

    truss.evaluate()?;

    let displacement = truss.joint_displacement(b).unwrap();
    println!("ux = {:.3e} m", displacement.x);
    Ok(())
}
```

## Testing

The project includes unit tests that validate the analysis results for a simple bar in
tension and ensure meaningful error reporting when required data is missing. Run the
full suite with:

```bash
cargo test
```
