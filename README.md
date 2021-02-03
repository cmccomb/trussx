[![Build Status](https://travis-ci.com/cmccomb/trussx.svg?branch=master)](https://travis-ci.com/cmccomb/trussx)
[![Crates.io](https://img.shields.io/crates/v/trussx.svg)](https://crates.io/crates/trussx)
[![docs.rs](https://docs.rs/trussx/badge.svg)](https://docs.rs/trussx)
# About
This package provides utilities for designing and analyzing truss structures

# Installation
Install through ``crates.io`` with:
```shell script
cargo install trussx
```

Then add it to your ``Cargo.toml` with:
```toml
[dependencies]
trussx = "0.1.1"
```
and add this to your root:
```rust
use trussx;
```
# Usage
Here are some basic examples of usage
## Building a truss
For example, you can build a truss with something like:
```rust
let x = Truss::new()
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
```

## Analyzing a truss
Coming soon!