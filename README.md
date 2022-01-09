[![Crates.io](https://img.shields.io/crates/v/trussx.svg)](https://crates.io/crates/trussx)
[![docs.rs](https://docs.rs/trussx/badge.svg)](https://docs.rs/trussx)
# About
This package provides utilities for designing and analyzing truss structures

# Usage
Here are some basic examples of usage
## Building a truss
For example, you can build a truss with something like:
```rust
fn main() {
    let mut x = Truss::new();
    let a = x.add_joint([0.0, 0.0, 0.0]);
    let b = x.add_joint([3.0, 0.0, 0.0]);
    let c = x.add_joint([1.5, 1.5, 0.0]);
    let _ab = x.add_edge(a, b);
    let _bc = x.add_edge(b, c);
    let _ac = x.add_edge(a, c);
}
```

## Analyzing a truss
Coming soon!