# Simple 2D Physics Engine

A basic 2D physics engine built in Rust using ggez for visualization and nalgebra for vector math. This project demonstrates fundamental physics simulation concepts.

## Overview

This engine simulates 2D rigid bodies with basic collision detection and response. It uses:
- **ggez:** For window creation, event handling (like mouse clicks to add objects), and rendering shapes.
- **nalgebra:** For vector and point operations essential for physics calculations.

The code aims for a clear separation between the core physics logic (`src/physics/`) and the simulation/rendering entry point (`src/main.rs`).

## Features

- **Physics Bodies:**
    - Dynamic and Static types.
    - Shapes: Circles and Axis-Aligned Rectangles.
    - Material properties: Density, Restitution (bounciness), Friction.
- **Integration:** Semi-implicit Euler integration for updating position and velocity based on forces (gravity, etc.).
- **Collision Detection:**
    - Pairwise (O(N^2)) detection between all bodies.
    - Collision checks implemented for Circle-Circle, Rectangle-Rectangle (AABB), and Circle-Rectangle pairs.
- **Collision Resolution:**
    - Iterative impulse-based solver (runs multiple passes per frame).
    - Handles restitution (bouncing) based on material properties.
    - Includes basic Coulomb friction calculation.
- **Visualization:** Simple rendering of bodies using ggez.
- **Interaction:** Click the left mouse button to add new dynamic balls to the simulation.

## Project Structure

```plaintext
physics-engine/
├── Cargo.toml
├── README.md
├── src/
│   ├── main.rs           # Entry point: ggez setup, game loop, rendering
│   └── physics/
│       ├── mod.rs        # Physics module definition and World struct
│       ├── bodies.rs     # Body, Shape, Material, BodyType definitions and update logic
│       └── collisions.rs # Collision detection and resolution logic
└── tests/
    └── physics_integration.rs # Integration tests for the physics engine
```

## Building and Running

1.  **Ensure Rust is installed:** If not, visit [rustup.rs](https://rustup.rs/).
2.  **Clone the repository:** `git clone <repository-url>`
3.  **Navigate to the directory:** `cd physics-engine`
4.  **Run the simulation:** `cargo run`
5.  **Run tests:** `cargo test`

## Current Limitations

- **No Rotation:** Bodies are treated as point masses (no angular velocity or torque).
- **Basic Solver:** The iterative impulse solver is simple and may exhibit jitter or instability in complex scenarios (like stacking, which is currently disabled in tests).
- **No Broad Phase:** Collision detection checks every pair of objects (O(N^2)), which will become slow with many objects.
- **Simple Shapes:** Only circles and axis-aligned rectangles are supported.

## Future Development Ideas

- Implement rotational physics (torque, angular velocity, moment of inertia).
- Add more complex shapes (polygons).
- Implement a broad-phase collision detection system (e.g., Spatial Hashing, Quadtree) for optimization.
- Explore more advanced solver techniques (e.g., Position Based Dynamics).
- Add joints and constraints.
