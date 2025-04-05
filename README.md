# My 2D Physics Engine

A simple 2D physics engine built in Rust that leverages modular design to separate simulation logic from rendering. This project serves as a foundation for physics-based applications, games, or simulations.

## Overview

This project demonstrates a basic 2D physics engine using:
- **ggez** for rendering, event handling, and window management.
- **nalgebra** for vector and matrix operations crucial to physics computations.

The engine is designed with a clear separation of concerns, enabling you to extend or swap out components (e.g., different rendering backends) with minimal changes to the core physics logic.

## Features

- **Modular Structure:** Organized code with distinct modules for physics and rendering.
- **Physics Simulation:** Includes basic body definitions, collision detection, and numerical integration (e.g., Euler, Verlet).
- **Easy Rendering:** Utilizes ggez to create a window and draw shapes to visualize the simulation.
- **Extendable:** Designed to easily add more features, such as advanced collision resolution, friction, or additional rendering techniques.

## Project Structure

```plaintext
my_2d_physics_engine/
├── Cargo.toml
└── src
    ├── main.rs           // Entry point: sets up the game loop and integrates modules.
    ├── physics/          // Physics simulation logic.
    │   ├── mod.rs        // Module exports.
    │   ├── bodies.rs     // Physical bodies definitions.
    │   ├── collisions.rs // Collision detection and resolution.
    │   └── integrator.rs // Numerical integration methods.
    └── rendering/        // Rendering code.
        ├── mod.rs        // Module exports.
        ├── window.rs     // Window creation and event handling.
        └── draw.rs       // Drawing functions for shapes and primitives.
