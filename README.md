# Verlet Integration Simulation in Rust

This project is a simple simulation of Verlet integration, a method used in computer graphics for numerically integrating Newton's equations of motion. This method is popular in simulations such as cloth and particle physics due to its simple implementation and stable results.

The simulation is implemented in Rust using the `macroquad` crate for rendering and the `rayon` crate for parallel computation.

## Features

- Verlet integration for accurate and stable physics simulation
- Multi-threaded computation for performance
- Interactive simulation where you can add particles by clicking
- Color-coded particles based on their velocity
- Configurable substep count using scroll wheel for higher precision

## How to Run

First, make sure you have Rust installed. If not, you can install it from [here](https://www.rust-lang.org/tools/install).

Then, clone this repository and navigate to the project directory:

```bash
git clone https://github.com/DuckyBlender/verlet-rs.git
cd verlet-rs
```

Finally, run the project:

```bash
cargo run
```

## Code Structure

- `VerletObject`: Represents a particle in the simulation. Each particle has a current position, a previous position, and an acceleration.
- `Solver`: Handles the physics simulation. It applies gravity to the particles, solves collisions, and updates the positions of the particles.
- `convert_velocity_to_color`: Converts the velocity of a particle to a color. Slow particles are blue, medium-speed particles are green, and fast particles are red.
- `hsl_to_rgb`: Converts a color from HSL color space to RGB color space.
- `main`: The main function of the program. It sets up the simulation, handles user input, and draws the particles.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
