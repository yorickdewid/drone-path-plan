# Path Planning

A simple drone path planning simulator that finds efficient paths through a grid-based environment.

## Features

- Simulates drone movement on a 2D grid
- Cost-based pathfinding
- Timeout mechanism for long calculations
- Random starting positions
- Support for loading custom grid maps from files

## Build

To build the project:

```bash
# Clone into repository
cd path-plan

# Build the project
cargo build --release
```

## Usage

Build and run the project with Cargo:

```bash
# Run with default grid
cargo run

# Run with a custom grid file
cargo run -- /home/eve/Downloads/grids/1000.txt
```

## Parameters

The simulation uses the following parameters:
- Trail length: 3 (prevents revisiting recent positions)
- Simulation time: 1000ms per step
- Default timeout: 5000ms

## Input Format

Grid files should contain space-separated integers representing the cost of each cell.
Example:
```
1 5 5 2 5
1 2 3 4 2
3 1 0 4 0
9 5 1 6 1
0 2 6 1 3
```
