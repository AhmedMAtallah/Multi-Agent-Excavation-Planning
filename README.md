# Multi-Agent Excavation Planning System

This project implements a distributed planning system for a fleet of excavation robots operating on a 2D grid. The robots are tasked with navigating the grid, digging at specified locations, and offloading the excavated material at predefined drop-off points.

## Project Overview

The system simulates multiple excavation robots that need to perform digging tasks on a grid. The planner computes motion plans for the robots, ensuring efficient task completion while avoiding collisions.

### Key Features

- **Grid-based Navigation**: Robots navigate a 2D grid, avoiding obstacles and other robots.
- **Task Assignment**: Robots are assigned to dig at specified locations and offload their payloads at drop-off points.
- **Collision Avoidance**: The planner ensures that robots do not collide while executing their tasks.
- **Task Efficiency**: The system optimizes task completion by allowing robots to operate concurrently.

## Project Structure