# ROS-Image-Based-Path-Simulation

## Problem Statement

This project involves developing a **ROS Node in C++** that generates and publishes a simulation image based on given input parameters. The node will:

1. Create an image of a specified size.
2. Mark obstacles, start position, and goal position.
3. Simulate the movement of an marker travelling from start to goal.

## Features

- **Customizable Simulation**: Define the width, height, obstacle positions, start, and goal locations.
- **Obstacle Representation**: Obstacles appear as black pixels in the generated image.
- **Start & Goal Markers**:
  - Start position is marked in **Yellow**.
  - Goal position is marked in **Blue**.
- **Arrow Path Simulation**: The arrow moves step-by-step towards the goal, stopping at obstacles.
- **Grid-by-Grid Update**: The path updates dynamically in the ROS topic.

## Input Parameters

The node requires the following parameters:

- **Width & Height**: Dimensions of the simulation space (in pixels).
- **Obstacle List**: List of `(x, y)` coordinates representing obstacles.
- **Starting Position**: `(x, y)` coordinate of the start position.
- **Goal Position**: `(x, y)` coordinate of the goal position.
- **Simulation Rate**: Speed of the simulation (in Hz).

## Installation & Usage

### Prerequisites

- ROS installed on your system
- OpenCV for image handling
- C++ compiler with CMake support

### Build Instructions

1. Clone the repository:
   ```bash
   git clone <repository_url>
   cd <repository_folder>
   ```
2. Build the package:
   ```bash
   catkin_make
   ```
3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

### Running the Node

Run the ROS node to generate the simulation:

```bash
roslaunch <package_name> <node_name>
```

### Expected Output

1. A white image (free space) is published.
2. Black obstacles are placed.
3. Yellow marker (start position) appears.
4. Blue marker (goal position) appears.
5. The arrow path updates grid-by-grid until it reaches the goal or an obstacle is hit.

## Example

To launch the simulation with custom parameters, modify the ROS launch file or pass arguments through a config file.

## Future Enhancements

- Add support for different obstacle shapes.
- Implement advanced path-planning algorithms.
- Provide a GUI interface for user-defined parameters.
