
# Quintic Polynomial Trajectory Planning for Robotic Manipulators

![Robotic Manipulator]![meca500](https://github.com/user-attachments/assets/8e2dcd7a-4520-42a2-8ae9-7c12691aa8ba)

This repository contains a Python implementation of a **quintic polynomial trajectory planner** for multi-joint robotic manipulators, optimized for minimum time using a **binary search algorithm**. The planner ensures smooth motion by maintaining continuity in position, velocity, and acceleration, while adhering to kinematic constraints like maximum velocity and acceleration. It supports both single-segment and multi-waypoint trajectories, making it versatile for various robotic applications.

## 📋 Overview

Trajectory planning is crucial for robotic manipulators to move efficiently and smoothly between joint configurations. This project leverages **quintic polynomials** to achieve jerk-free motion and uses a **binary search algorithm** to find the shortest feasible time for trajectories. The implementation is designed for multi-degree-of-freedom (DOF) manipulators and is validated through simulations for a 6-DOF robot.

### Key Features
- **Smooth Motion**: Ensures continuity in position, velocity, and acceleration using quintic polynomials.
- **Time Optimization**: Finds the minimum feasible time with a binary search algorithm.
- **Multi-Waypoint Support**: Handles complex trajectories with multiple intermediate waypoints.
- **Python Implementation**: Uses NumPy for calculations and Matplotlib for visualization.
- **Generalizable**: Easily adaptable to manipulators with any number of joints.

## 🚀 Why Quintic Polynomials?

Quintic polynomials are fifth-degree polynomials that describe a joint's angle as a function of time:

```
θ(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

They are ideal for trajectory planning because they:
- Satisfy boundary conditions for position, velocity, and acceleration.
- Ensure smooth, jerk-continuous motion, reducing stress on actuators.
- Allow synchronized motion across multiple joints by sharing the same duration.

## 🔍 Binary Search for Time Optimization

To minimize trajectory time while respecting kinematic constraints (e.g., maximum velocity and acceleration), the planner uses a binary search algorithm. Here's how it works:

1. Define a time range `[T_min, T_max]` (e.g., 1 to 100 seconds).
2. Test the midpoint time by computing the quintic polynomial trajectory for each joint.
3. Check if the trajectory satisfies velocity and acceleration limits.
4. If feasible, try a shorter time; if not, try a longer time.
5. Repeat until the time converges to the smallest feasible value.

For multi-waypoint trajectories, the algorithm optimizes each segment's duration independently, ensuring continuity at waypoints.

## 🛠️ Implementation

The planner is implemented in Python as a `QuinticTrajectoryPlanner` class, with the following components:

- **Quintic Polynomial Solver**: Computes coefficients for each joint's trajectory.
- **Constraint Checker**: Verifies velocity and acceleration limits by sampling the trajectory.
- **Binary Search Optimizer**: Finds the minimum feasible time for each segment.
- **Trajectory Generator**: Produces position, velocity, and acceleration profiles.
- **Plotter**: Visualizes trajectories with segment boundaries.

### Example Code Snippet
```python
for seg in range(num_segments):
    low = T_min
    high = T_max
    while high - low > tolerance:
        mid = (low + high) / 2
        T_segments = T_opt_segments + [mid]
        is_feasible = check_constraints(T_segments)
        if is_feasible:
            T_opt_seg = mid
            high = mid
        else:
            low = mid
    T_opt_segments.append(T_opt_seg)
```

## 📈 Simulation Results

The planner was tested on a 6-DOF manipulator with the following parameters:
- **Initial Angles**: `[10.0, 20.0, 30.0, 40.0, 50.0, 60.0]` rad
- **Final Angles**: `[2.0, 10.8, -0.5, 1.2, -0.3, 0.6]` rad
- **Max Velocity**: 2.5 rad/s
- **Max Acceleration**: 2.0 rad/s²

**Results**:
- **Single-Segment**: Optimized time ≈ 27.044 seconds.
- **Multi-Waypoint** (one intermediate waypoint at `[5.0, 15.0, 15.0, 20.0, 25.0, 30.0]`): Total time ≈ 67.044 seconds.

The trajectories were smooth, constraint-compliant, and visualized with clear segment boundaries.

![Multi-Waypoint Trajectory](https://via.placeholder.com/800x400.png?text=Multi-Waypoint+Trajectory)

## 🔧 Extending to Multi-Waypoint Trajectories

To adapt the single-segment planner for multiple waypoints, the following modifications were made:

1. **Input Structure**: Accept a list of waypoint joint angles and velocities.
2. **Segment Planning**: Compute quintic polynomials for each segment, ensuring continuity at waypoints.
3. **Time Optimization**: Use binary search to optimize each segment's duration.
4. **Trajectory Generation**: Concatenate segment trajectories and track start times.
5. **Plotting**: Add vertical lines to mark segment boundaries in visualizations.

## 🌟 Applications

This trajectory planner is applicable to:
- **Industrial Robotics**: Assembly lines, pick-and-place tasks.
- **Medical Robotics**: Surgical robots requiring precise, smooth motion.
- **Autonomous Systems**: Manipulators in dynamic environments.

## 🔮 Future Work

- **Jerk Constraints**: Limit the rate of change of acceleration for smoother motion.
- **Joint Optimization**: Optimize all segment times simultaneously.
- **Real-Time Integration**: Embed the planner in control systems for dynamic tasks.

## 📚 References

- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*.
- Siciliano, B., et al. (2008). *Robotics: Modelling, Planning and Control*.
- Spong, M. W. (1994). *Robot Dynamics and Control*.
- Gasparetto, A., & Zanotto, V. (2008). *A New Method for Smooth Trajectory Planning of Robot Manipulators*.
- Verscheure, D., et al. (2009). *Time-Optimal Path Planning for Robots*.

## 🤝 Contributing

Contributions are welcome! Feel free to submit issues, fork the repository, or create pull requests to improve the planner or add new features.

## 📬 Contact

For questions or feedback, reach out via [GitHub Issues](https://github.com/placeholder/robotics/issues) or connect with me on [LinkedIn](https://linkedin.com/in/placeholder).

---

*Happy coding, and may your robots move smoothly!* 🦾
