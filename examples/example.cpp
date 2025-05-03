/*
#include "QuinticTrajectoryPlanner.hpp"
#include <iostream>

int main() {
    // Parameters for 6-DOF manipulator with 4 waypoints
    int num_joints = 6;
    std::vector<std::vector<double>> waypoints = {
        {10.0, 20.0, 30.0, 40.0, 50.0, 60.0},  // Initial angles (rad)
        {5.0, 15.0, 15.0, 20.0, 25.0, 30.0},   // First intermediate angles (rad)
        {3.5, 12.5, 7.0, 10.0, 12.0, 15.0},    // Second intermediate angles (rad)
        {2.0, 10.8, -0.5, 1.2, -0.3, 0.6}      // Final angles (rad)
    };
    std::vector<std::vector<double>> v_waypoints = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Initial velocities (rad/s)
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},  // First intermediate velocities (rad/s)
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},  // Second intermediate velocities (rad/s)
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}   // Final velocities (rad/s)
    };
    std::vector<double> v_i = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Initial velocities (rad/s)
    std::vector<double> v_f = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Final velocities (rad/s)
    std::vector<double> a_i = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Initial accelerations (rad/s^2)
    std::vector<double> a_f = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Final accelerations (rad/s^2)
    std::vector<double> v_max = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5};  // Max velocities (rad/s)
    std::vector<double> a_max = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};  // Max accelerations (rad/s^2)

    // Initialize planner
    QuinticTrajectoryPlanner planner(
        waypoints, v_waypoints, v_i, v_f, a_i, a_f, v_max, a_max,
        0.1, 10.0, 0.01, 5, 100
    );

    // Run the planner
    auto [success, joint_angles] = planner.run();

    // Output results
    if (success) {
        std::cout << "Trajectory planning successful.\n";
        for (int j = 0; j < num_joints; ++j) {
            std::cout << "Joint " << j + 1 << " angles (first 10 samples): ";
            for (int i = 0; i < std::min(10, (int)joint_angles[j].size()); ++i) {
                std::cout << joint_angles[j][i] << " ";
            }
            std::cout << "...\n";
        }
    } else {
        std::cout << "Trajectory planning failed due to constraint violations.\n";
    }

    return 0;
}

*/

#include "QuinticTrajectoryPlanner.hpp"
#include <iostream>

int main() {
    // Parameters for 6-DOF manipulator with only first and last waypoints
    int num_joints = 6;
    std::vector<std::vector<double>> waypoints = {
        {10.0, 20.0, 30.0, 40.0, 50.0, 60.0},  // First (initial) angles (rad)
        {2.0, 10.8, -0.5, 1.2, -0.3, 0.6}      // Last (final) angles (rad)
    };
    std::vector<std::vector<double>> v_waypoints = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Velocities at first waypoint (rad/s)
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}   // Velocities at last waypoint (rad/s)
    };
    std::vector<double> v_i = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Initial velocities (rad/s)
    std::vector<double> v_f = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Final velocities (rad/s)
    std::vector<double> a_i = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Initial accelerations (rad/s^2)
    std::vector<double> a_f = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Final accelerations (rad/s^2)
    std::vector<double> v_max = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5};  // Max velocities (rad/s)
    std::vector<double> a_max = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};  // Max accelerations (rad/s^2)

    // Initialize planner
    QuinticTrajectoryPlanner planner(
        waypoints, v_waypoints, v_i, v_f, a_i, a_f, v_max, a_max,
        0.1, 10.0, 0.01, 5, 100
    );

    // Run the planner
    auto [success, joint_angles] = planner.run();

    // Output results
    if (success) {
        std::cout << "Trajectory planning successful.\n";
        for (int j = 0; j < num_joints; ++j) {
            std::cout << "Joint " << j + 1 << " angles (first 10 samples): ";
            for (int i = 0; i < std::min(10, (int)joint_angles[j].size()); ++i) {
                std::cout << joint_angles[j][i] << " ";
            }
            std::cout << "...\n";
        }
    } else {
        std::cout << "Trajectory planning failed due to constraint violations.\n";
    }

    return 0;
}