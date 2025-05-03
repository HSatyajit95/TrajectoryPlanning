#ifndef QUINTIC_TRAJECTORY_PLANNER_HPP
#define QUINTIC_TRAJECTORY_PLANNER_HPP

#include <vector>
#include <Eigen/Dense>
#include <tuple>
#include <string>

class QuinticTrajectoryPlanner {
public:
    /**
     * Initialize the trajectory planner with joint parameters and constraints.
     *
     * @param waypoints List of vectors, each containing joint angles for all joints at a waypoint.
     * @param v_waypoints List of vectors, each containing velocities at waypoints.
     * @param v_i Initial velocities for all joints (rad/s).
     * @param v_f Final velocities for all joints (rad/s).
     * @param a_i Initial accelerations for all joints (rad/s^2).
     * @param a_f Final accelerations for all joints (rad/s^2).
     * @param v_max Maximum velocities for all joints (rad/s).
     * @param a_max Maximum accelerations for all joints (rad/s^2).
     * @param T_min Initial search range minimum time per segment (s).
     * @param T_max Initial search range maximum time per segment (s).
     * @param tolerance Precision for binary search (s).
     * @param max_reruns Maximum number of times to double T_max.
     * @param num_samples Number of time samples per segment.
     */
    QuinticTrajectoryPlanner(
        const std::vector<std::vector<double>>& waypoints,
        const std::vector<std::vector<double>>& v_waypoints,
        const std::vector<double>& v_i,
        const std::vector<double>& v_f,
        const std::vector<double>& a_i,
        const std::vector<double>& a_f,
        const std::vector<double>& v_max,
        const std::vector<double>& a_max,
        double T_min = 0.1,
        double T_max = 10.0,
        double tolerance = 0.01,
        int max_reruns = 5,
        int num_samples = 100
    );

    /**
     * Plan a quintic polynomial trajectory for a single joint over one segment.
     *
     * @param theta_i Initial joint angle (rad).
     * @param theta_f Final joint angle (rad).
     * @param v_i Initial velocity (rad/s).
     * @param v_f Final velocity (rad/s).
     * @param a_i Initial acceleration (rad/s^2).
     * @param a_f Final acceleration (rad/s^2).
     * @param T Segment duration (s).
     * @param t_samples Array of time points for evaluation.
     * @return Tuple of position, velocity, acceleration trajectories, and polynomial coefficients.
     */
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
    planQuinticTrajectory(
        double theta_i, double theta_f, double v_i, double v_f, double a_i, double a_f,
        double T, const std::vector<double>& t_samples
    ) const;

    /**
     * Check if velocity and acceleration constraints are satisfied for all joints across all segments.
     *
     * @param T_segments List of durations for each segment (s).
     * @param num_samples Number of time samples for evaluation per segment.
     * @return Tuple of feasibility boolean, max velocities, and max accelerations.
     */
    std::tuple<bool, std::vector<double>, std::vector<double>>
    checkConstraintsAllJoints(const std::vector<double>& T_segments, int num_samples = 1000) const;

    /**
     * Find the minimum total time by assigning durations to each segment that satisfy constraints.
     *
     * @return List of optimal durations for each segment.
     */
    std::vector<double> findMinimumTime();

    /**
     * Generate trajectories for all joints across all segments using optimal segment times.
     *
     * @return Tuple of time samples, trajectories (pos, vel, acc, coeffs per joint per segment), and violations.
     */
    std::tuple<std::vector<double>,
               std::vector<std::vector<std::tuple<std::vector<double>,
                                                  std::vector<double>,
                                                  std::vector<double>,
                                                  std::vector<double>>>>,
               std::vector<std::string>>
    generateTrajectories();

    /**
     * Execute the full trajectory planning process.
     *
     * @return Tuple of success boolean and joint angle list across all segments.
     */
    std::tuple<bool, std::vector<std::vector<double>>> run();

private:
    std::vector<std::vector<double>> waypoints_;
    std::vector<std::vector<double>> v_waypoints_;
    std::vector<double> v_i_;
    std::vector<double> v_f_;
    std::vector<double> a_i_;
    std::vector<double> a_f_;
    std::vector<double> v_max_;
    std::vector<double> a_max_;
    double T_min_;
    double T_max_;
    double tolerance_;
    int max_reruns_;
    int num_samples_;
    int num_joints_;
    int num_segments_;
    std::vector<double> T_opt_segments_;
    std::vector<std::vector<std::tuple<std::vector<double>,
                                       std::vector<double>,
                                       std::vector<double>,
                                       std::vector<double>>>> trajectories_;
    std::vector<double> t_segment_starts_;
};

#endif // QUINTIC_TRAJECTORY_PLANNER_HPP