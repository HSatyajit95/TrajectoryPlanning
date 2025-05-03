#include "QuinticTrajectoryPlanner.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

QuinticTrajectoryPlanner::QuinticTrajectoryPlanner(
    const std::vector<std::vector<double>>& waypoints,
    const std::vector<std::vector<double>>& v_waypoints,
    const std::vector<double>& v_i,
    const std::vector<double>& v_f,
    const std::vector<double>& a_i,
    const std::vector<double>& a_f,
    const std::vector<double>& v_max,
    const std::vector<double>& a_max,
    double T_min,
    double T_max,
    double tolerance,
    int max_reruns,
    int num_samples
) : waypoints_(waypoints),
    v_waypoints_(v_waypoints),
    v_i_(v_i),
    v_f_(v_f),
    a_i_(a_i),
    a_f_(a_f),
    v_max_(v_max),
    a_max_(a_max),
    T_min_(T_min),
    T_max_(T_max),
    tolerance_(tolerance),
    max_reruns_(max_reruns),
    num_samples_(num_samples),
    num_joints_(waypoints[0].size()),
    num_segments_(waypoints.size() - 1) {
    // Validate inputs
    for (const auto& w : waypoints) {
        assert(w.size() == num_joints_ && "All waypoints must have same number of joints");
    }
    assert(v_waypoints.size() == waypoints.size() && "v_waypoints must match number of waypoints");
    for (const auto& v : v_waypoints) {
        assert(v.size() == num_joints_ && "All v_waypoints must have same number of joints");
    }
    assert(v_i.size() == num_joints_ && "v_i must match number of joints");
    assert(v_f.size() == num_joints_ && "v_f must match number of joints");
    assert(a_i.size() == num_joints_ && "a_i must match number of joints");
    assert(a_f.size() == num_joints_ && "a_f must match number of joints");
    assert(v_max.size() == num_joints_ && "v_max must match number of joints");
    assert(a_max.size() == num_joints_ && "a_max must match number of joints");
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
QuinticTrajectoryPlanner::planQuinticTrajectory(
    double theta_i, double theta_f, double v_i, double v_f, double a_i, double a_f,
    double T, const std::vector<double>& t_samples
) const {
    Eigen::MatrixXd A(6, 6);
    A << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T,
         0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T,
         0, 0, 2, 6*T, 12*T*T, 20*T*T*T;

    Eigen::VectorXd b(6);
    b << theta_i, v_i, a_i, theta_f, v_f, a_f;

    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

    std::vector<double> pos(t_samples.size());
    std::vector<double> vel(t_samples.size());
    std::vector<double> acc(t_samples.size());
    std::vector<double> coeffs_vec(coeffs.data(), coeffs.data() + coeffs.size());

    for (size_t i = 0; i < t_samples.size(); ++i) {
        double t = t_samples[i];
        pos[i] = coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t +
                 coeffs[4]*t*t*t*t + coeffs[5]*t*t*t*t*t;
        vel[i] = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t*t +
                 4*coeffs[4]*t*t*t + 5*coeffs[5]*t*t*t*t;
        acc[i] = 2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t*t +
                 20*coeffs[5]*t*t*t;
    }

    return {pos, vel, acc, coeffs_vec};
}

std::tuple<bool, std::vector<double>, std::vector<double>>
QuinticTrajectoryPlanner::checkConstraintsAllJoints(const std::vector<double>& T_segments, int num_samples) const {
    bool is_feasible = true;
    std::vector<double> max_vels(num_joints_, 0.0);
    std::vector<double> max_accs(num_joints_, 0.0);

    for (int seg = 0; seg < num_segments_; ++seg) {
        std::vector<double> t_samples(num_samples);
        for (int i = 0; i < num_samples; ++i) {
            t_samples[i] = (T_segments[seg] * i) / (num_samples - 1);
        }

        const auto& theta_i = waypoints_[seg];
        const auto& theta_f = waypoints_[seg + 1];
        const auto& v_i_seg = (seg == 0) ? v_i_ : v_waypoints_[seg];
        const auto& v_f_seg = v_waypoints_[seg + 1];
        const auto& a_i_seg = (seg == 0) ? a_i_ : std::vector<double>(num_joints_, 0.0);
        const auto& a_f_seg = (seg == num_segments_ - 1) ? a_f_ : std::vector<double>(num_joints_, 0.0);

        for (int j = 0; j < num_joints_; ++j) {
            auto [pos, vel, acc, coeffs] = planQuinticTrajectory(
                theta_i[j], theta_f[j], v_i_seg[j], v_f_seg[j], a_i_seg[j], a_f_seg[j],
                T_segments[seg], t_samples
            );

            double max_vel = 0.0;
            double max_acc = 0.0;
            for (const auto& v : vel) max_vel = std::max(max_vel, std::abs(v));
            for (const auto& a : acc) max_acc = std::max(max_acc, std::abs(a));

            max_vels[j] = std::max(max_vels[j], max_vel);
            max_accs[j] = std::max(max_accs[j], max_acc);

            if (max_vel > v_max_[j] + 1e-6 || max_acc > a_max_[j] + 1e-6) {
                is_feasible = false;
            }
        }
    }

    return {is_feasible, max_vels, max_accs};
}

std::vector<double> QuinticTrajectoryPlanner::findMinimumTime() {
    double current_T_max = T_max_;
    int rerun_count = 0;

    while (rerun_count <= max_reruns_) {
        std::cout << "\nRerun " << rerun_count << ": Searching in range T_segment=[" 
                  << T_min_ << ", " << current_T_max << "]\n";
        std::vector<double> T_opt_segments; // Local variable
        double total_T_opt = 0;

        for (int seg = 0; seg < num_segments_; ++seg) {
            double low = T_min_;
            double high = current_T_max;
            double T_opt_seg = high;

            while (high - low > tolerance_) {
                double mid = (low + high) / 2;
                std::vector<double> T_segments = T_opt_segments; // Copy previous segment times
                T_segments.push_back(mid); // Add current segment time
                for (int i = T_opt_segments.size() + 1; i < num_segments_; ++i) {
                    T_segments.push_back(T_min_); // Fill remaining segments
                }

                auto [is_feasible, max_vels, max_accs] = checkConstraintsAllJoints(T_segments);
                if (is_feasible) {
                    T_opt_seg = mid;
                    high = mid;
                } else {
                    low = mid;
                }
            }
            T_opt_segments.push_back(T_opt_seg); // Store the optimal time for this segment
        }

        // Check feasibility of all segments using T_opt_segments
        auto [is_feasible, max_vels, max_accs] = checkConstraintsAllJoints(T_opt_segments);
        if (is_feasible) {
            T_opt_segments_ = T_opt_segments;
            total_T_opt = 0;
            for (const auto& T : T_opt_segments) total_T_opt += T;
            std::cout << "Found feasible segment times: ";
            for (const auto& T : T_opt_segments) std::cout << T << " ";
            std::cout << ", Total T_opt=" << total_T_opt << " seconds\n";
            return T_opt_segments;
        }

        rerun_count++;
        current_T_max *= 2;
        std::cout << "Segment times not feasible, doubling T_max to " << current_T_max << "\n";
    }

    std::cout << "Warning: Reached maximum reruns (" << max_reruns_ 
              << "). Returning last T_opt_segments, but it may not be feasible.\n";
    std::vector<double> T_opt_segments; // Local variable for fallback case
    T_opt_segments_ = T_opt_segments; // Assign empty vector to member
    return T_opt_segments;
}

std::tuple<std::vector<double>,
           std::vector<std::vector<std::tuple<std::vector<double>,
                                              std::vector<double>,
                                              std::vector<double>,
                                              std::vector<double>>>>,
           std::vector<std::string>>
QuinticTrajectoryPlanner::generateTrajectories() {
    if (T_opt_segments_.empty()) {
        throw std::runtime_error("Must find minimum segment times before generating trajectories.");
    }

    std::vector<std::vector<std::tuple<std::vector<double>,
                                       std::vector<double>,
                                       std::vector<double>,
                                       std::vector<double>>>> trajectories(num_joints_);
    std::vector<std::string> violations;
    std::vector<double> t_samples_all;
    double current_t = 0;

    t_segment_starts_ = {0};
    for (const auto& T_seg : T_opt_segments_) {
        current_t += T_seg;
        t_segment_starts_.push_back(current_t);
    }

    for (int seg = 0; seg < num_segments_; ++seg) {
        std::vector<double> t_samples_seg(num_samples_);
        for (int i = 0; i < num_samples_; ++i) {
            t_samples_seg[i] = (T_opt_segments_[seg] * i) / (num_samples_ - 1);
        }
        std::vector<double> t_samples_seg_global(num_samples_);
        for (int i = 0; i < num_samples_; ++i) {
            t_samples_seg_global[i] = t_samples_seg[i] + t_segment_starts_[seg];
        }
        t_samples_all.insert(t_samples_all.end(), t_samples_seg_global.begin(), t_samples_seg_global.end());

        const auto& theta_i = waypoints_[seg];
        const auto& theta_f = waypoints_[seg + 1];
        const auto& v_i_seg = (seg == 0) ? v_i_ : v_waypoints_[seg];
        const auto& v_f_seg = v_waypoints_[seg + 1];
        const auto& a_i_seg = (seg == 0) ? a_i_ : std::vector<double>(num_joints_, 0.0);
        const auto& a_f_seg = (seg == num_segments_ - 1) ? a_f_ : std::vector<double>(num_joints_, 0.0);

        for (int j = 0; j < num_joints_; ++j) {
            auto [pos, vel, acc, coeffs] = planQuinticTrajectory(
                theta_i[j], theta_f[j], v_i_seg[j], v_f_seg[j], a_i_seg[j], a_f_seg[j],
                T_opt_segments_[seg], t_samples_seg
            );
            trajectories[j].push_back({pos, vel, acc, coeffs});

            double max_vel = 0.0;
            double max_acc = 0.0;
            for (const auto& v : vel) max_vel = std::max(max_vel, std::abs(v));
            for (const auto& a : acc) max_acc = std::max(max_acc, std::abs(a));

            if (max_vel > v_max_[j] + 1e-6) {
                violations.push_back("Joint " + std::to_string(j + 1) + ", Segment " +
                                     std::to_string(seg + 1) + ": Max velocity " +
                                     std::to_string(max_vel) + " rad/s exceeds limit " +
                                     std::to_string(v_max_[j]) + " rad/s");
            }
            if (max_acc > a_max_[j] + 1e-6) {
                violations.push_back("Joint " + std::to_string(j + 1) + ", Segment " +
                                     std::to_string(seg + 1) + ": Max acceleration " +
                                     std::to_string(max_acc) + " rad/s^2 exceeds limit " +
                                     std::to_string(a_max_[j]) + " rad/s^2");
            }
        }
    }

    trajectories_ = trajectories;
    return {t_samples_all, trajectories, violations};
}

std::tuple<bool, std::vector<std::vector<double>>> QuinticTrajectoryPlanner::run() {
    T_opt_segments_ = findMinimumTime();
    double total_T = 0;
    for (const auto& T : T_opt_segments_) total_T += T;
    std::cout << "Minimum segment times: ";
    for (const auto& T : T_opt_segments_) std::cout << T << " ";
    std::cout << ", Total time required: " << total_T << " seconds\n";

    auto [t_samples, trajectories, violations] = generateTrajectories();

    if (!violations.empty()) {
        std::cout << "Constraint Violations (unexpected):\n";
        for (const auto& v : violations) {
            std::cout << v << "\n";
        }
    } else {
        std::cout << "All constraints satisfied.\n";
    }

    std::vector<std::vector<double>> joint_angles(num_joints_);
    for (int j = 0; j < num_joints_; ++j) {
        for (int seg = 0; seg < num_segments_; ++seg) {
            const auto& [pos, vel, acc, coeffs] = trajectories[j][seg];
            joint_angles[j].insert(joint_angles[j].end(), pos.begin(), pos.end());
        }
    }

    return {violations.empty(), joint_angles};
}