import numpy as np
import matplotlib.pyplot as plt

class QuinticTrajectoryPlanner:
    """Class for planning quintic polynomial trajectories for a multi-DOF manipulator with multiple waypoints."""
    
    def __init__(self, waypoints, v_waypoints, v_i, v_f, a_i, a_f, v_max, a_max, 
                 T_min=0.1, T_max=10.0, tolerance=0.01, max_reruns=5, num_samples=100):
        """
        Initialize the trajectory planner with joint parameters and constraints.
        
        Parameters:
        - waypoints: List of numpy arrays, each containing joint angles for all joints at a waypoint [theta_0, theta_1, ..., theta_n]
        - v_waypoints: List of numpy arrays, each containing velocities at waypoints (v_i for first, v_f for last)
        - v_i, v_f: Initial and final velocities for all joints (rad/s)
        - a_i, a_f: Initial and final accelerations for all joints (rad/s^2)
        - v_max, a_max: Maximum velocities and accelerations for all joints (rad/s, rad/s^2)
        - T_min, T_max: Initial search range for minimum time per segment (s)
        - tolerance: Precision for binary search (s)
        - max_reruns: Maximum number of times to double T_max
        - num_samples: Number of time samples per segment for plotting
        """
        self.waypoints = [np.array(w) for w in waypoints]
        self.v_waypoints = [np.array(v) for v in v_waypoints]
        self.v_i = np.array(v_i)
        self.v_f = np.array(v_f)
        self.a_i = np.array(a_i)
        self.a_f = np.array(a_f)
        self.v_max = np.array(v_max)
        self.a_max = np.array(a_max)
        self.T_min = T_min
        self.T_max = T_max
        self.tolerance = tolerance
        self.max_reruns = max_reruns
        self.num_samples = num_samples
        self.num_joints = len(waypoints[0])
        self.num_segments = len(waypoints) - 1
        self.T_opt_segments = None
        self.trajectories = []
        self.t_segment_starts = None
        
        # Validate inputs
        assert all(len(w) == self.num_joints for w in waypoints), "All waypoints must have same number of joints"
        assert len(v_waypoints) == len(waypoints), "v_waypoints must match number of waypoints"
        assert all(len(v) == self.num_joints for v in v_waypoints), "All v_waypoints must have same number of joints"
        assert len(v_i) == self.num_joints, "v_i must match number of joints"
        assert len(v_f) == self.num_joints, "v_f must match number of joints"
        assert len(a_i) == self.num_joints, "a_i must match number of joints"
        assert len(a_f) == self.num_joints, "a_f must match number of joints"
        assert len(v_max) == self.num_joints, "v_max must match number of joints"
        assert len(a_max) == self.num_joints, "a_max must match number of joints"

    def plan_quintic_trajectory(self, theta_i, theta_f, v_i, v_f, a_i, a_f, T, t_samples):
        """
        Plan a quintic polynomial trajectory for a single joint over one segment.
        
        Parameters:
        - theta_i, theta_f: Initial and final joint angles (rad)
        - v_i, v_f: Initial and final velocities (rad/s)
        - a_i, a_f: Initial and final accelerations (rad/s^2)
        - T: Segment duration (s)
        - t_samples: Array of time points for evaluation
        
        Returns:
        - pos, vel, acc: Position, velocity, acceleration trajectories
        - coeffs: Polynomial coefficients [a0, a1, a2, a3, a4, a5]
        """
        A = np.array([
            [1, 0, 0, 0, 0, 0],              # theta(0) = theta_i
            [0, 1, 0, 0, 0, 0],              # v(0) = v_i
            [0, 0, 2, 0, 0, 0],              # a(0) = a_i
            [1, T, T**2, T**3, T**4, T**5],  # theta(T) = theta_f
            [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],  # v(T) = v_f
            [0, 0, 2, 6*T, 12*T**2, 20*T**3]  # a(T) = a_f
        ])
        b = np.array([theta_i, v_i, a_i, theta_f, v_f, a_f])
        coeffs = np.linalg.solve(A, b)
        t = t_samples
        pos = (coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + 
               coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5)
        vel = (coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 
               4*coeffs[4]*t**3 + 5*coeffs[5]*t**4)
        acc = (2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 
               20*coeffs[5]*t**3)
        return pos, vel, acc, coeffs

    def check_constraints_all_joints(self, T_segments, num_samples=1000):
        """
        Check if velocity and acceleration constraints are satisfied for all joints across all segments.
        
        Parameters:
        - T_segments: List of durations for each segment (s)
        - num_samples: Number of time samples for evaluation per segment
        
        Returns:
        - is_feasible: True if all constraints are satisfied, False otherwise
        - max_vels, max_accs: Maximum velocities and accelerations for each joint across all segments
        """
        is_feasible = True
        max_vels = np.zeros(self.num_joints)
        max_accs = np.zeros(self.num_joints)
        
        for seg in range(self.num_segments):
            t_samples = np.linspace(0, T_segments[seg], num_samples)
            theta_i = self.waypoints[seg]
            theta_f = self.waypoints[seg + 1]
            v_i_seg = self.v_waypoints[seg] if seg == 0 else self.v_waypoints[seg]
            v_f_seg = self.v_waypoints[seg + 1]
            a_i_seg = self.a_i if seg == 0 else np.zeros(self.num_joints)
            a_f_seg = self.a_f if seg == self.num_segments - 1 else np.zeros(self.num_joints)
            
            for j in range(self.num_joints):
                pos, vel, acc, _ = self.plan_quintic_trajectory(
                    theta_i[j], theta_f[j], v_i_seg[j], v_f_seg[j], a_i_seg[j], a_f_seg[j], 
                    T_segments[seg], t_samples
                )
                max_vels[j] = max(max_vels[j], np.max(np.abs(vel)))
                max_accs[j] = max(max_accs[j], np.max(np.abs(acc)))
                if max_vels[j] > self.v_max[j] + 1e-6 or max_accs[j] > self.a_max[j] + 1e-6:
                    is_feasible = False
        
        return is_feasible, max_vels, max_accs

    def find_minimum_time(self):
        """
        Find the minimum total time by assigning durations to each segment that satisfy constraints.
        Uses binary search to find optimal duration for each segment.
        
        Returns:
        - T_opt_segments: List of optimal durations for each segment
        """
        current_T_max = self.T_max
        rerun_count = 0
        
        while rerun_count <= self.max_reruns:
            print(f"\nRerun {rerun_count}: Searching in range T_segment=[{self.T_min:.3f}, {current_T_max:.3f}]")
            T_opt_segments = []
            total_T_opt = 0
            
            # For each segment, find minimum feasible duration
            for seg in range(self.num_segments):
                low = self.T_min
                high = current_T_max
                T_opt_seg = high
                
                while high - low > self.tolerance:
                    mid = (low + high) / 2
                    # Create temporary T_segments with current segment duration as mid
                    T_segments = T_opt_segments + [mid] + [self.T_min] * (self.num_segments - len(T_opt_segments) - 1)
                    is_feasible, max_vels, max_accs = self.check_constraints_all_joints(T_segments)
                    if is_feasible:
                        T_opt_seg = mid
                        high = mid
                    else:
                        low = mid
                    # print(f"Segment {seg+1}, T={mid:.3f}: Feasible={is_feasible}, Max Vels={max_vels.round(3)}, Max Accs={max_accs.round(3)}")
                
                T_opt_segments.append(T_opt_seg)
            
            # Check feasibility of all segments together
            is_feasible, max_vels, max_accs = self.check_constraints_all_joints(T_opt_segments)
            if is_feasible:
                self.T_opt_segments = T_opt_segments
                total_T_opt = sum(T_opt_segments)
                print(f"Found feasible segment times: {T_opt_segments}, Total T_opt={total_T_opt:.3f} seconds")
                return T_opt_segments
            
            # If not feasible, double T_max and increment rerun count
            rerun_count += 1
            current_T_max *= 2
            print(f"Segment times {T_opt_segments} not feasible, doubling T_max to {current_T_max:.3f}")
        
        # After max reruns, return last T_opt_segments with warning
        print(f"Warning: Reached maximum reruns ({self.max_reruns}). Returning last T_opt_segments={T_opt_segments}, but it may not be feasible.")
        self.T_opt_segments = T_opt_segments
        return T_opt_segments

    def generate_trajectories(self):
        """
        Generate trajectories for all joints across all segments using optimal segment times.
        
        Returns:
        - t_samples: Array of time points across all segments
        - trajectories: List of lists, each containing (pos, vel, acc, coeffs) for each segment per joint
        - violations: List of constraint violation messages
        """
        if self.T_opt_segments is None:
            raise ValueError("Must find minimum segment times before generating trajectories.")
        
        trajectories = [[] for _ in range(self.num_joints)]
        violations = []
        t_samples_all = []
        current_t = 0
        
        self.t_segment_starts = [0]
        for T_seg in self.T_opt_segments:
            current_t += T_seg
            self.t_segment_starts.append(current_t)
        
        for seg in range(self.num_segments):
            t_samples_seg = np.linspace(0, self.T_opt_segments[seg], self.num_samples)
            t_samples_seg_global = t_samples_seg + self.t_segment_starts[seg]
            t_samples_all.extend(t_samples_seg_global)
            
            theta_i = self.waypoints[seg]
            theta_f = self.waypoints[seg + 1]
            v_i_seg = self.v_waypoints[seg] if seg == 0 else self.v_waypoints[seg]
            v_f_seg = self.v_waypoints[seg + 1]
            a_i_seg = self.a_i if seg == 0 else np.zeros(self.num_joints)
            a_f_seg = self.a_f if seg == self.num_segments - 1 else np.zeros(self.num_joints)
            
            for j in range(self.num_joints):
                pos, vel, acc, coeffs = self.plan_quintic_trajectory(
                    theta_i[j], theta_f[j], v_i_seg[j], v_f_seg[j], a_i_seg[j], a_f_seg[j], 
                    self.T_opt_segments[seg], t_samples_seg
                )
                trajectories[j].append((pos, vel, acc, coeffs))
                
                # Check constraints
                max_vel = np.max(np.abs(vel))
                max_acc = np.max(np.abs(acc))
                if max_vel > self.v_max[j] + 1e-6:
                    violations.append(f"Joint {j+1}, Segment {seg+1}: Max velocity {max_vel:.3f} rad/s exceeds limit {self.v_max[j]} rad/s")
                if max_acc > self.a_max[j] + 1e-6:
                    violations.append(f"Joint {j+1}, Segment {seg+1}: Max acceleration {max_acc:.3f} rad/s^2 exceeds limit {self.v_max[j]} rad/s^2")
        
        self.trajectories = trajectories
        return np.array(t_samples_all), trajectories, violations

    def plot_trajectories(self, save_path='joint_trajectories_multi_waypoints.png'):
        """
        Plot position, velocity, and acceleration trajectories for all joints across all segments.
        Include starting joint angles, waypoints, and end joint angles as scatter points in the position plot.
        Include a single legend at the bottom with 3 columns and vertical lines for segment boundaries.
        
        Parameters:
        - save_path: Path to save the plot
        """
        if not self.trajectories:
            raise ValueError("Must generate trajectories before plotting.")
        
        t_samples, _, _ = self.generate_trajectories()
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        
        # Plot trajectories with consistent colors
        colors = plt.cm.tab10(np.linspace(0, 1, self.num_joints))
        handles = []
        labels = []
        
        for j in range(self.num_joints):
            pos_all = []
            vel_all = []
            acc_all = []
            t_all = []
            
            for seg in range(self.num_segments):
                pos, vel, acc, _ = self.trajectories[j][seg]
                t_seg = np.linspace(self.t_segment_starts[seg], self.t_segment_starts[seg + 1], self.num_samples)
                pos_all.extend(pos)
                vel_all.extend(vel)
                acc_all.extend(acc)
                t_all.extend(t_seg)
            
            line, = ax1.plot(t_all, pos_all, color=colors[j])
            ax2.plot(t_all, vel_all, color=colors[j])
            ax3.plot(t_all, acc_all, color=colors[j])
            handles.append(line)
            labels.append(f'Joint {j+1}')
        
        # Plot waypoints as scatter points on position plot
        for j in range(self.num_joints):
            # Starting joint angles (first waypoint)
            ax1.scatter([self.t_segment_starts[0]], [self.waypoints[0][j]], 
                        color=colors[j], marker='o', s=100, label='Start' if j == 0 else None, zorder=5)
            
            # Intermediate waypoints
            for w in range(1, len(self.waypoints)-1):
                ax1.scatter([self.t_segment_starts[w]], [self.waypoints[w][j]], 
                            color=colors[j], marker='^', s=100, label='Intermediate' if j == 0 and w == 1 else None, zorder=5)
            
            # End joint angles (final waypoint)
            ax1.scatter([self.t_segment_starts[-1]], [self.waypoints[-1][j]], 
                        color=colors[j], marker='s', s=100, label='End' if j == 0 else None, zorder=5)
        
        # Add vertical lines for segment boundaries
        for t in self.t_segment_starts[1:-1]:
            ax1.axvline(t, color='k', linestyle='--', alpha=0.5)
            ax2.axvline(t, color='k', linestyle='--', alpha=0.5)
            ax3.axvline(t, color='k', linestyle='--', alpha=0.5)
        
        # Customize plots
        ax1.set_title('Joint Position')
        ax1.set_ylabel('Angle (rad)')
        ax1.grid(True)
        
        ax2.set_title('Joint Velocity')
        ax2.set_ylabel('Velocity (rad/s)')
        ax2.grid(True)
        
        ax3.set_title('Joint Acceleration')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Acceleration (rad/s^2)')
        ax3.grid(True)
        
        # Add single legend at the bottom, including trajectory and waypoint labels
        handles.extend([
            plt.Line2D([0], [0], color='k', marker='o', linestyle='None', markersize=10, label='Start'),
            plt.Line2D([0], [0], color='k', marker='^', linestyle='None', markersize=10, label='Intermediate'),
            plt.Line2D([0], [0], color='k', marker='s', linestyle='None', markersize=10, label='End')
        ])
        labels.extend(['Start', 'Intermediate', 'End'])
        # fig.legend(handles, labels, loc='lower center', ncol=6, bbox_to_anchor=(0.5, 0.02))
        fig.legend(handles, labels, loc='lower center', ncol=6)
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.2)  # Adjust to make space for legend
        
        # Save and show plot
        plt.savefig(save_path, dpi=600)
        plt.show()

    def run(self):
        """
        Execute the full trajectory planning process:
        - Find minimum segment times
        - Generate trajectories
        - Check constraints
        - Plot trajectories
        """
        # Find minimum segment times
        T_opt_segments = self.find_minimum_time()
        total_T = sum(T_opt_segments)
        print(f"Minimum segment times: {T_opt_segments}, Total time required: {total_T:.3f} seconds")
        
        # Generate and plot trajectories
        t_samples, trajectories, violations = self.generate_trajectories()
        
        # Print constraint violations
        if violations:
            print("Constraint Violations (unexpected):")
            for v in violations:
                print(v)
        else:
            print("All constraints satisfied.")
        
        # Plot trajectories
        self.plot_trajectories()

# Example usage
if __name__ == "__main__":
    # Parameters for 6-DOF manipulator with 4 waypoints (initial, two intermediates, final)
    num_joints = 6
    waypoints = [
        np.array([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]),  # Initial angles (rad)
        np.array([5.0, 15.0, 15.0, 20.0, 25.0, 30.0]),   # First intermediate angles (rad)
        np.array([3.5, 12.5, 7.0, 10.0, 12.0, 15.0]),    # Second intermediate angles (rad)
        np.array([2.0, 10.8, -0.5, 1.2, -0.3, 0.6])      # Final angles (rad)
    ]
    v_waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Initial velocities (rad/s)
        np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),  # First intermediate velocities (rad/s)
        np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),  # Second intermediate velocities (rad/s)
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # Final velocities (rad/s)
    ]
    v_i = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Initial velocities (rad/s)
    v_f = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Final velocities (rad/s)
    a_i = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Initial accelerations (rad/s^2)
    a_f = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Final accelerations (rad/s^2)
    random_values = np.random.uniform(1.0, 5.0, size=6)
    v_max = random_values
    print(f"v_max: {v_max}")
    a_max = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])  # Max accelerations (rad/s^2)
    
    # Initialize planner
    planner = QuinticTrajectoryPlanner(
        waypoints, v_waypoints, v_i, v_f, a_i, a_f, v_max, a_max,
        T_min=0.1, T_max=10.0, tolerance=0.01, max_reruns=5, num_samples=100
    )
    
    # Run the planner
    planner.run()
