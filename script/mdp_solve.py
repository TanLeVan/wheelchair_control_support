import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import json

# Ensure proper backend is used
matplotlib.use('TkAgg')  # or 'Qt5Agg' depending on your environment

# Save to file path
saving_path = "../data/value_function_temp.json"

# Cost function parameter
w1 = 1.0  # weight for theta
w2 = 2.0  # weight for distance cost Cd
zeta = 1.0  # constant cost
r = 2.0  # reference distance for Cd

# Parameter to calculate value function
v_max = 0.3  # max linear velocity
v_min = -0.3
number_of_linear_vel = 12 #difference between each discretized linear velocity
omega_max = 1.6  # max angular velocity
number_of_angle_vel = 12 # difference between each discretized angular velocity
discount_factor = 0.9  # discount factor
delta_t = 0.1# time step
max_distance = 2.0  #furthest distance from goal to be considered
number_of_distance_point = 200
max_abs_angle_distance = np.pi #furthest angular distance to be considered (in both direction) 
number_of_angle_point = 50
max_iterations = 1000  # maximum iterations for value iteration
tolerance = 0.1  # convergence threshold
max_steps = 1000  # maximum steps for the trajectory

# Discretize state space
d_values = np.linspace(0, max_distance, number_of_distance_point)  
theta_values = np.linspace(-max_abs_angle_distance, max_abs_angle_distance, number_of_angle_point)  
v_values = np.linspace(v_min, v_max, number_of_linear_vel)  # linear velocities to consider
omega_values = np.linspace(-omega_max, omega_max, number_of_angle_vel)  # angular velocities to consider

# Initialize value function V(d, theta) to zeros
V = np.zeros((len(d_values), len(theta_values)))

# Cost function
def cost(d, theta):
    if d > r:
        Cd = zeta
    else:
        Cd = (zeta * d) / r
    return (w1 * abs(theta) + w2 * Cd)

# Transition function
def transition(d, theta, v, omega, d_values, theta_values):
    # Compute the next state (d', theta')
    d_next = np.sqrt(max(d**2 - 2 * d * np.cos(theta) * v * delta_t + (v**2 * delta_t**2), 0))  # ensure non-negative distance
    theta_next = theta - omega * delta_t
    
    # Wrap theta_next to be within [-pi, pi]
    theta_next = np.mod(theta_next + np.pi, 2 * np.pi) - np.pi

    # Find the nearest neighbor in discretized state space
    i_next = np.argmin(np.abs(d_values - d_next))  # Closest index for d
    j_next = np.argmin(np.abs(theta_values - theta_next))  # Closest index for theta
    
    return i_next, j_next

# Value Iteration with nearest neighbor transition
for iteration in range(max_iterations):
    delta = 0
    for i, d in enumerate(d_values):
        for j, theta in enumerate(theta_values):
            # Skip the terminal state at d = 0
            # if d == 0:
            #     continue

            # Current value
            v_current = V[i, j]
            # min_value = float('inf')
            
            # Iterate over all possible actions (v, omega)
            # Update the value function using softmin
            integral = 0
            Q_values = []
            for v in v_values:
                for omega in omega_values:
                    # Compute next state
                    i_next, j_next = transition(d, theta, v, omega, d_values, theta_values)
                    
                    # Bellman update
                    Q_xu = cost(d, theta) + discount_factor * V[i_next, j_next]
                    Q_values.append(Q_xu)
                    # min_value = min(min_value, Q_xu)
            
            # Update value function
            Q_values = np.array(Q_values)
            min_Q = np.min(Q_values)
            log_integral = - min_Q + np.log(np.sum(np.exp(-(Q_values - min_Q))))
            V[i, j] = - log_integral
            # V[i,j] = min_value
            delta = max(delta, abs(v_current - V[i, j]))
    
    # Print the iteration progress
    print(f"Iteration {iteration + 1}: Max delta = {delta:.6f}")
    
    # Check for convergence
    if delta < tolerance:
        print(f"Value iteration converged after {iteration + 1} iterations")
        break

# Save value function to csv file
# np.savetxt(saving_path, V, delimiter=",")

# Save value function to Json file
metadata = {
    "discount_factor" : discount_factor,  # discount factor
    "delta_t" : delta_t,  # time step
    "max_distance" : max_distance,  #furthest distance from goal to be considered
    "number_of_distance_point" : number_of_distance_point,
    "max_abs_angle_distance" : max_abs_angle_distance, #furthest angular distance to be considered (in both direction) 
    "number_of_angle_point" : number_of_angle_point
}
cost_function_param ={
    "angle_weight" : w1,
    "distance_weight" : w2,
    "distance_threshold" : r,  # beyond which the cost for distance do not change
    "zeta" : zeta, # steepness of distance cost
}
data = {
    "cost_function_param" : cost_function_param,
    "metadata": metadata,
    "matrix": V.tolist()
}
with open(saving_path, "w") as f:
    json.dump(data, f, indent=4)  # Pretty-print with indentation
print(f"Saved to {saving_path}")

# Visualization of Value Function with heatmap
plt.ion()  # Ensure interactive mode is on
d_mesh, theta_mesh = np.meshgrid(d_values, theta_values)
plt.contourf(d_mesh, theta_mesh, V.T, cmap='coolwarm', levels=100)
plt.colorbar(label='Value')
plt.xlabel('Distance to goal (m)')
plt.ylabel('Orientation (rad)')
plt.title('Optimal Value Function with Heatmap')

# Random initial state
d_init = np.random.choice(d_values[1:])  # Random initial distance (exclude d=0)
theta_init = np.random.choice(theta_values)  # Random initial orientation
i_init = np.argmin(np.abs(d_values - d_init))
j_init = np.argmin(np.abs(theta_values - theta_init))

# Compute the trajectory of optimal actions starting from the random initial state
d_traj = [d_init]
theta_traj = [theta_init]

i, j = i_init, j_init
step = 0  # Initialize step counter
while d_traj[-1] > 0 and step < max_steps:  # Continue until reaching the terminal state d = 0 or max steps reached
    min_value = np.inf
    best_v, best_omega = 0, 0
    
    # Find the optimal action
    for v in v_values:
        for omega in omega_values:
            i_next, j_next = transition(d_values[i], theta_values[j], v, omega, d_values, theta_values)
            value = cost(d_values[i], theta_values[j]) + discount_factor * V[i_next, j_next]
            if value < min_value:
                min_value = value
                best_v, best_omega = v, omega
    
    # Apply the optimal action to get the next state
    i_next, j_next = transition(d_values[i], theta_values[j], best_v, best_omega, d_values, theta_values)
    
    # Append the new state to the trajectory
    d_traj.append(d_values[i_next])
    theta_traj.append(theta_values[j_next])
    
    # Update the indices
    i, j = i_next, j_next
    step += 1  # Increment step counter

# Plot the trajectory with arrows
plt.quiver(d_traj[:-1], theta_traj[:-1], np.diff(d_traj), np.diff(theta_traj), scale=1, scale_units='xy', color='black')

# Show the plot
plt.show(block=True)  # Ensure the plot stays open

