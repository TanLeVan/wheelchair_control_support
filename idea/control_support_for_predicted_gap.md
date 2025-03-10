If found a gap with confident > threshold,  then what to do:
## Idea 1: Adding a actractor field from the middle point of gap
At the moment, cost function is:
C =  (v-v_u)^2 + (w - w_u)^2
Choose (v,w) to minimize C

Now. Give a gap with confident > threshold => At a given state, the optimal (v,w) to reach the gap can be calculated using Q*(). However, then the 
share_control_node need to have information about Q*().
The simplest method should only require shared control node to know about the goal (x,y, theta_g)A share control scheme that:
- use a weighted sum of user's joystick input and best trajectory to go toward the goal can be employed.
- Use a cost function that take into account the goal (similar to DWA cost function):
    C(v,u) = w_u * C_u + w_gap *C_gap
    in which;
    C_u = (v-v_u)^2 + (w - w_u)^2
    C_gap = w_a* heading different of robot future position and goal + w_d * distance diferent to the goal

Future is how much future?


How to send desired gap to controller? (DONE)
- Use topic to sent gap with the largest confident. 
- share control node subscribe to gap topic
- If gap confident > threshold, trigger gap support mode
- Else nothing
- Obstacle avoidance mode always run.

### Pseudo code:
If joy_vel = 0
    cmd_vel <- 0

else
    for vel_pair in vel_pair_list:
        traj <- generate_trajectory(vel_pair) 
        if (traj collide)
            continue
        else
            if gap_confident >= 0:
                min_cost = VERY LARGE 
                traj_cost = cost_function(traj)
                if (traj_cost < min_cost)
                {
                    best_traj = traj;
                    min_cost = traj_cost
                    cmd_vel_.linear.x = vel_pair.first;
                    cmd_vel_.angular.z = vel_pair.second;
                }

            else if gap_confident < 0:
                min_vel_distance = VERY LARGE
                if(hypot(vel_pair.first - joy_vel_.linear.x, vel_pair.second - joy_vel_.angular.z) < min_vel_distance)
                {
                    best_traj = traj;
                    min_vel_distance = hypot(vel_pair.first - joy_vel_.linear.x, vel_pair.second - joy_vel_.angular.z);
                    cmd_vel_.linear.x = vel_pair.first;
                    cmd_vel_.angular.z = vel_pair.second;
                }

double cost_function(vel_pair, traj, user_vel, gap):
    C_u = hypot(vel_pair.first - joy_vel_.linear.x, vel_pair.second - joy_vel_.angular.z)
    yaw += period_ * yaw_rate;
    x += linear_vel * std::cos(state.yaw_) * period_;
    y += linear_vel * std::sin(state.yaw_) * period_; 
    angle_diff = atan2(gap.mid_x, gap.mid_y) - yaw;
    distance_diff = hypot(x - gap.mid_x, y -gap.mid_y);
    return w_u *C_u + w_a * angle_diff + w_d * distance_diff


