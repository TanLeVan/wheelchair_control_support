## Theory
state: x_t
user input: u_t
Cost function: C_t(x_t, u_t)
Transition function: x_{t+1} = T(x_t. u_t, timestep)
Value function at a state: V*(x_t). Reference the precalculated value function
Optimal value-action function at a state and action: Q(x_t, u_t) = C_t(x_t, u_t) + gamma* V*(x_{t+1}) = C_t(x_t, u_t) + gamma* V*(T(x_t. u_t, timestep))
User policy (probability of user taking action u at state x given the goal) = exp(V*  - Q*):
Consider all past user policy: Product of user policy from past to present

## BE CAREFUL
- The TIMESTEP used for transition function should be the same as when to calculate for the value function. Also that SHOULD be the same as the time step
between each record of /odom and /joystick topic
- Other parameters :
    - Cost function
    - Discount factor gamma
    - 

## Implemetation
Precalculate the value function and dump to a JSON (YAML) file to be read later. The file should contain the value function as
well as meta data such as the domain range of the action and state:
    - Action:
        - linear velocity v
        - angular velocity omega
    - State (currently):
        - distance toward gap
        - angle toward gap


Create a class GapIntentionEstimator():
- Input: - /odom topic
         - /joystick topic
         - /scan topic
         - Available gaps (HOW ??). I can have a FindGap() object as a member of GapIntentionEstimator to 
         directly find avaliable gap. No need for input from external source.
         - Value funtion: I can read a JSON file that contain the value function of all the available states

- Output: A single gap that is most probable and have probability above a certain threshold. Publish custom Gap message

- What data do I have:
    - odom message for the position of the robot with respect to a fixed coordinate frame "/odom" frame
    - Joystick message 
    - List of potential Gaps at any point in time


- What procedure do I need:
    - Read from a csv file and dumped a value function into a matrix.(std::vector<std::vector<>>) (done)

    - Synchronize subscribtion to /odom and /joystick topic. Each /odom message need to have a corresponding /joystick message. 
    Add /odom and /joystick to a fixed length queue. (done)

    - Calculate the sequence of states given a goal position
    - Calculate value function from a state
    - Calculate action value function from a state and action
    - Calculate user policy from state and action
    - Calculate the logit from sequence of states and action
    - Calculate the probability of going to all goals.
    - Publish gap with the highest prediction that above certain threshold

- How the program should run:
    - If new /odom and /joy message is received, save to a queue and to form a trajectory with respect to a gap later
    - If new scan message is receive:
        - Find all gaps from scan message
        - Calculate logit of gaps
        - Calculate probability to each gaps


## Testing
[ ] Test gap finding algorithm:
    - Test scan filter
    - Test find gap
