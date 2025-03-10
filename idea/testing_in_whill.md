## Testing
[o] Test gap finding algorithm:
    [o] Test scan filter:
        - Scan filter works ok
    [o] Test find gap:
        - Find gap behave weirdly. In the visualization, the two end of gap is in place of non-existing scan point
    [o] Test find gap with rosbag:
        - Run rosbag for /Wheelchair_support/test_scan/merge_scan_bag and run test_find_gap2
        - See the behavour
        - Speculation: Looping through /merge scan do not behave correctly. Need to write seperate script to subscribe
        => The min and max angle of scan range [-pi, pi], causing wrong visualization in rviz. The current calculation of x,y coordinate assume
        angle from [0,2pi] [Solved]

        - Some times seg fault occur? investigating [Solve]

[o] Test intention estimation 
    [o] Find the subscription period of odom and joystick topic: 0.01~0.012s:
        - The period is too small. Subscription rate to large (solve with topic_tools)
    [o] laser_scan_callbacl cause seg fault
        - error in the value_function()
    => the program does run, but the result is bad. What is the condition:
        [ ] Gap finding is unstable due to noisy sensor
        [ ] Confident in desired gap of the user is extremely high, which is not desirable. The confident in one gap should be reasonably low
        [ ] Do not predict correctly the intended gap 
        The problem of high confident and do not predict correcly is correlated. Speculation
            - Bad design of cost function.
            - The confident is normalized accross all available gap. This mean if only one gap, then the confident for that gap is very high.
            At the moment there is no mechanism for alternative selection, that is no gap is desired.
        

Proposed solution to improve cost function.
    [ ] Visualize how the confident level increase and decrease for one gap given a trajectory and input
    [ ] Add something (?) to lower the confidence of gap and account for the case where use do not want to go to gap
    [o] Use softmin instead of mmin to calculate value funcntion
        - Softmin function cause the value function at arbitrary state to be negative, with unfavorable state less negative than favorable state. 
        The expectation is that value function should be positive and this is the case when using hardmin but not soft min.
        - Opt for inverse the cost function
    [ ]Add more point to be the target goal in the gap instead of just the middle point   
    [0] Using a  more numerically stable calculation method 
    [ ] Chaning the cost function. The current cost function and state is not very appropriate considering gap intention
