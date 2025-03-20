## Problem
- Footprint TOO LARGE, hinder motion
- Support system intervene TOO AGGRESIVELY when intended gap is detected. This can be balanced by adjusting the weight of user and of the system => AGGRESSIVENESS make user not feeling in control.
- Sometimes the LIDAR just does not work very well
- EDGE CASE: Only one gap => eventhough user do not intend to go to that gap, system still try to steer toward
the gap

The edge case pose a question:
- AT WHAT MOMENT DO THE USER NEED SUPPORT?
    - At time when CLEARLY the user want to tranverse through the gap but their ACTION result in MINOR (NON-OPTIMAL CONTROL) ERROR
    ? How to indicate "CLEARLY" tranverse through gap 
    ? How to indicate NON-OPTIMAL CONTROL

    
- WHEN DO USER NOT NEED SUPPORT:
    - At time when user INDICATE DIFFERENT INTENTION THROUGH ACTION
    !! Maybe add a "no goal" situation, to handle the case where non of the available gap is the intended goal of the user

## Idea
- Adding a few more candidate point on the gap ?


## Done solution
- Add "No-gap" hypothesis to account for the case when user do not want to go toward any gap. Changing the cost
of gap such that the difference between the confidence of gap with highest confidence and the confidence on "no-gap" hypothesis
is taken into account to adjust the agressiveness of the support.

