# Idea for experiment
## What previous research set the goal for the experiment
Intention prediction path:
- Reduce execution time for the same task
- Metric:
    - Task completion time
    - Dynamic Time Wraping, which compare the similarity between real path and designated path

Shared control DWA:


HUman-wheelchair Collaboration (Yiannis Demiris):
- Execution time and Safety metric

## Preliminary test
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


