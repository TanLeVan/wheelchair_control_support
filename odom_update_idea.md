## Virtual obstacle point
- Virtual obstacle point contain value indicating if the point is an obstacle or not (ranging from 0 to 1)
- When wheelchair move, virtual obstacle point is transformed relative to the wheelchair position and mapped to a cell in the Map
- Each cell can contain multiple virtual obstacle point. Value of cell is the value of the virtual obstacle with largest value

## HOw virtual obstacle point is created, updated and delete
- if cell contain no virtual obstacle, a new virtual osbtacle is made at the middle of the cell
- When updating the value of the cell, the value of virtual obstacle is updated according to probabilisic rule.  
- Virtual object out of bound is deleted

## potential prolem
1. During transformation, when virtual obstacle is at the edge of cell, potentially there will be cell with temporary no obstacle. THIS PROBLEM IS COFIRMED
Initialization of new virtual obstacle will cause the number of obstacle to increase overtime. The increase of virtual point pose a problem to the calculation speed. Whether or not that many point is needed to track the movement of the cell. Mechanism to reduce the number of obstacle?
2. 

## Test
- How to test for potential problem 1:

## Solution
1. Limit the number of virtual  obstacle within the cell. When the virtual obstacle within a cell excceed a certain threshhold, delete the 

## NEED TO DO
- Create a seperate map constructor to update map


## What is the problem that I am trying to solve
Smooth transition from one cell to another. The initial idea is to use a point within a cell to track for wheelchair movement. This approach face a problem:
- When virtual object is at the boundary of cell, virtual object can overlap to one cell, causing the value of that cell being unclear.
- The overlappping problem also create situation where some cell do not have any virtual object temporary (HAS BEEN CONFIRM)
- One concern is the overlapping of virtual point in the outer cell, where the cell size is larger.

- If allow one cell to have multiple virtual obstacle, with the mechanism of creating new obstacle for cell without 