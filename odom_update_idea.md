## Virtual obstacle point
- Virtual obstacle point contain value indicating if the point is an obstacle or not (ranging from 0 to 1)
- When wheelchair move, virtual obstacle point is transformed relative to the wheelchair position and mapped to a cell in the Map
- Each cell can contain multiple virtual obstacle point. Value of cell is the value of the virtual obstacle with largest value

## HOw virtual obstacle point is created, updated and delete
- if cell contain no virtual obstacle, a new virtual osbtacle is made at the middle of the cell
- When updating the value of the cell, the value of virtual obstacle is updated according to probabilisic rule.  
- Virtual object out of bound is deleted

## potential prolem
1. During transformation, when virtual obstacle is at the edge of cell, potentially there will be cell with temporary no obstacle. Create new obstacle?

## Test
- How to test for potential problem 1:
    - 

## NEED TO DO
- Create a seperate map constructor to update map
