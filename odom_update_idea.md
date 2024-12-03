## Virtual obstacle point
- Virtual obstacle point contain value indicating if the point is an obstacle or not (ranging from 0 to 1)
- When wheelchair move, virtual obstacle point is transformed relative to the wheelchair position and mapped to a cell in the Map
- Each cell can contain multiple virtual obstacle point. Value of cell is the value of the virtual obstacle with largest value
- if cell contain no virtual obstacle, a new virtual osbtacle is made at the middle of the point
- When updating the value of the cell, the value of virtual obstacle is updated according to probabilisic rule. 

