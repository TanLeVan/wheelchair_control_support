# Idea testing
## Testing clusterting based method for outlier detection and removal
- DBSCAN
- OPTICS
....


- Use Hough transformation to detect line
- Use DBSCAN to cluster the rest of the point
- Outliar do not belong to line or cluster

## Idea 2
- DBSCAN to find cluster
- Within a cluster choose a few points each, merge with outliers to form a second scan
- Perform hough on the second scan to find lines. 
Drawback:
- Hough is slow
- Noise point causes the detected line to be inaccurate

## Idea 3
- DBSCAN to find cluster
- Subsample the cluster
- HOUGH transform 
Drawback
- Hough is not robust to noise. Tuning hough is difficult. Hough is slow

## Idea 4 
- Extended line tracking (An Extended Line Tracking Algorithm, Leonardo Romeo Munoz)
- Chatgpt prompt:
Create a ros2 node that subscribe to a scan message and find the lines in the scan message. The node should have the following functionality:
- Subscribe to scan message
- Apply Line tracking algorithm sequentially to detect line in the scan
    - First apply LineTracking function
    - Then apply Backtracking function
- Visualize all line detected and the points contribute to the line with the same color.


- A line is represented by (r, theta). Point (x,y) on the line sastify r = x cos(theta) +  y sin(theta)
- Distance from point(x,y) to line (r, theta) is represented by:
d = r - xcos(theta) - ysin(theta)
- From n points (xi, yi) the Least Square fitting line is 
r = x_bar cos(theta) + y_bar sin (theta)
theta = 1/2 arctan(-2 Sxy / Syy - Sxx)
where x_bar = mean of xi, y_bar = sum of yi, Sxx = sum of (xi - x_bar)^2, Syy = sum of (yi - y_bar)^2, Sxy = sum of (x-x_bar)(y-y_bar)

LineTracking function: Detect line from scan
Algorithm 1 Line Tracking Algorithm
INPUT: Z, a sequence of points (z1, . . . ,zN)and Tmax
OUTPUT: Θ, a sequence of lines (θ1, .. . , θM)and I, a
sequence of indexes for Θ
i←1,j←1,l←1,Θ←(),I←()
while j < N −2do
θl←best line that ﬁts (zi, . . . , zj+1)// (eq. 3)
T←d(zj+2, θl)// (eq. 2)
if T > Tmax then
Add θlto Θ
Add hi, j + 1ito I// indexes of the line θl
l←l+ 1
i←j+ 2
j←i
else
j←j+ 1
end if
end while
return Θand I

BackTracking function:
Algorithm 2 Backtracking Algorithm
INPUT: Z(z1, . . . ,zN), Tmax,Θ(θ1, . . . , θM) and I
OUTPUT: Θand I
for i= 1, . . . , M −1do
hs1, e1i ← I(i)// indexes of line θi
hs2, e2i ← I(i+ 1) // indexes of line θi+1
repeat
d2←d(ze1, θi+1)
if e1−s1+ 1 >2then
θ0
i←best line that ﬁts (zs1, . . . , ze1−1)
d1←d(ze1, θ0
i)// θ0
idoes not include ze1
else
d1=Tmax
end if
if d2< d1then
e1←e1−1,s2←s2−1// adjust indexes
Update indexes of lines θiand θi+1 in I
recompute lines θiand θi+1 and update Θ
else
break repeat
end if
until s1=e1
end for
Delete from Θ and I, lines with no points
return Θ and I

## Conclusion
DBSCAN to find cluster
LT with cluster to find line
Check for point that not in cluster but belong to line
The rest are outliers

# Code pretty
## Requirement
- Subscribe to /scan message, publish a filtered /filtered_scan message
- Perform DBSCAN on /scan. Points is classified into clustered
    Parameter:
    - DBSCAN: eps, min_sample
    - 

- For each cluster, apply Line Tracking (LT) + Backtracking algo for points inside cluster. Find lines from cluster.
For line in detected lines, calcualate distance of line to point in outliers. Point in outlier that are close to line are considered to be in the same 
cluster
    Parameter:
        - point threshold: Line with no of points < than theshold are ignored.
        - Cluster with no of point < threshold is not considered?
    
- Remaining outliers are true outlier

- Visualization.
    - Function to visualize group of point with the same color
    - Function to visualize line that input 2 point and draw line between point. Can choose color of line
    - Use the above 2 function. Visualize point of the same cluster and line found in that cluster to be of the same color. Use  plt.cm.get_cmap('Set2',)
    - Visualize set of outliers point in red color