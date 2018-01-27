# Motion-Planning
Multiple Robots motion planning application in a 2D space with obstacles. 
<p>
	In the recent years, there has been renewed interest and development of wheeled robots that can crawl and clean the wall. Let’s call a wall crawling mobile robot, an Autonomous Surface Vehicle (ASV). Note: This is a non-typical use of the term ASV; the “surface” in ASV usually refers to water surface; in our assignment, “surface” in ASV means wall surface.
</p>

<p>
To cut the cost of cleaning the interior of many of its new remarkable buildings, UQ-Facilities would like to use multiple of the aforementioned ASVs to clean the wall. Their idea is to have multiple ASVs connected by push brooms and let the robots move in synchrony, so that the broom will clean the area. They have developed the hardware mechanism for this, and are now struggling to coordinate these vehicles. In this assignment, your task is to help UQ-Facilities by developing a prototype software to coordinate multiple ASVs carrying push brooms.</p>
<p>
For this prototype, the problem is simplified as follows:
</p>
<ol>
1. Each ASV is modeled as a point, each broom as a straight linesegment, and the environment as a normalized 2D plane (i.e., a 2D Euclidean space of size [0,1]X[0,1]).
2. The environment is fully observable, deterministic, static, and continuous.
3. All obstacles in the environment have rectangular form, and are axis-
aligned.
4. The cost of moving from one configuration to another is the sum of the
straight-line distance traveled by each ASV.
</ol>
<p>
This program calculates a collision free path for the ASVs and brooms to move from a given initial configuration to a given goal configuration, while satisfying the following requirements:
</p>
<ol>
1. The length of each broom is fixed at 0.05 units in length.
2. The system forms a connected chain at all times. A connected chain means each ASV can be connected to at most two brooms and each end of
each broom is tied to an ASV.
3. The polygon formed by connecting the two ends of the connected chain
with a straight line segment is convex and have an area of at least πrmin2, where rmin = 0.007(n-1) and n is the number of ASVs.

4. The planned path is given as a sequence of positions (primitive steps) such that on each step, each individual ASV moves by a distance of at most 0.001 units.
</ol>