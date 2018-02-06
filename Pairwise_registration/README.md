# This program uses the Iterative Closest Point algorithm in order to incrementally register a series of point clouds two by two.

The idea is to transform all the clouds in the first cloud’s frame.
This is done by finding the best transform between each consecutive cloud, and accumulating these transforms over the whole set of clouds.
