TriangleBox.js
==============

Implements collision between triangles and AABBs through the separating axis theorem (SAT).

The function provides not only a boolean test (does the AABB overlap the triangle?) but also a least translation vector that solves the collision by moving the box to a non-overlapping position with the minimum possible displacement.

This code was written with the aid of the book "Real-time collision detection" by Christer Ericson. It's a rather good book. See page 169 for AABB-triangle.

The main difference between this solution and the one in the book is that this one normalizes the axes before calculating distances to give you the least translation vector, this makes this solution much more useful for game development.

See live interactive demo at https://dcbc94ebc2bed8001aa8ecb83eb9ddcaa9d17090.googledrive.com/host/0B9scOMN0JFaXb0Q1VUhCWUswRUE/test.html
