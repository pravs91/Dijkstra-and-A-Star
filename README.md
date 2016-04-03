# BFS, Dijkstra and A-Star graph algorithms
Implemented Breadth-First-Search, Dijsktra and A-star graph search algorithms to find the shortest path between two nodes on a directed graph. 

## Description of classes
* ```GraphNode``` - represents a node in a graph. Includes properties like ```coordinate```, ```distToStart``` and ```distToGoal``` with getters and setters
* ```GraphEdge``` - represents a weighted edge in a graph. Each edge is equivalent to a road, with properties like ```roadName```, ```roadType```,```length```
and ```speedLimit```.
* ```MapGraph``` - class that implements the algorithms.

## Features
* The user can get the shortest path between two nodes based on distance or shortest time
* Each edge is given a ```speedLimit```, based on its ```roadType```
* The ```speedLimit``` varies based on the time of the day, which helps model peak traffic conditions
