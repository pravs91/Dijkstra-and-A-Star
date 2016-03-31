/**
 * @author Praveen Sankaranarayanan
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author Praveen Sankaranarayanan
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private int numVertices;
	private int numEdges;
	private int numNodesVisited;
	private static final double MAX_SPEED = 75.0;
	
	// Map between each GeographicPoint and its corresponding GraphNode, for faster retrieval.
	private Map<GeographicPoint, GraphNode> locationMap;
	
	// adjacency list between each GraphNode and a list of GraphEdge objects
	private Map<GraphNode, List<GraphEdge>> adjListMap;
	
	// map between road type and speed limit
	private Map<String,Double> speedMap;
	private Map<String,Double> peakTimeMap;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// initialize member variables
		this.numVertices = 0;
		this.numEdges = 0;
		this.locationMap = new HashMap<GeographicPoint,GraphNode>();
		this.adjListMap = new HashMap<GraphNode, List<GraphEdge>>();
		this.speedMap = new HashMap<String,Double>();
		this.peakTimeMap = new HashMap<String,Double>();
		
		// assign speed limit for different road types during NON-PEAK hours
		speedMap.put("motorway", 70.0);
		speedMap.put("motorway_link", 60.0);
		speedMap.put("primary", 55.0);
		speedMap.put("secondary", 45.0);
		speedMap.put("tertiary", 35.0);
		speedMap.put("residential", 25.0);
		speedMap.put("unclassified", 15.0);
		
		// assign speed limit for different road types during PEAK hours		
		peakTimeMap.put("motorway", 45.0);
		peakTimeMap.put("motorway_link", 40.0);
		peakTimeMap.put("primary", 35.0);
		peakTimeMap.put("secondary", 30.0);
		peakTimeMap.put("tertiary", 35.0);
		peakTimeMap.put("residential", 25.0);
		peakTimeMap.put("unclassified", 15.0);
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		// return the keySet of locationMap, which is of type Set<GeographicPoint>
		return new HashSet<GeographicPoint>(locationMap.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{	// validate input argument
		if(location == null || locationMap.containsKey(location)){
			return false;
		}
		// Create a new GraphNode corresponding to 'location' and add node to locationMap.
		GraphNode node = new GraphNode(location);
		locationMap.put(location, node);
		// Add node to adjListMap with empty edges list
		adjListMap.put(node, new ArrayList<GraphEdge>());
		numVertices++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		// validate all arguments
		if(from == null || to == null || roadName == null || roadType == null || length < 0){
			throw new IllegalArgumentException();
		}
		// check if from and to are already in graph
		if(!locationMap.containsKey(from) || !locationMap.containsKey(to)){
			throw new IllegalArgumentException();
		}
		GraphNode src = locationMap.get(from);
		GraphNode dest = locationMap.get(to);
		
		// add edge to List in adjacency list for src, use current time to add speed_limit

		Calendar now = Calendar.getInstance();
		int day = now.DAY_OF_WEEK;
		int hour = now.getTime().getHours();
		boolean peakHours = (day > 1 && day < 5) && (hour >7 && hour <10 || (hour>16 && hour<19));
		double speed;
		if(speedMap.containsKey(roadType)){
			speed = peakHours ? peakTimeMap.get(roadType) : speedMap.get(roadType);	
		} else {
			speed = 20.0;
		}
		
		double time = (length/ speed) * 60; // minutes
//		System.out.println("Speed: " + speed + " length: " + length + " time: " + time);
		(adjListMap.get(src)).add(new GraphEdge(src,dest,roadName,roadType,length,speed,time));
		numEdges++;
	}
	
	/**
	 * Method to return a String representation of the Graph
	 * @return string representation of graph, with each node and its edges
	 */
	public String toString(){
		String s = "\nGraph with " + numVertices + " vertices and " + numEdges + " edges.\n";
		if(getNumVertices() > 20){
			s += "Graph is too huge to print (>20 vertices).\n";
			return s;
		}
		// Iterate through adjacency list
		// For each node, print the coordinates for itself and its connected edges' node.
		for(GraphNode node : adjListMap.keySet()){
			s += node.toString() + " : [ ";
			for(GraphEdge edge: adjListMap.get(node)){
				s += edge.getDest().toString() + " ";
			}
			s += " ]\n";
		}
		return s;
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// helper method to do the actual algorithm
		return doSearch(start,goal,nodeSearched,"BFS");
	}

	
	public List<GeographicPoint> doSearch(GeographicPoint start, 
		     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,String type){
		if(start == null || goal == null){
			System.out.println("start or goal is null");
			return null;
		}
		// check if start and goal are in the Graph
		if(!locationMap.containsKey(start) || !locationMap.containsKey(goal)){
			System.out.println("start or goal is not in the graph");
			return null;
		}
		// a map linking each node in the visited path to its parent
		HashMap<GraphNode, GraphNode> parentMap = new HashMap<GraphNode, GraphNode>();
		// check if a path exists using helper method that does BFS
		boolean pathExists = false;
		if(type.equals("BFS")){
			pathExists = implementBFS(locationMap.get(start),locationMap.get(goal),parentMap,nodeSearched);
		} else if(type.equals("Dijkstra")){
			pathExists = implementDijkstraOrAstar(locationMap.get(start),locationMap.get(goal),parentMap,nodeSearched,false, false);
		} else {
			pathExists = implementDijkstraOrAstar(locationMap.get(start),locationMap.get(goal),parentMap,nodeSearched,true, false);
		}
		if(!pathExists){
			System.out.println("No path found from start to goal.");
			return null;
		}
		// helper method to construct a list of GeographicPoint(s) in the path
		return constructPath(locationMap.get(start),locationMap.get(goal),parentMap);		
		
	}
	/**
	 * Helper method to find if a path exists from start to goal using BFS
	 * Implements the BFS algorithm discussed in video lecture
	 * @param start - starting node in the graph
	 * @param goal - goal node in the graph
	 * @param parentMap - map linking each node in the path to its parent
	 * @return true if a path exists, false otherwise
	 */
	private boolean implementBFS(GraphNode start, GraphNode goal,HashMap<GraphNode,
							  GraphNode> parentMap, Consumer<GeographicPoint> nodeSearched){
		boolean found = false;
		numNodesVisited = 0;
		Set<GraphNode> visited = new HashSet<GraphNode>(); // visited set
		Queue<GraphNode> toSearch = new LinkedList<GraphNode>();// queue to explore
		toSearch.add(start);
		visited.add(start);
		while(!toSearch.isEmpty()){
			GraphNode curr = toSearch.remove();
			nodeSearched.accept(curr.getCoordinate());
			numNodesVisited++;
			if(curr == goal){
				found = true;
				break;
			}
			// get the edges for the node being visited
			for(GraphEdge edge: adjListMap.get(curr)){
				// getDest returns the GraphNode for each edge's destination i.e. neighbor
				GraphNode neighbor = edge.getDest();
				if(!visited.contains(neighbor)){
					visited.add(neighbor);
					parentMap.put(neighbor, curr); // add curr as parent to neighbor
					toSearch.add(neighbor);
				}
			}
		}
		return found;
	}
	
	/**
	 * Construct the path from start to goal
	 * @param start - start node
	 * @param goal - goal node
	 * @param parentMap - map linking each node in the path to its parent
	 * @return List of GeographicPoint objects in the path
	 */
	private List<GeographicPoint> constructPath(GraphNode start, GraphNode goal,HashMap<GraphNode,GraphNode> parentMap){
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GraphNode curr = goal;
		// add each node to the head of the list, starting from goal
		while(curr != start){
			path.addFirst(curr.getCoordinate());
			curr = parentMap.get(curr); // move curr to parent
		}
		path.addFirst(curr.getCoordinate()); // add start
		return path;
	}
	
	/**
	 * Print the path
	 * @param path
	 */
	public void printPath(List<GeographicPoint> path){
		if(path == null){
			return;
		}
		for(GeographicPoint pt: path){
			System.out.print("(" + pt.getX() + ", " + pt.getY() + ") -> ");
		}
	}
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
		return doSearch(start,goal,nodeSearched,"Dijkstra");
	}

	/**
	 * 
	 * @param start starting graph node
	 * @param goal target graph node
	 * @param parentMap map linking each node in the path to its parent
	 * @param nodeSearched - hook up for visualization
	 * @param isAstar - boolean value to specify which algorithm to use
	 * @return true if a path exists from start to goal
	 */
	public boolean implementDijkstraOrAstar(GraphNode start, GraphNode goal,HashMap<GraphNode,
			  GraphNode> parentMap, Consumer<GeographicPoint> nodeSearched,boolean isAstar, boolean shortestTime){
		// follow course video algorithm
		numNodesVisited = 0;
		boolean found=false;
		PriorityQueue<GraphNode> pQueue = new PriorityQueue<GraphNode>();
		Set<GraphNode> visited = new HashSet<GraphNode>(); // visited set
		// initialize all nodes' distances to +INF
		for(GraphNode node: adjListMap.keySet()){
			node.setDistToStart(Double.POSITIVE_INFINITY);
			node.setDistToGoal(Double.POSITIVE_INFINITY);
		}
		start.setDistToStart(0);
		start.setDistToGoal(0);
		pQueue.add(start);
		//System.out.println("visit order:");
		while(!pQueue.isEmpty()){
			GraphNode curr = pQueue.remove();
			numNodesVisited++;
			if(!visited.contains(curr)){
				if(!isAstar){
					curr.setDistToGoal(0); // use distToGoal 0 for Dijkstra
				}
				else {					
					if(shortestTime){
						curr.setDistToGoal(curr.getCoordinate().distance(goal.getCoordinate())/MAX_SPEED);
					} else{
						curr.setDistToGoal(curr.getCoordinate().distance(goal.getCoordinate())); // distance from curr to goal
					}
				}
				visited.add(curr);
				//System.out.println("(" + curr.getCoordinate().getX() + ", " + curr.getCoordinate().getY() + ") ");
				nodeSearched.accept(curr.getCoordinate());
				if(curr == goal){
					found = true;
					break;
				}
				for(GraphEdge edge: adjListMap.get(curr)){
					GraphNode neighbor = edge.getDest();
					if(!isAstar){
						neighbor.setDistToGoal(0); // set distToGoal to 0 for Dijkstra for each neighbor
					}
					else { 
						if(shortestTime){ // use time as metric
							neighbor.setDistToGoal(neighbor.getCoordinate().distance(goal.getCoordinate())/MAX_SPEED);
						} else{
							// use straight line distance to goal
							neighbor.setDistToGoal(neighbor.getCoordinate().distance(goal.getCoordinate()));
						}
					}
					double factor;
					if(shortestTime){
						factor = edge.getTime();
					} else{
						factor = edge.getLength();
					}
					if(curr.getDistToStart() + factor < neighbor.getDistToStart()){
						neighbor.setDistToStart(curr.getDistToStart() + factor);
						parentMap.put(neighbor, curr);
						pQueue.add(neighbor);
					}
				}
			}
		}
		System.out.println("Number of nodes visited: " + numNodesVisited);
		return found;
	}
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
		public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
		return doSearch(start,goal,nodeSearched,"Astar");
	}	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/maps/ucsd.map", theMap);
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		System.out.println(theMap);
		
		System.out.println("BFS:");
		theMap.printPath(theMap.bfs(new GeographicPoint(1.0,1.0), new GeographicPoint(8.0,-1.0)));
		theMap.printPath(theMap.bfs(new GeographicPoint(4.0,2.0), new GeographicPoint(8.0,-1.0)));
		theMap.printPath(theMap.bfs(new GeographicPoint(4.0,-1.0), new GeographicPoint(7.0,3.0)));
		theMap.printPath(theMap.bfs(new GeographicPoint(1.0,1.0), new GeographicPoint(5.0,-1.0)));
		theMap.printPath(theMap.bfs(new GeographicPoint(1.0,1.0), new GeographicPoint(5.0,1.0)));
		System.out.println();
		System.out.println("Dijkstra:");
		theMap.printPath(theMap.dijkstra(new GeographicPoint(1.0,1.0), new GeographicPoint(4.0,1.0)));
		theMap.printPath(theMap.dijkstra(new GeographicPoint(1.0,1.0), new GeographicPoint(8.0,-1.0)));
		theMap.printPath(theMap.dijkstra(new GeographicPoint(4.0,2.0), new GeographicPoint(8.0,-1.0)));
		System.out.println();
		System.out.println("Astar");
		theMap.printPath(theMap.aStarSearch(new GeographicPoint(1.0,1.0), new GeographicPoint(4.0,1.0)));
		theMap.printPath(theMap.aStarSearch(new GeographicPoint(1.0,1.0), new GeographicPoint(8.0,-1.0)));
		theMap.printPath(theMap.aStarSearch(new GeographicPoint(4.0,2.0), new GeographicPoint(8.0,-1.0)));
		
//		MapGraph map2 = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/hollywood_small.map", map2);
		//System.out.println(map2);
		

		//MapGraph theMap = new MapGraph();
		Calendar now = Calendar.getInstance();
		System.out.println(now.getTime().getHours());
		System.out.println(now.DAY_OF_WEEK);
		System.out.println(now.HOUR_OF_DAY);
		
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
