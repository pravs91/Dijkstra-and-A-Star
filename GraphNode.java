package roadgraph;

import geography.GeographicPoint;

/**
 * 
 * @author Praveen Sankaranarayanan
 * A class that represents a vertex (node) in the graph
 *
 */
public class GraphNode implements Comparable<GraphNode> {
	private GeographicPoint coordinate;
	private double distToStart;
	private double distToGoal;
	
	public GraphNode(GeographicPoint location){
		this.coordinate = location;
	}
	
	public GeographicPoint getCoordinate(){
		return coordinate;
	}
	
	public double getDistToStart(){
		return distToStart;
	}
	
	public void setDistToStart(double distToStart){
		this.distToStart = distToStart;
	}
	
	public double getDistToGoal(){
		return distToGoal;
	}
	
	public void setDistToGoal(double distToGoal){
		this.distToGoal = distToGoal;
	}
	
	// String representation of GraphNode
	public String toString(){
		String s = "(" + coordinate.getX() + ", " + coordinate.getY() + ")";
		return s;
	}
	
	public int compareTo(GraphNode other){
		if((this.distToStart + this.distToGoal)< (other.distToStart + other.distToGoal)){
			return -1;
		}
		else if((this.distToStart + this.distToGoal) > (other.distToStart + other.distToGoal)){
			return 1;
		} else {
			return 0;
		}
	}
}
