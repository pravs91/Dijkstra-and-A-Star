package roadgraph;

/**
 * 
 * @author Praveen Sankaranarayanan
 * A class that represents an edge in the graph,
 * with 2 GraphNode objects and road properties.
 *
 */

public class GraphEdge {
	private GraphNode src;
	private GraphNode dest;
	private String roadName;
	private String roadType;
	private double length;
	private double speedLimit;
	private double time;
	
	public GraphEdge(GraphNode src, GraphNode dest,String roadName, String roadType, double length, double speed, double time){
		this.src = src;
		this.dest = dest;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		this.speedLimit = speed;
		this.time = time;
	}
	
	public GraphNode getSrc(){
		return src;
	}
	
	public GraphNode getDest(){
		return dest;
	}
	
	public String getRoadName(){
		return roadName;
	}
	
	public String getRoadType(){
		return roadType;
	}
	
	public double getLength(){
		return length;
	}
	
	public double getSpeedLimit(){
		return speedLimit;
	}
	
	public double getTime(){
		return time;
	}
	
	public void setSpeedLimit(double speed){
		this.speedLimit = speed;
	}
	
}
