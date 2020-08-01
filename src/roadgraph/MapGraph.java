package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;


public class MapGraph {
	//TODO: Add your member variables here in WEEK 2

	private Map<GeographicPoint, MapNode> grid;
	private int numVerticies;
	private int numEdges;
	private ArrayList<MapEdge> edges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		
		grid = new HashMap<GeographicPoint, MapNode>();
		numVerticies=0;
		numEdges=0;
		edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVerticies;
	}
	
	public ArrayList<MapEdge> getEdges(){
		return this.edges;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return grid.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(location ==null || grid.containsKey(location)){
			return false;
		}
		
		MapNode elem = new MapNode(location);
		
		grid.put(location, elem);
		numVerticies++;
		return true;
	}
	
	
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		//TODO: Implement this method in WEEK 2
		if(!grid.containsKey(from) || !grid.containsKey(to)){
			throw new IllegalArgumentException("The requested intersections have not been added to the graph.");
		}
		
		if(roadType == null){
			throw new IllegalArgumentException("Parameters cannot be null.");
		}
		
		if(length<0){
			throw new IllegalArgumentException("Length cannot be less than 0.");
		}
		
		MapNode fromElem = grid.get(from);
		fromElem.addNeighbor(grid.get(from), grid.get(to), roadName, roadType,length);
		
		numEdges++;
		edges.add(new MapEdge(roadName,roadType,length, grid.get(from), grid.get(to)));
	}
	

	
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		if(start == null || goal==null){
			return null;
		}
		
		MapNode startItem = grid.get(start);
		MapNode endItem = grid.get(goal);
		Map<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		boolean found = bfsSuccessful(startItem,endItem,parentMap, nodeSearched);
		if(!found){
			return null;
		}
		return constructPath(goal, parentMap);
	}
	
	
	private List<GeographicPoint> constructPath (GeographicPoint goal, Map<MapNode,MapNode> parentMap){
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.addFirst(goal);
		MapNode item = grid.get(goal);
		
		while(parentMap.containsKey(item)){
			MapNode elem = parentMap.get(item);
			path.addFirst(elem.getLocation());
			item = elem;
		}
		return path;
	}
	
	
	private boolean bfsSuccessful(MapNode start, MapNode end,
			Map<MapNode,MapNode> parentMap, Consumer<GeographicPoint> nodeSearched){
		Queue<MapNode> q = new LinkedList<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		q.add(start);
		visited.add(start);
		
		while(!q.isEmpty()){
			MapNode elem = q.poll();
			
			// Hook for visualization. 
			nodeSearched.accept(elem.getLocation());
			
			if(elem.equals(end)){
				return true;
			}
			for(MapEdge child: elem.getMapEdges()){
				MapNode i = child. getEndPoint();
				if(!visited.contains(i)){
					q.add(i);
					visited.add(i);
					parentMap.put(i, elem);
				}
			}
		}
		return false;
	}
	

	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	
	
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
{
		
		MapTempNode startNode = new MapTempNode(grid.get(start), grid.get(start));
		PriorityQueue<MapTempNode> pq = new PriorityQueue<MapTempNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode, MapNode>();
		int numRemovedNodes=0;

		startNode.setDistance(0);
		pq.add(startNode);
		
		
		while(!pq.isEmpty()){
			MapTempNode curr = pq.poll();
			numRemovedNodes++;
			if(!visited.contains(curr.getEndNode())){
				
				visited.add(curr.getEndNode());
				
				// Hook for visualization.  See writeup.
				nodeSearched.accept(curr.getEndNode().getLocation());
				
				if(!curr.getEndNode().equals(curr.getStartNode())){
					parentMap.put(curr.getEndNode(), curr.getStartNode());	
				}
				
				if (curr.getEndNode().getLocation().equals(goal)){
					System.out.println("Number of removed nodes inside: dijkstra: "+numRemovedNodes);
					return constructPath(goal,parentMap);
				}
				
				List<MapEdge> children = curr.getEndNode().getMapEdges();
				for(MapEdge child:children){
					if(!visited.contains(child.getEndPoint())){
						MapTempNode tnode = new MapTempNode(child.getStartPoint(), child.getEndPoint());
						tnode.setDistance(curr.getDistance()+child.getDistance());
						pq.add(tnode);
					}
				}
				
			}
		}
		System.out.println("Number of removed nodes inside: dijkstra: "+numRemovedNodes);
		return new LinkedList<GeographicPoint>();
}

	
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		MapTempNodeVector startNode = new MapTempNodeVector(grid.get(start), grid.get(start));
		PriorityQueue<MapTempNodeVector> pq = new PriorityQueue<MapTempNodeVector>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode, MapNode>();
		int numRemovedNodes=0;
		startNode.initializeActualDistance();
		startNode.initializePredictedDistance();
		
		pq.add(startNode);
		
		
		while(!pq.isEmpty()){
			MapTempNodeVector curr = pq.poll();
			numRemovedNodes++;
			if(!visited.contains(curr.getEndNode())){
				
				// Hook for visualization.  See writeup.
				nodeSearched.accept(curr.getEndNode().getLocation());
				
				visited.add(curr.getEndNode());
				
				if(!curr.getEndNode().equals(curr.getStartNode())){
					parentMap.put(curr.getEndNode(), curr.getStartNode());
				}
				
				if(curr.getEndNode().getLocation().equals(goal)){
					System.out.println("Number of removed nodes inside: aStarSEarch: "+numRemovedNodes);
					return constructPath(goal,parentMap);
				}
				
				for(MapEdge child: curr.getEndNode().getMapEdges()){
					if(!visited.contains(child)){
						MapTempNodeVector nv = new MapTempNodeVector (child.getStartPoint(), child.getEndPoint());
						nv.setActualDistance(child.getDistance()+curr.getActualDist());
						nv.setEstimatedDistance(child.getEndPoint(), grid.get(goal));
						
						pq.add(nv);
						
					}
				}
				
			}
	}
		System.out.println("Number of removed nodes inside: aStarSEarch: "+numRemovedNodes);
		return new LinkedList<GeographicPoint>();
	}
	
	private MapEdge pickRandomFromStart(MapNode start){
		return start.getMapEdges().get((int) (Math.random()*start.getMapEdges().size()));
	}
	private void removeNeighboringEdges(MapEdge startEdge, HashSet<MapEdge> edgesSet){
		Iterator<MapEdge> iter = edgesSet.iterator();
		ArrayList<MapEdge> edgesToRemove = new ArrayList<MapEdge>();
		
		while(iter.hasNext()){
			MapEdge edge  = iter.next();
			if(edge.getEndPoint().equals(startEdge.getEndPoint())||
					edge.getEndPoint().equals(startEdge.getStartPoint())||
					edge.getStartPoint().equals(startEdge.getEndPoint()) ||
					edge.getStartPoint().equals(startEdge.getStartPoint())){
					
				edgesToRemove.add(edge);
			}
		}
		
		for(MapEdge e:edgesToRemove){
			edgesSet.remove(e);
		}
	}
	
	public List<GeographicPoint> coverMinim(GeographicPoint start){
		
		HashSet<MapEdge> edgesSet = new HashSet<MapEdge>();
		List<GeographicPoint> result = new ArrayList<GeographicPoint>();
		
		//Copy of all edges
		for(MapEdge edge: this.getEdges()){
			edgesSet.add(edge);
		}
		
		MapNode startNode = grid.get(start);
		MapEdge startEdge = pickRandomFromStart(startNode);
		
		result.add(startEdge.getStartPoint().getLocation());
		result.add(startEdge.getEndPoint().getLocation());
		edgesSet.remove(startEdge);
		removeNeighboringEdges(startEdge, edgesSet);
		
		while(!edgesSet.isEmpty()){
			//pick any edge
			MapEdge edge = edgesSet.iterator().next();
			result.add(edge.getStartPoint().getLocation());
			result.add(edge.getEndPoint().getLocation());
			//remove all edges that touch this edge's startpoint or endpoint from edgesSet 
			removeNeighboringEdges(edge, edgesSet);
		}
		return result;
	}
		
		public void greedy(GeographicPoint rootPoint){
			double cost = 0;
			MapNode root = grid.get(rootPoint);
		
			PriorityQueue<MapTempNode> pq = new PriorityQueue<MapTempNode>();
			HashSet<MapNode> visited = new HashSet<MapNode>();
			ArrayList<MapTempNode> vis = new ArrayList<MapTempNode>();
			HashMap<MapTempNode, MapTempNode> hm = new HashMap<MapTempNode, MapTempNode >();
			
			MapTempNode node = new MapTempNode(root, root);
			node.setDistance(0);
			pq.add(node);
			
			for(MapNode n: grid.values()){
				MapTempNode tNode = new MapTempNode(n,n);
				pq.add(tNode);
			}
			
			while(!pq.isEmpty()){
				MapTempNode nodeN = pq.poll();
				if(!visited.contains(nodeN.getEndNode())){
					visited.add(nodeN.getStartNode());
					vis.add(nodeN);
					
					double minDist = Double.POSITIVE_INFINITY;
					MapNode minNode = null;
					
					for(MapEdge edge: nodeN.getStartNode().getMapEdges()){
						if(!edge.getEndPoint().equals(nodeN.getParent())){
							if(edge.getDistance()<minDist){
								minDist = edge.getDistance();
								minNode = edge.getEndPoint();
							}
						}
					}
					
					if(minNode !=null){
						MapTempNode newN = new MapTempNode(minNode,minNode); 
						newN.setParent(nodeN.getStartNode());
						newN.setDistance(minDist);
						pq.add(newN);
						hm.put(nodeN, newN);
						cost+=minDist;
					}
				}
			}
			
			hm.put(vis.get(vis.size()-1),node);
			
			MapTempNode key = node;
			while(hm.containsKey(key)){
				key = hm.get(key);
			}
			System.out.println("Cost:"+cost);
		}
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");
		*/
		
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//		
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		/*MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);*/
	}
	
}
