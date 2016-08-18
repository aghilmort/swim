package swim.core.network.algorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import swim.core.network.MultipathRoute;
import swim.core.network.Network;
import swim.core.network.ODPair;
import swim.core.network.Route;
import swim.core.network.misc.FibonacciHeap;
import swim.core.network.misc.FibonacciHeapNode;
import swim.core.network.misc.ReadOnlyProperty;

/**All-to-All Dijkstra shortest path algorithm
 * Calculates shortest path lengths
 * Calculates   
 * @author Saleh
 *
 */
public class Dijkstra {
	
	/**The network on which traffic is routed
	 * 
	 */
	final Network network;
	
	
	/**
	 * The link cost property
	 */
	final ReadOnlyProperty<Double> linkCosts;
	

	/**The acceptable ratio between the cost of any active route,
	 * kept in _i[j], and the cost of the shortest route for the 
	 * same O-D pair. Must be greater than 0 to avoid missing a 
	 * shortest path due to round-off errors
	 */
	double tolerance = 1e-6;
	
	/**Initialize Dijkstra's shortest path algorithm
	 * @param network the network in which shortest paths are to be found
	 * @param linkCost the link cost property
	 */
	public Dijkstra(Network network, ReadOnlyProperty<Double> linkCost){
		this.network = network;
		this.linkCosts = linkCost;
	}
	
	public Result doIt(){

		return new Result(initDestinationSlices());
		
	}
	
	/**Find the shortest path length to the given destination node from each of the graph nodes
	 */
	private DestinationSlice[] initDestinationSlices(){
		DestinationSlice[] destinationSlices = new DestinationSlice[network.getNodeCount()+1];
		
		for(int dest: network.getNodes()){
			destinationSlices[dest] = new DestinationSlice(dest);
		}
		
		return destinationSlices;
	}
	
	public class Result{
		
		final DestinationSlice[] destinationSlices;
		
		public Result(DestinationSlice[] destinationSlices){
			this.destinationSlices = destinationSlices;			
		}
		
		public double getShortestDistance(int origin, int destination) {		
			return destinationSlices[destination].getDistanceFrom(origin);
		}
		
		public Route getShortestMultipathRoute(int origin, int destination) {
			return destinationSlices[destination].getRoute(origin);
		}	
	}

	private class DestinationSlice{
		
		private double[] distance;
		
		private int[] pathCounts;
		
		private MultipathRoute[] routes;
		
		public DestinationSlice(int destination){

			countPaths(destination);			
			
		}

		private ArrayList<Integer> sortOrigins(int destination){
			
			ArrayList<Integer> sortedOrigins = new ArrayList<Integer>(network.getNodeCount());

			FibonacciHeapNode<Integer>[] nodes = new FibonacciHeapNode[network.getNodeCount()+1];
			distance = new double[network.getNodeCount()+1];
			

			FibonacciHeap<Integer> heap = new FibonacciHeap<Integer>();
			
			for(int j: network.getNodes()){
				nodes[j] = new FibonacciHeapNode<Integer>(j, Double.POSITIVE_INFINITY);
			}

			for(int j: network.getNodes()){
				heap.insert(nodes[j], Double.POSITIVE_INFINITY);
			}

			heap.decreaseKey(nodes[destination], 0.0);
			
			while(!heap.isEmpty()){
				FibonacciHeapNode<Integer> j = heap.removeMin();
				int origin = j.getData();
				sortedOrigins.add(origin);
				
				for(int e:network.getInLinks(origin)){
					int k = network.getOrigin(e);                                        
					double alt = j.getKey()+linkCosts.get(e);
					if(alt < nodes[k].getKey()){
						heap.decreaseKey(nodes[k], alt);
					}
				}
			}
			
			for(int i: network.getNodes()){
				distance[i] = nodes[i].getKey();
			} 
                        
			return sortedOrigins;
		}

		/**Get the shortest possible distance from the source node of the slice to the given node
		 * @param dest destination node id
		 * @return
		 */
		public double getDistanceFrom(int origin){
                    
                        //System.out.println("distance from " + origin + ": " + distance[origin]);
			return distance[origin];
		}

		/**Find the critical out-bound links from each source node to the destination
		 * criticalOutboundLinksFrom[s] = links outgoing from s, that belong
		 * to the shortest route to the destination
		 */
		@SuppressWarnings("unchecked")
		private List<Integer>[] findCriticalLinksTo(int destination) {
			List<Integer>[] criticalOutboundLinksFrom = new List[network.getNodeCount()+1];

			for (int origin:network.getNodes()){
				criticalOutboundLinksFrom[origin] = new LinkedList<Integer>();

				double shortestRouteCost = getDistanceFrom(origin);
				if(origin!=destination && shortestRouteCost<Double.POSITIVE_INFINITY){
					for(int link: network.getOutLinks(origin)){
						int linkDestination= network.getDestination(link);
						double linkCost = linkCosts.get(link);
						double remainingRouteCost = getDistanceFrom(linkDestination);
						if(linkCost + remainingRouteCost <= (1+tolerance)*shortestRouteCost){							
							criticalOutboundLinksFrom[origin].add(link);
						}
					}
				}
			}
			return criticalOutboundLinksFrom;
		}

		private void countPaths(int destination) {
			
			ArrayList<Integer> sortedOrigins = sortOrigins(destination);			
			
			List<Integer>[] criticalOutboundLinksFrom = findCriticalLinksTo(destination);
			
			routes = new MultipathRoute[network.getNodeCount()+1];			
			pathCounts = new int[network.getNodeCount()+1];
			
			for(int origin:sortedOrigins){
				
				if(getDistanceFrom(origin)==Double.POSITIVE_INFINITY){
					pathCounts[origin]=0;		//zero alternatives
					routes[origin]=null;	//no route
				}else if(origin==destination){
					pathCounts[origin] = 1;		//one alternative that has an empty route
					routes[origin] = new MultipathRoute(network, new ODPair(origin, destination));
				}
				else{
					/**count routes from that source by, tracing every outgoing link to its 
					 * destination and then adding the number of routes from the destination 
					 * of each outgoing link destination to the total number of alternatives
					 */
					
					pathCounts[origin] = 0; 
					for(int e:criticalOutboundLinksFrom[origin]){
						int linkDestination = network.getDestination(e);
						pathCounts[origin] += pathCounts[linkDestination] ;
					}					
					
					routes[origin] = new MultipathRoute(network, new ODPair(origin, destination));

					for(int e:criticalOutboundLinksFrom[origin]){
						int linkDestination = network.getDestination(e);
						
						double linkProbability = 
							((double)(pathCounts[linkDestination]))/pathCounts[origin];
						
						routes[origin].addLink(e, linkProbability);
						
						routes[origin].addComponent(linkProbability, routes[linkDestination]);
					}
			
				}
			}
		}

		public Route getRoute(int origin) {
			return routes[origin];
		}

	}
	
}