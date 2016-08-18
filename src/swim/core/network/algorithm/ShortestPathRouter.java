package swim.core.network.algorithm;

import java.io.PrintStream; 
import java.util.List;
import java.util.Map;

import org.apache.commons.collections15.Transformer;

import swim.core.network.UWSensorNetwork;
import swim.core.network.Assignment;
import swim.core.network.Demand;
import swim.core.network.ODPair;
import swim.core.network.Path;
import swim.core.network.Route;
import swim.core.network.algorithm.Router;
import swim.core.network.misc.ReadOnlyProperty;
import edu.uci.ics.jung.algorithms.shortestpath.DijkstraShortestPath;

/** Implements the {@link Router} interface.
 * Assumes infinite link capacities, and assumes that link cost
 * is constant at its value when flow is zero. It then routes 
 * each user to the initially shortest path.
 * After the shortest path for each pair is located, flows are
 * added to the links
 * @author Saleh Ibrahim
 *
 */
public class ShortestPathRouter implements Router{
	
	protected final UWSensorNetwork network;
	
	
	protected ReadOnlyProperty<Double> linkCost;
	
	/**
	 * Stream to log the details of the routing process to the console.
	 * If null, details are not logged
	 */
	protected PrintStream logStream;

	/** shortest-path algorithm
	 * 
	 */
	private Dijkstra algorithm;
	
	/**Construct a {@link ShortestPathRouter}.
	 * @param logStream when not null, the {@link #route(Demand)} method 
	 * reports the details of the assignment procedure.
	 */
	public ShortestPathRouter(UWSensorNetwork network, ReadOnlyProperty<Double> linkCost, PrintStream logStream) {
		this.logStream = logStream;
		this.network = network;
		if(linkCost==null)
			this.linkCost = network.getTravelCost();
		else
			this.linkCost = linkCost;
		
		algorithm = new Dijkstra(network, linkCost);
	}	

	/* (non-Javadoc)
	 * @see tn.Network.Router#route(tn.Network.Demand)
	 */
	@Override
	public Assignment route(Demand demand) {

		int demandCount = demand.count();

		Assignment assignment = new Assignment(network, demandCount);
		
		Dijkstra.Result result = algorithm.doIt();
		
		for(Map.Entry<ODPair, Double> entry: demand){

			ODPair od = entry.getKey();
			
			double volume = entry.getValue();
			
			int origin = od.getOrigin();
			int destination = od.getDestination();
			
			double shortestPathCost = 
				result.getShortestDistance(origin, destination);
			
			if(shortestPathCost==Double.POSITIVE_INFINITY){
				System.err.printf("\nProblem infeasible: No path could be found between %s.\nTerminating...\n", od.toString());
				System.exit(-1);
			}
			
			Route route = result.getShortestMultipathRoute(origin, destination);
				
			assignment.add(route, volume);

			if(logStream!=null){

				logStream.printf("\n%s=%f", od.toString(), volume);

				logStream.printf("- %f (%s: $%f) ", 
						volume, route.toString(), 
						shortestPathCost);								
				
				logStream.printf("= %f ", volume);
			}
		}			
		return assignment;			
	}

}
