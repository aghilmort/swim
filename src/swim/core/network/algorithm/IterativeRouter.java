package swim.core.network.algorithm;

import java.io.PrintStream;
import java.util.Map;

import swim.core.network.Assignment;
import swim.core.network.Demand;
import swim.core.network.UWSensorNetwork;
import swim.core.network.ODPairAssignment;
import swim.core.network.Route;
import swim.core.network.algorithm.IncrementalRouter;
import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.ODPair;
import swim.core.network.Path;

/**Implements the {@link Router} interface. Starts with an initial
 * assignment using the initial shortest paths (ignoring capacities
 * and ignoring effect of flow on link cost). Then iteratively reroute
 * flows away from congested links and continues as long as the process
 * is converging, until a given termination condition is satisfied.
 * @author Saleh Ibrahim
 *
 */
public class IterativeRouter extends IncrementalRouter{

	double resolution;
	
	static final double defaultResolution = 0.015625;
	
	public IterativeRouter(
			UWSensorNetwork network,
			double resolution,
			PrintStream out) {
		
		super( network, resolution, out);
	}

	/* (non-Javadoc)
	 * @see tn.Network.Graph.ShortestPathRouter#route(tn.Network.Demand)
	 */
	@Override
	public Assignment route(Demand demand) {
		
		assert(network == demand.getNetwork());
		
		DoublePropertyMap flowBackup = network.cloneFlow();
		
		Assignment assignment = super.route(demand);
		
		network.addFlow(assignment.getFlow());
					
		int iterationCount = 0;
		boolean moreRounds = true;

		while(moreRounds && iterationCount<100){
			moreRounds=false;
			
			Assignment reassignment = super.route(demand);
			
			for(ODPairAssignment e: assignment){
				Route route = e.route;
				double assignmentVolume = e.volume;
				
				if(assignmentVolume>0.0){					
					ODPair od = route.getODPair();
					double demandVolume = demand.get(od);
					
					Route newRoute = reassignment.getRoute(od);
					
					if(!route.equals(newRoute)){
						moreRounds = true;
						
						double reroutedVolume =  Math.min(resolution*demandVolume, assignmentVolume);
						
						assignment.add(route, -reroutedVolume);
						route.addFlowToFlowMap(-reroutedVolume, network.getFlow());
						
						assignment.add(newRoute, reroutedVolume);
						newRoute.addFlowToFlowMap(reroutedVolume, network.getFlow());
						
						if(logStream!=null){
							logStream.printf("\nrerouting %f flow from %s of cost $%f to %s of cost $%f.", 
									reroutedVolume, route.toString(), route.getCost(network.getTravelCost()), 
									newRoute, newRoute.getCost(network.getTravelCost()));
						}
						
						//TODO: Investigate the effect of breaking before all assignments are considered for rerouting
						break;
					}
				}
			}
			
			iterationCount++;
		}			
		
		network.setFlow(flowBackup);
					
		return assignment;			
	}
	
}

