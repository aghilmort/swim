package swim.core.network.algorithm;

import java.io.PrintStream;
import java.util.Map;

import swim.core.network.UWSensorNetwork;
import swim.core.network.Assignment;
import swim.core.network.Demand;
import swim.core.network.ODPair;
import swim.core.network.algorithm.Router;
import swim.core.network.algorithm.ShortestPathRouter;
import swim.core.network.misc.DoublePropertyMap;

/** Implements the {@link Router} interface.
 * Assumes infinite link capacities, and assumes that link cost
 * is constant at its value when flow is zero. It then routes 
 * each user to the initially shortest path.
 * After the shortest path for each pair is located, flows are
 * added to the links
 * @author Saleh Ibrahim
 *
 */
public class IncrementalRouter extends ShortestPathRouter{
	
	/**The incremental fraction of flow to apply in each iteration
	 * 
	 */
	protected double resolution;

	protected static final double defaultResolution = 0.015625; 

	/**Construct a {@link ShortestPathRouter}.
	 * @param network
	 * @param increment the minimum fraction of flow to apply to each of the alternative shortest paths
	 * @param report When true, the {@link #route(Demand)} method reports the details of the assignment.
	 */
	public IncrementalRouter(UWSensorNetwork network,
			double increment,
			PrintStream out) {
		
		super( network, network.getTravelCost(), out);
		
		if(increment>0)
			this.resolution = increment;
		else
			this.resolution =  defaultResolution;				
	}
		
	/* (non-Javadoc)
	 * @see tn.Network.Router#route(tn.Network.Demand)
	 */
	@Override
	public Assignment route(Demand demand) {
		assert(network == demand.getNetwork());
		
		DoublePropertyMap flowBackup = network.resetFlow();

		int demandCount = demand.count();

		Assignment assignment = new Assignment(network, demandCount);
		
		Demand incrementalDemand = new Demand(network);
		Demand residualDemand = new Demand(network);
		
		int numberOfIncrements = (int) (1.0/resolution)-1;			
		
		for(Map.Entry<ODPair, Double> entry: demand){				

			ODPair od = entry.getKey();
			double volume = entry.getValue();	
			double volumeIncrement = resolution*volume;
			double residualVolume = volume -  numberOfIncrements*volumeIncrement; 
			
			incrementalDemand.add(od, volumeIncrement);
			residualDemand.add(od, residualVolume);
		}
					
		for(int i=0; i<numberOfIncrements; i++){
			Assignment assignmentIncrement = super.route(incrementalDemand);
			assignment.add(assignmentIncrement);
			network.setFlow(assignment.getFlow());
		}
		
		Assignment residualAssignment = super.route(residualDemand);
		assignment.add(residualAssignment);
		
		network.resetFlow(flowBackup);
		
		return assignment;	
	}

}
