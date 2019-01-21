/**
 * 
 */
package swim.core.network.simulation;

import java.util.Arrays;

import swim.core.network.Demand;
import swim.core.network.ODPair;
import swim.core.network.algorithm.ShortestPathRouter;
import swim.core.network.misc.math.Combinations;
import swim.core.network.test.TestNetwork.NetworkId;

/**
 * @author Sherif Tolba
 *
 */
public class SPDelayEnergyEnum extends Simulation{

	public SPDelayEnergyEnum(){
		super(SimulationId.SinglePathDelayEnergyEnumeration, 
				NetworkId.SampleSinglePath20nodes19links,
				ShortestPathRouter.class);
	}
	
	public void run(){
		
		// Keep track of simulation time
		startSimulationTimer();
		
		// Find all possible 5 node combinations
		Combinations c = new Combinations(20,5);
		
		double totalDelay;
		double totalLinkCost;
		int counter = 0;
		
		
		 while (c.hasNext()) {
			 
			 counter++;

			// Set the nodes in the combination as processing nodes
			int[] nodes = c.next();
			
			// Adjust node id's to start at 1 (by default, they numbered 0 to N-1)
			for(int e = 0; e < nodes.length; e++){
				nodes[e] += 1;
			}
			
			uWSNetwork.setNodeType(nodes[0], 3);
			uWSNetwork.setNodeType(nodes[1], 3);
			uWSNetwork.setNodeType(nodes[2], 3);
			uWSNetwork.setNodeType(nodes[3], 3);
			uWSNetwork.setNodeType(nodes[4], 3);
			
			uWSNetwork.setNodeCompressionRatio(nodes[4], 0.5);
			uWSNetwork.setNodeCompressionRatio(nodes[3], 0.55);
			uWSNetwork.setNodeCompressionRatio(nodes[2], 0.6);
			uWSNetwork.setNodeCompressionRatio(nodes[1], 0.65);
			uWSNetwork.setNodeCompressionRatio(nodes[0], 0.7);
			
			double previousDemand = 0.0;
			double volume;
			demand  = new Demand(uWSNetwork);
			
			for(int i = 1; i < 20; i++){
				volume = 3000.0;
				ODPair od = new ODPair(i, i+1);
				volume = uWSNetwork.getProcessingModel().processData(i, volume, previousDemand);
				demand.add(od, volume);	
				previousDemand = volume;
			}
			
			// Costs will be zero here as there is still no flow on the links, 
			// however, this has no effect in this specific case as there is 
			// only a single path and there are no alternatives.
			//TODO: take care of this for the more general cases
			uWSNetwork.getCostModel().calculateCosts();
			
			assignment = router.route(demand);
			flow = assignment.getFlow();
			uWSNetwork.setFlow(flow);
			
			// Here the costs can be correctly computed based on the assigned flow on each link
			uWSNetwork.getCostModel().calculateCosts();
			
//			printDefaultLinkProperties();
//			
//			System.out.print("Processing nodes: ");
			System.out.println(Arrays.toString(nodes));
//			
//			printDefaultNodeProperties();
//			printDemand();

			totalDelay = 0.0;
			totalLinkCost = 0.0;
			for(int i = 1; i <= uWSNetwork.getLinkCount(); i++){
				totalDelay += uWSNetwork.getTravelTime(i);
				totalLinkCost += uWSNetwork.getLinkCost(i);
			}
			
//			printDefaultLinkProperties();
//			System.out.print("Processing nodes: ");
//			System.out.println(Arrays.toString(nodes));
//			
//			System.out.print("\n\n");
//			System.out.print("Total Delay: ");
//			System.out.print(totalDelay);
//			System.out.print("\n\n");
//			System.out.print("Total Link Cost: ");
//			System.out.print(totalLinkCost);
//			System.out.print("\n\n");
				
//			System.out.print(totalDelay);
//			System.out.print("\n");
//			System.out.print(totalLinkCost);
//			System.out.print("\n");		
			
			// Reset the nodes' type to regular nodes
			uWSNetwork.setNodeType(nodes[0], 2);
			uWSNetwork.setNodeType(nodes[1], 2);
			uWSNetwork.setNodeType(nodes[2], 2);
			
			uWSNetwork.setNodeCompressionRatio(nodes[0], 1.0);
			uWSNetwork.setNodeCompressionRatio(nodes[1], 1.0);
			uWSNetwork.setNodeCompressionRatio(nodes[2], 1.0);
			uWSNetwork.setNodeCompressionRatio(nodes[3], 1.0);
			uWSNetwork.setNodeCompressionRatio(nodes[4], 1.0);
		   
		 }
		 
		 // Stop the simulation timer
		 stopSimulationTimer();
		 // Print its value
		 printRuntime();
		
	}
	
}
