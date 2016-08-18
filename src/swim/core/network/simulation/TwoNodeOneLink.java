/**
 * 
 */
package swim.core.network.simulation;

import swim.core.network.Assignment;
import swim.core.network.algorithm.ShortestPathRouter;
import swim.core.network.test.TestNetwork;
import swim.core.network.test.TestNetwork.NetworkId;

/**
 * @author Sherif Tolba
 *
 */
public class TwoNodeOneLink extends Simulation{
	
	public TwoNodeOneLink(){
		super(SimulationId.simple2Nodes1LinkNetwork, NetworkId.Sample2nodes1link,ShortestPathRouter.class);
	}
	
	public void run(){
		
		startSimulationTimer();
	
		assignment = router.route(demand);
		flow = assignment.getFlow();
		uWSNetwork.setFlow(flow);
		
		stopSimulationTimer();
		
		printDefaultLinkProperties();
		printDefaultNodeProperties();
		printDemand();

		double totalDelay = 0.0;
		for(int i = 1; i <= uWSNetwork.getLinkCount(); i++){
			totalDelay += uWSNetwork.getTravelTime(i);
		}
		System.out.print("\n\n");
		System.out.print("Total Delay: ");
		System.out.print(totalDelay);
		System.out.print("\n\n");
		
		printRuntime();
		
	}
}
