/**
 * 
 */
package swim.core.network.simulation;

import swim.core.network.Assignment;
import swim.core.network.Demand;
import swim.core.network.UWSensorNetwork;
import swim.core.network.algorithm.FrankWolfeRouter;
import swim.core.network.algorithm.IncrementalRouter;
import swim.core.network.algorithm.IterativeRouter;
import swim.core.network.algorithm.Router;
import swim.core.network.algorithm.ShortestPathRouter;
import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.test.TestNetwork;
import swim.core.network.test.TestNetwork.NetworkId;

/**
 * @author Sherif Tolba
 *
 */
public abstract class Simulation {
	
	protected UWSensorNetwork uWSNetwork;	// Instance of the UWSensorNetwork network class
	//private Demand rawDemand;				// Data that need to be sent
	protected Demand demand;				// Processed data (if any processing nodes exist)
	protected long runtime;					// The total runtime in milliseconds
	protected Router router;				// A router to route traffic
	protected DoublePropertyMap flow;	
	protected String simulationId;			// Simulation Id
	protected String networkId;				// Id of the network used in the simulation
	protected TestNetwork testNet;			// The network on which the simulation will run
	protected Assignment assignment;		// Router's assignment of the given demand
	
	public Simulation(SimulationId simId, NetworkId netId, Class<?> router){

		this.simulationId = simId.toString();
		this.networkId = netId.toString();
		
		this.testNet = new TestNetwork(netId);
		this.uWSNetwork = testNet.getNetwork();
		this.demand = testNet.getDemand();
		
//		rawDemand = testNet.getDemand();
//		demand = uWSNetwork.getProcessingModel().processData(rawDemand);
		
		if(router.getName().equals(ShortestPathRouter.class.getName())){
				this.router = new ShortestPathRouter(uWSNetwork, uWSNetwork.getLinkCost(), null);
		} else if(router.getName().equals(IncrementalRouter.class.getName())){
				//TODO: initialize incremental router
		} else if(router.getName().equals(IterativeRouter.class.getName())){
				//TODO: initialize iterative router
		} else if(router.getName().equals(FrankWolfeRouter.class.getName())){
				//TODO: initialize Frank Wolfe router
		}
	
	}
	
	public void printDefaultLinkProperties(){
		// Print out default network edge properties.
		System.out.print(uWSNetwork.toString("links", new String[]{"Cpcty", "Flow", "Length", "FFTTime", "TTime", "LinkCost"}));
		System.out.print("\n");
	}
	
	public void printDefaultNodeProperties(){
		// Print out default network node properties.
		System.out.print(uWSNetwork.toString("nodes", new String[]{"Coordinates", "NodeType", "IdleListeningPower", "ReceivingPower"}));
		System.out.print("\n");
	}
	
	public void printLinkProperties(String[] propertiesToPrint){
		// Print out specified network edges properties.
		System.out.print(uWSNetwork.toString("links", propertiesToPrint));
		System.out.print("\n");
	}
	
	public void printNodeProperties(String[] propertiesToPrint){
		// Print out specified network nodes properties.
		System.out.print(uWSNetwork.toString("nodes", propertiesToPrint));
		System.out.print("\n");
	}
	
	public void printDemand(){
		System.out.print(demand);
		System.out.print("\n");
	}
	
	public abstract void run();
	
	public void startSimulationTimer(){
		// Set start time
		runtime = System.currentTimeMillis();
	}
	
	public void stopSimulationTimer(){
		runtime = System.currentTimeMillis() - runtime;
	}
	
	public void printRuntime(){
		System.out.printf("Runtime =%d ms\n", runtime);
	}
	
	
}
