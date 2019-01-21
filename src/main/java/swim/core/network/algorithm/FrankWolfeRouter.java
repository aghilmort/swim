package swim.core.network.algorithm;

import java.io.PrintStream;

import org.apache.commons.collections4.Transformer;

import swim.core.network.algorithm.*;
import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.UWSensorNetwork;
import swim.core.network.Assignment;
import swim.core.network.Demand;
//import tn.TrafficNetwork.Graph;

/** Implements the {@link Router} interface.
 * Assumes infinite link capacities, and assumes that link cost
 * is constant at its value when flow is zero. It then routes 
 * each user to the initially shortest path.
 * Starting from the initial assignment, new costs are calculated
 * and the corresponding new shortest paths are found. 
 * @author Saleh Ibrahim
 *
 */
public class FrankWolfeRouter implements Router{
	
	
	/**Output stream for printing trace information
	 * 
	 */
	protected PrintStream out;

	/**Flags to control what gets printed in the trace output
	 * 
	 */
	boolean printAssignment = false,
			printFlows = false,
			printTravelTime = true,
			printLineSearch = false;
	
	/**The network in which the given demand is being routed
	 * 
	 */
	UWSensorNetwork network;
	
	/**The acceptable relative error between iterations to signal convergence
	 * 
	 */
	double tolerance;

	/**Used when {@link tolerance} is not set (or set to zero).
	 * 
	 */
	static final double defaultTolerance = 1e-8;
	
	/**The transformer passed to the shortest path algorithm step  
	 * Returns the link travel cost
	 */
	Transformer<Integer, Number> costTransformer;
	
	/**Construct a {@link FrankWolfeRouter} object
	 * @param tolerance The acceptable relative error between iterations to signal convergence
	 * @param out Output stream for printing trace information
	 */
	public FrankWolfeRouter( double tolerance, PrintStream out) {
		this.out = out;
		if(tolerance==0.0)
			this.tolerance = defaultTolerance;
		else
			this.tolerance =tolerance; 
	}
	
	
	Assignment currentAssignment;

	double ue_objective_function;
	double systemTravelCost;
	Router shortestPathRouter, router2;

	/**Calculate the initial assignment using shortest path router
	 * @param demand
	 */
	private void init(Demand demand){
		
		network = demand.getNetwork();
		
		shortestPathRouter = new ShortestPathRouter(network,network.getTravelCost(), null); 
		
		//Calculate initial shortest path assignment
		currentAssignment = reroute(demand);

		if(printAssignment && out!=null){
			out.print("\n ------------ Frank-Wolfe Initialization ---------------\n");
			out.print(currentAssignment.toString());			
		}
			
		update(currentAssignment.getFlow());		
	}
	
	/**Route the given demand using the shortest path router, 
	 * fixing the link costs at their current values
	 * @param demand the demand to be routed
	 * @return the resulting assignment
	 */
	private Assignment reroute(Demand demand) {
		return shortestPathRouter.route(demand);
	}

	/**Set the link flow values and calculate the corresponding 
	 * objective function and system travel time
	 * @param newFlow the new flow to be applied to the links 
	 * (replacing the current flow)
	 */
	private void update(DoublePropertyMap newFlow)
	{
		network.setFlow(newFlow);
		
		if(printFlows &&  out!=null){
			out.print("Link flows:\n");
			out.print(newFlow.toString());
		}
		
		
		systemTravelCost= network.getTotalTravelCost().getSum();
		ue_objective_function = 
			//systemTravelCost
			network.getTotalTravelTimeIntegral().getSum()
		;
		
		if(printTravelTime && out!=null){
			out.printf("Objective function = %10f \t", ue_objective_function );
			out.printf("System travel time =%10f\n",systemTravelCost);
		}
	}


	/**	Calculate the objective function value corresponding to the given flow 
	 * @param newFlow 
	 * @return the value of the objective function 
	 */
	private double calculateObjectiveFunction(DoublePropertyMap newFlow)
	{
		DoublePropertyMap oldFlow = network.resetFlow(newFlow);
		
		double result= network.getTotalTravelCost().getSum();
		
		network.resetFlow(oldFlow);
		
		return result;
	}

	/**	Compute the optimal combination between the current assignment and 
	 * the new assignment, by searching for the best combination ratio using 
	 * the Golden Section method
	 * @param newAssignment the new shortest path assignment
	 * @return an assignment that is a linear combination of the current 
	 * assignment and the new assignment
	 */
	private DoublePropertyMap lineSearch(Assignment newAssignment)	
	{
		DoublePropertyMap newFlow = newAssignment.getFlow();

		DoublePropertyMap 	flow_1, flow_2;
		
	 	double variable_1, variable_2;
		double value_1, value_2;
		double ALPHA = 0.618;
		double lower_limit = 0;
		double upper_limit = 1;
		
		variable_1 = lower_limit + ( 1 - ALPHA ) * ( upper_limit - lower_limit );
		variable_2 = lower_limit + ALPHA * ( upper_limit - lower_limit );
		
		flow_1 = DoublePropertyMap.combine(variable_1, network.getFlow(), 1-variable_1, newFlow);
		flow_2 = DoublePropertyMap.combine(variable_2, network.getFlow(), 1-variable_2, newFlow);			
		
		value_1 = calculateObjectiveFunction( flow_1 );
		value_2 = calculateObjectiveFunction( flow_2 );

		do
		{
			if(printLineSearch && out!=null){
				out.printf("l:(%f,%f), u(%f,%f) - ",variable_1, value_1, variable_2, value_2);
			}
			
			if( value_1 > value_2)
			{
				lower_limit = variable_1;
				
				variable_1 = variable_2;
				flow_1 = flow_2;
				value_1 = value_2;
				
				variable_2 = lower_limit + ALPHA * ( upper_limit - lower_limit);				
				flow_2 = DoublePropertyMap.combine(variable_2, network.getFlow(), 1-variable_2, newFlow);			
				value_2 = calculateObjectiveFunction( flow_2);
			}
			else if( value_1 <= value_2)
			{
				upper_limit = variable_2;
				
				variable_2 = variable_1;
				flow_2 = flow_1;
				value_2 = value_1;
				
				variable_1 = lower_limit + ( 1 - ALPHA ) * (upper_limit - lower_limit);
				flow_1 = DoublePropertyMap.combine(variable_1, network.getFlow(), 1-variable_1, newFlow);			
				value_1 = calculateObjectiveFunction(flow_1);
				
			}

		}while ( (upper_limit - lower_limit) >= tolerance);

		double result = ((upper_limit+lower_limit)/2);

		flow_1 = DoublePropertyMap.combine(result, network.getFlow(), 1-result, newFlow);			
		currentAssignment = network.combineAssignment(result, currentAssignment, 1-result, newAssignment);
		
		return flow_1;
	}

	/* (non-Javadoc)
	 * @see tn.Network.Router#route(tn.Network.Demand)
	 */
	@Override
	public Assignment route(Demand demand) {
		network = demand.getNetwork();
		
		DoublePropertyMap flowBackup = network.cloneFlow();
		init(demand);

		double error=1;			
		int iteration =1;
		double stop_criterion= tolerance;
		
		DoublePropertyMap newFlow=null, assignmentFlow=null;

		while(error>stop_criterion && iteration<100)
		{
			error = this.systemTravelCost;
			
			Assignment newAssignment = reroute(demand);
			
			newFlow = lineSearch(newAssignment);

			update(newFlow);
			
			error = Math.abs((error-this.systemTravelCost)/error);
			
			iteration++;				
		}
				
		network.setFlow(flowBackup);
		
		newFlow = currentAssignment.getFlow();
		
		return currentAssignment;			
	}
	
}
