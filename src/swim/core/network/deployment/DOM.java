/**
 * 
 */
package swim.core.network.deployment;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class DOM {

	/** Possible values for the objective function component id
	 */
	public enum objectiveComponentId{
		/** Minimize average end-to-end delay
		 */
		AverageDelay,

		/** Minimize average energy consumption per generated packet
		 */
		AverageEnergy,

		/** Maximize network life time
		 */
		NetworkLifeTime,

		/** Minimize worst-case average latency per source
		 */
		MaximumLatency,

		/** Minimize the number of gateway nodes
		 */
		GatewayDeploymentCost,

		/** Minimize cost of moving gateway nodes to their new locations.
		 */
		GatewayRedeploymentCost
	}
	
	/** Possible choices for the algorithm of solving the DOP.
	 */
	public enum solutionMethodId{
		/** Finds the optimal solution
		 */
		BranchAndBound,
		/** Finds a good solution using a randomized method
		 */
		Randomized,
		/** Simple greedy heuristic
		 */
		Greedy,
		/** Greedy with interchange heuristic
		 */
		GreedyInterchange		
	}
	
	/** Composite Objective Function
	 *  This interface allows the user to access and modify the objective function
	 *  Use Problem.get_Objective() method to retrieve the Objective object associated with a problem
	 */
	public interface Objective {
		
		/** Add an objective function component and its corresponding weight. The objective function is specified as a weighted sum of individual objectives. By repeatedly calling add_Component for different objective components, the user can compose a multi-objective function.
		 *  If an objective component with the same id already exists in the objective function, the weight is accumulated, otherwise a new component is added.
		 */
		public void addComponent(objectiveComponentId id, double weight);
		
		/** Inspect the weight of an objective component
		 */
		public double getComponentWeight(objectiveComponentId id);
		
		/** Get rid of all objective components and start anew
		 */
		public void reset();
		
	}
	
	
}
