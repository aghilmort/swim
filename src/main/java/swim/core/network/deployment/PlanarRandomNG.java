/**
 * 
 */
package swim.core.network.deployment;

import java.util.Vector;

import swim.core.network.UWSensorNetwork;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.misc.math.DiscreteFunction;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class PlanarRandomNG extends NodeGenerator{

	public PlanarRandomNG(
			UWSensorNetwork network,				// the network
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			int nodeType,							// node type
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy,					// node's initial energy, Function id, 
			int firstId,							// the id to start from
			double baseX, double xExtent,
			double baseY, double yExtent, 
			int count, 
			double baseZ, 
			double gBase, double gRange, int[] seed){
		
		super(network, 
			count,														// number of nodes
			firstId,													// the id to start from
			powerLevels,												// node's power levels
			DiscreteFunction.constant(deploymentCost),					// node's deployment cost
			DiscreteFunction.constant(transportEnergyRate),				// node's consumed energy rate during movement (verify)
			nodeType,														// node type
			DiscreteFunction.constant(idleListeningPower),				// node's idle listening power
			DiscreteFunction.constant(receivingPower),					// node's receiving power
			DiscreteFunction.constant(initialEnergy),					// node's initial energy, Function id, 
			DiscreteFunction.uniformRandom(baseX, xExtent, seed[0]),	// node's coordinates
			DiscreteFunction.uniformRandom(baseY, yExtent, seed[1]), 
			DiscreteFunction.constant(baseZ), 
			DiscreteFunction.uniformRandom(gBase, gRange, seed[2]));	// node's data generation rate

	}	
	
}
