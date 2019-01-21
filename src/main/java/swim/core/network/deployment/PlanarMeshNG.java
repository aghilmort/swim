/**
 * 
 */
package swim.core.network.deployment;

import java.util.Vector;

import swim.core.network.UWSensorNetwork;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.misc.math.*;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class PlanarMeshNG extends NodeGenerator {

	public PlanarMeshNG(
			UWSensorNetwork network,				// the network
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			int nodeType,							// node type
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy,					// node's initial energy, Function id, 
			int firstId,							// the id to start from
			double baseX, double xExtent, int xCount, 
			double baseY, double yExtent, int yCount, 
			double baseZ, 
			double gBase, double gRange, int gSeed){
		
		super(network, 
			xCount*yCount,													// number of nodes
			firstId,														// the id to start from
			powerLevels,													// node's power levels
			DiscreteFunction.constant(deploymentCost),						// node's deployment cost
			DiscreteFunction.constant(transportEnergyRate),					// node's consumed energy rate during movement (verify)
			nodeType,															// node type
			DiscreteFunction.constant(idleListeningPower),					// node's idle listening power
			DiscreteFunction.constant(receivingPower),						// node's receiving power
			DiscreteFunction.constant(initialEnergy),						// node's initial energy, Function id, 
			DiscreteFunction.sawtooth(baseX, xExtent/(xCount-1), xCount), 	// node's coordinates
			DiscreteFunction.staircase(baseY, yExtent/(yCount-1), xCount), 
			DiscreteFunction.constant(baseZ), 	
			DiscreteFunction.uniformRandom(gBase, gRange, gSeed));			// node's data generation rate
		
		
	}

	
}
