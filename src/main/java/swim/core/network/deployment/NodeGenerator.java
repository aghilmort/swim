/**
 * 
 */
package swim.core.network.deployment;

import java.util.LinkedList;
import java.util.Vector;

import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.UWSensorNetwork;
import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;
import swim.core.network.misc.math.Function;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class NodeGenerator {

	UWSensorNetwork network;
	
	int count;
	private int firstNodeId;			// Added by Sherif
	Vector<PowerLevel> powerLevels;
	Function deploymentCost;
	Function transportEnergyRate;
	Integer nodeType;
	Function idleListeningPower;
	Function receivingPower;
	Function initialEnergy; 
	Function x, y, z, g;

	public NodeGenerator(
			UWSensorNetwork network,				// the network
			int count,								// number of nodes
			int firstNodeId,						// the id to start from
			Vector<PowerLevel> powerLevels,			// node's power levels
			Function deploymentCost,				// node's deployment cost
			Function transportEnergyRate,			// node's consumed energy rate during movement (verify)
			Integer nodeType,							// node type
			Function idleListeningPower,			// node's idle listening power
			Function receivingPower,				// node's receiving power
			Function initialEnergy,					// node's initial energy, Function id, 
			Function x, Function y, Function z, 	// node's coordinates
			Function g){
		this.network = network;
		this.count = count;
		this.firstNodeId = firstNodeId;
		this.powerLevels = powerLevels;						
		this.deploymentCost = deploymentCost;				
		this.transportEnergyRate = transportEnergyRate;		
		this.nodeType = nodeType;								
		this.idleListeningPower = idleListeningPower;		
		this.receivingPower = receivingPower;				
		this.initialEnergy = initialEnergy;					 
		this.x = x;
		this.y = y;
		this.z = z;
		this.g = g;
	}

	public Vector<Integer> generate() throws InvalidNodeId, InvalidLinkId{
		
		//TODO: reserve Count elements
		
		Vector<Integer> nodes = new Vector<Integer>(count);
		
		for(int i=firstNodeId-1; i<firstNodeId+count-1; i++)
		{
			int id = network.addNode(new LinkedList<Integer>(), new LinkedList<Integer>(), 
					x.evaluate(i), y.evaluate(i), z.evaluate(i), 
					g.evaluate(i),
					powerLevels, 
					deploymentCost.evaluate(i), 
					transportEnergyRate.evaluate(i), 
					nodeType,
					idleListeningPower.evaluate(i), 
					receivingPower.evaluate(i), 
					initialEnergy.evaluate(i));
			nodes.add(id); 
		}
		return nodes;
	}
	
}
