/**
 * 
 */
package swim.core.network.test;

import java.io.IOException;
import java.util.Random;
import java.util.Vector;

import swim.core.network.InterferenceModel;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.UWSensorNetwork;
import swim.core.network.deployment.DOM;
import swim.core.network.deployment.NodeGenerator;
import swim.core.network.deployment.PlanarRandomNG;
import swim.core.network.deployment.Problem;
import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;
import swim.core.network.misc.math.DiscreteFunction;

/**
 * @author Saleh Ibrahim, Sherif Tolba
 *
 */
public class TestDOM {

	private static UWSensorNetwork network;
	
	public static void LoadBalancing(UWSensorNetwork net) throws Exception{
		
		network = net;
	
		//Max Number of Surface Gateway Nodes
		int M = 1;
	
		System.out.println("Optimizing Deployment for Load Balancing / Lifetime");
	
		//Create problem instance
		Problem p = Problem.create(network);
	
		//Set the problem-wide parameters
		{
			p.setAcousticBitRate(100000);
			p.setAcousticLinkPeakUtilization(1);
			p.setAcousticPropagationVelocity(1500);
			p.setDeploymentVehicleDockingLocation(0,0,0);
			p.setGatewayTransportCost(0);
			p.setPacketLength(400);
			p.getObjective().addComponent(DOM.objectiveComponentId.NetworkLifeTime, 1e-10);
			p.getObjective().addComponent(DOM.objectiveComponentId.AverageDelay, 1);
	
			p.setInterferenceModel(InterferenceModel.create(0));
	
		}
	
		PowerLevel lowPowerLevel = PowerLevel.create(1, 120, 200);
		//PowerLevel highPowerLevel = PowerLevel.create(10, 500, 700);
		
		// Under-water sensor node properties:
		// ===================================
		// Node type: regular node
		// PowerLevels: LowPowerLevel, HighPowerLevel
		// IdleListeningPower: 0.01
		// ReceivingPower: 0.1
		// InitialEnergy: 1e4
		// TransportEnergyRate: 0
		// DeploymentCost: 0
	
		{// Generate fixed under-water nodes
			
			//PlanarMeshNodeGenerator NG(SensorType, 0, 0, 600,7, 0, 600, 7, 100, 1, 0,0);
			
			Random rnd = new Random(13103);
			int[] seeds = {rnd.nextInt(), rnd.nextInt(), rnd.nextInt()};
			
	    	Vector<PowerLevel> powerLevels = new Vector<PowerLevel>();
			powerLevels.add(lowPowerLevel);
			//powerLevels.add(highPowerLevel);
			
			PlanarRandomNG nG = new PlanarRandomNG(network, powerLevels, 0, 0, 2, 0.01, 0.1, 1e4, 1, 0.0, 400.0, 0.0, 400.0, 25, 100.0, 1, 0, seeds);
			p.fixedNodes = nG.generate();
			
		}
	
		//Create a node instance and add it to the fixedNodes list
		//DOM::Node* node = P->FixedNodes().add(SensorType, 100,101,102,1);
	
		//Set node-specific parameters
		//{node->set_Id(nodeId++);}
		//{node->set_Coordinates(i*100,j*100,100);}
		//{node->set_DataGenerationRate(1);}
		//{node->set_AverageChannelAccessDelay(0.5);}
	
		//P->FixedNodes()[0]->set_Coordinates(-180,0,0);
		//P->FixedNodes()[1]->set_Coordinates(80,0,0);
		//P->FixedNodes()[2]->set_Coordinates(0, 50,0);
		
	
		// Surface gateway node properties:
		// ================================
		// Node type: sink node
		// PowerLevels: LowPowerLevel, HighPowerLevel
		// IdleListeningPower: 0.01
		// ReceivingPower: 0.1
		// InitialEnergy: 1e5
		// DeploymentCost: 1000
		// TransportEnergyRate: 10
		
		{//Create gateway nodes
			
			Vector<PowerLevel> powerLevels = new Vector<PowerLevel>();
			powerLevels.add(lowPowerLevel);
			//powerLevels.add(highPowerLevel);
			
			NodeGenerator NG = new NodeGenerator(network, M, 26, powerLevels, DiscreteFunction.constant(0), DiscreteFunction.constant(0), 1, DiscreteFunction.constant(0.01), DiscreteFunction.constant(0.1), DiscreteFunction.constant(1e5), DiscreteFunction.constant(123), DiscreteFunction.constant(234), DiscreteFunction.constant(345), DiscreteFunction.constant(0));
			p.gatewayNodes = NG.generate();
		}
	
		//Create a node instance and add it to the Gateway list
		//DOM::Node* node = P->GatewayNodes().add(GatewayType, 123,234,345);
		//{node->set_Id(nodeId++);}
		//{node->set_DataGenerationRate(0);}
		//{node->set_ResidualEnergy(1e5);}
		//{node->set_AverageChannelAccessDelay(0.5);}
	
		double obj = p.findBestGatewayLocations(DOM.solutionMethodId.BranchAndBound);
	
		System.out.print("Objective value=");
		System.out.println(obj);
	
		for(int i = 0; i < p.gatewayNodes().size(); i++)
		{
			int node = p.gatewayNodes().get(i);
			double x, y, z;
			if(network.isActive(node)){
				Vector<Double> coords = network.getCoordinates(node);
				x = coords.get(0);
				y = coords.get(1);
				z = coords.get(2);
				System.out.print("Gateway active location:");
				System.out.print(x);
				System.out.print(",");
				System.out.print(y);
				System.out.print(",");
				System.out.println(z);
			}
		}
	 
		//delete P;

	}
	
	public static void main(String[] args) throws Exception{
		
//		CNetworkLifeTimeTest Test = new CNetworkLifeTimeTest(5,3);

		UWSensorNetwork network = new UWSensorNetwork();
		
		DynamicDeploymentTest Test = new DynamicDeploymentTest(network, 5, 9);

		Test.test();

//		loadBalancing();
		
	}
}
