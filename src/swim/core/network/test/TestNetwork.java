package swim.core.network.test;

import java.util.LinkedList;
import java.util.Random;
import java.util.Vector;

import swim.core.network.Demand;
import swim.core.network.FlowModel;
import swim.core.network.PowerModel;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.UWSensorNetwork;
import swim.core.network.ODPair;
import swim.core.network.PolynomialFlowModel;
import swim.core.network.deployment.NodeGenerator;
import swim.core.network.deployment.PlanarMeshNG;
import swim.core.network.deployment.PlanarRandomNG;
import swim.core.network.deployment.UniformWaterColNG;
import swim.core.network.err.InvalidDemand;
import swim.core.network.io.NetworkReader;

/**
 * To add a new sample.:
 * 1. add the sample name under the enum NetworkId
 * 2. add the network topology and demand definition to the constructor Sample(NetworkId networkId) as follows:
 * 	2.a add a case under switch(networkId)
 *  2.b copy the definition code from another case and modify it
// *  Note: don't forget the "break" line at the end of the case you will add
 * 
 *
 */
public class TestNetwork {
	
	public enum NetworkId{
		Sample2nodes1link,
		SampleSinglePath20nodes19links,
		Sample4nodes6links,
		Sample4nodes6links1ODDemand,
		Sample6nodes9links,
		Sample4nodes4links,
		SiouxFalls,
		SiouxFalls1ODPair,
		SiouxFalls16ODPair,
		SiouxFalls16demand,
		Anaheim,
		SinglePathNet20N19L,
		Sample11nodes10links,	// Hesham's sample network
		SiouxFallsWNodeProps	// Sioux Falls  w/ node properties added 
	}
	
	UWSensorNetwork network;
	Demand demand;
	FlowModel flowModel;
	
	public UWSensorNetwork getNetwork() {
		UWSensorNetwork uWSNetwork = network;
		return uWSNetwork;
	}

	public Demand getDemand() {
		return demand;
	}
	
	public TestNetwork(NetworkId networkId){
		
    	
    	PowerLevel lowPowerLevel;
    	Vector<PowerLevel> powerLevels;
    	Random rnd;
    	PlanarRandomNG randNG;
    	UniformWaterColNG uniformWCNG;
    	Vector<Integer> fixedNodes;
    	double[] distance = new double[20];
    	
    	try{
	    	switch(networkId){
	    	
	    	case Sample2nodes1link:
	    		
	    		// ===============================================================
	    		// This is a simple network consisting of two nodes and one link. 
	    		// The nodes have 1 and 2 as their id's. The link is directed and 
	    		// originates at node 1 and is destined to node 2.
	    		// ===============================================================
	    		
				//construct network
				network = new UWSensorNetwork(2, 2, 1);
				flowModel = new PolynomialFlowModel(network, 1,4);
				
				UWSensorNetwork.ACOUSTIC_LINK_PEAK_UTILIZATION = 0.18;
				
				// Specify nods' power levels
				lowPowerLevel = new PowerLevel(1, 120, 200);
				//PowerLevel highPowerLevel = new PowerLevel(10, 500, 700);
				
				powerLevels = new Vector<PowerLevel>();
				
				powerLevels.add(lowPowerLevel);
				//powerLevels.add(highPowerLevel);
				
				rnd = new Random(13103);
				
				int[] seeds = {rnd.nextInt(), rnd.nextInt(), rnd.nextInt()};
				
				// Generate two under water nodes
				randNG = new PlanarRandomNG(network, powerLevels, 0, 0, 2, 0.01, 0.1, 1e4, 1, 0.0, 400.0, 0.0, 400.0, 2, 100.0, 1, 0, seeds);
				fixedNodes = randNG.generate();
				
				for(int node : fixedNodes){
					network.setAverageChannelAccessDelay(node, 0.5);
					network.setResidualEnergy(node, 1e4);
					//network.setDataGenerationRate(node, 1);
					//network.getPowerModel().setIdleListeningPower(node, 0.01);
					//network.getPowerModel().setTransportEnergyRate(node, 0.0);
					//network.setDeploymentCost(node, 0.0);
				}
				
				distance[0] = network.distanceTo(fixedNodes.get(0), fixedNodes.get(1));
				
		    	//add links
		    	network.addLink(1, 2, 100000, distance[0], distance[0]/1500, 1500, 1, flowModel);	//1
		    	
				network.getCostModel().calculateCosts();

		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,2), 300000);
		    	break;
		    	
	    	case SampleSinglePath20nodes19links:
	    		
	    		// ===============================================================
	    		// This is a simple single path network consisting of 20 nodes and  
	    		// 19 links. The nodes have id's from 1 to 20. The links are  
	    		// directed and originate at node 1 and end at node 20.
	    		// ===============================================================
	    		
				//construct network
				network = new UWSensorNetwork(20, 20, 19);
				flowModel = new PolynomialFlowModel(network, 1,4);
				
				UWSensorNetwork.ACOUSTIC_LINK_PEAK_UTILIZATION = 0.18;
				
				// Specify nods' power levels
				lowPowerLevel = new PowerLevel(1, 120, 200);
				//PowerLevel highPowerLevel = new PowerLevel(10, 500, 700);
				
				powerLevels = new Vector<PowerLevel>();
				
				powerLevels.add(lowPowerLevel);
				//powerLevels.add(highPowerLevel);
				
				rnd = new Random(13103);
				
				int seed = rnd.nextInt();
				
				// Generate two under water nodes
				uniformWCNG = new UniformWaterColNG(network, powerLevels, 0, 0, 2, 0.01, 0.1, 1e4, 1, 0.0, 0.0, 0.0, 3800.0, 20, 1, 0, seed);
				fixedNodes = uniformWCNG.generate();
				
				for(int node : fixedNodes){
					network.setAverageChannelAccessDelay(node, 0.5);
					network.setResidualEnergy(node, 1e4);
					//network.setDataGenerationRate(node, 1);
					//network.getPowerModel().setIdleListeningPower(node, 0.01);
					//network.getPowerModel().setTransportEnergyRate(node, 0.0);
					//network.setDeploymentCost(node, 0.0);
				}
				
				distance[0] = -1;
				for(int i = 0; i < 19; i++){
					distance[i+1] = network.distanceTo(fixedNodes.get(i), fixedNodes.get(i+1));
				}
				
				
		    	//add links
				for(int i = 1; i < 20; i++){
					network.addLink(i, i+1, 100000, distance[i], distance[i]/1500, 1500, 1, flowModel);	//1
				}
				
		    	break;
		    	
			case Sample4nodes4links:
				//construct network
				network = new UWSensorNetwork(4, 4, 4);
				flowModel = new PolynomialFlowModel(network, 1,4);
		    	
		    	//add links
		    	network.addLink(1, 2, 9.0, 1, 5.0, 0, 1, flowModel);	//1
		    	network.addLink(1, 3, 7.0, 1, 3.0, 0, 1, flowModel);	//2
		    	network.addLink(2, 4, 8.0, 1, 4.0, 0, 1, flowModel);	//3
		    	network.addLink(3, 4, 6.0, 1, 3.0, 0, 1, flowModel);	//4
		    	
		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,4), 1);
		    	break;

			case Sample4nodes6links:
				network = new UWSensorNetwork(4, 4, 7);
				flowModel = new PolynomialFlowModel(network, 1,4);
		    	
				/* from, to, capacity, distance, free flow travel time, speed,
				*  toll, link type, flow model, extra cost(omega), difficulty
				*/
		    																		//FFTT	 BPR		CCF
/*
				network.addLink(1, 2, 9.0, 1, 5.7437, 0, 0, 1, flowModel,0.1,1); //a     //5      5.66462    9.75164
				network.addLink(1, 3, 7.0, 1, 6.38477, 0, 0, 1, flowModel,2,2); //b     //3      3.56709    6.31790
				network.addLink(2, 4, 8.0, 1, 13.3739, 0, 0, 1, flowModel,3,3); //c     //4      6.48572    20.7919
				network.addLink(2, 3, 4.0, 1, 4.98185, 0, 0, 1, flowModel,4,4); //d     //1      1.20951    1.85582
				network.addLink(3, 2, 3.0, 1, 1.14064, 0, 0, 1, flowModel,5,5); //e     //1      1.00000    1.00000
				network.addLink(3, 4, 6.0, 1, 4.12674, 0, 0, 1, flowModel,6,6); //f     //3      3.77792    5.84522	
*/				

				network.addLink(1, 2, 2.0, 1, 5.316, 0, 1, flowModel); //a     //5      5.66462    9.75164
				network.addLink(1, 3, 1.0, 1, 4.335, 0, 1, flowModel); //b     //3      3.56709    6.31790
				network.addLink(2, 4, 2.0, 1, 4.334, 0, 1, flowModel); //c     //4      6.48572    20.7919
				network.addLink(2, 3, 1.0, 1, 1.453, 0, 1, flowModel); //d     //1      1.20951    1.85582
				network.addLink(3, 2, 2.0, 1, 0.981, 0, 1, flowModel); //e     //1      1.00000    1.00000
				network.addLink(3, 4, 1.0, 1, 5.314, 0, 1, flowModel); //f     //3      3.77792    5.84522	
				network.addLink(2, 4, 2.0, 1, 1.820, 0, 1, flowModel); //c     //4      6.48572    20.7919

				
		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,4), 1);
		    	demand.add(new ODPair(3,4), 1);
		    	demand.add(new ODPair(1,3), 1);
		    	demand.add(new ODPair(2,3), 1);
		    	demand.add(new ODPair(2,4), 2);
		    	demand.add(new ODPair(1,2), 1);		
	
		    	break;

			case Sample4nodes6links1ODDemand:
				network = new UWSensorNetwork(4, 4, 6);
				flowModel = new PolynomialFlowModel(network, 1,4);
				
		    	network.addLink(1, 2, 9.0, 1, 5.0, 0, 1, flowModel);	//1
		    	network.addLink(1, 3, 7.0, 1, 3.0, 0, 1, flowModel);	//2
		    	network.addLink(2, 4, 8.0, 1, 4.0, 0, 1, flowModel);	//3
		    	network.addLink(2, 3, 4.0, 1, 1.0, 0, 1, flowModel);	//4
		    	network.addLink(3, 2, 3.0, 1, 1.0, 0, 1, flowModel);	//5
		    	network.addLink(3, 4, 6.0, 1, 3.0, 0, 1, flowModel);	//6
		    	
		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,4), 1);
	
		    	break;
				
			case Sample6nodes9links:
				network = new UWSensorNetwork(6, 6, 9);
				flowModel = new PolynomialFlowModel(network, 1,4);
		    	
		    	//add links
		    	network.addLink(1, 2, 5.0, 1, 1.0, 0, 1, flowModel);	//1
		    	network.addLink(1, 3, 5.0, 1, 2.0, 0, 1, flowModel);	//2
		    	network.addLink(2, 3, 5.0, 1, 1.0, 0, 1, flowModel);	//3
		    	network.addLink(2, 4, 5.0, 1, 2.0, 0, 1, flowModel);	//4
		    	network.addLink(3, 4, 5.0, 1, 1.0, 0, 1, flowModel);	//5
		    	network.addLink(3, 5, 5.0, 1, 2.0, 0, 1, flowModel);	//6
		    	network.addLink(4, 5, 5.0, 1, 1.0, 0, 1, flowModel);	//7
		    	network.addLink(4, 6, 5.0, 1, 2.0, 0, 1, flowModel);	//8
		    	network.addLink(5, 6, 5.0, 1, 1.0, 0, 1, flowModel);	//9
		    	
		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,6), 8);
		    	/*demand.add(new ODPair(3,4), 1);
		    	demand.add(new ODPair(1,3), 1);
		    	demand.add(new ODPair(2,3), 1);
		    	demand.add(new ODPair(2,4), 2);
		    	demand.add(new ODPair(1,2), 1);*/			
		    	break;
		    	
			case SiouxFalls:
				readInputFromFile("./JTraffic/siouxFallsInput.txt");
				break;
				
			case SinglePathNet20N19L:
				readInputFromFile("singlePathNetwork_20N_19L.txt");
				network.getCostModel().calculateCosts();
				break;
				
			case Sample11nodes10links:
				readInputFromFile("Hesham_test.txt");
				network.getCostModel().calculateCosts();
				break;
				
			case SiouxFallsWNodeProps:
				readInputFromFile("siouxFallsInput_Hesham.txt");
				network.getCostModel().calculateCosts();
		    	break;
				
			case SiouxFalls1ODPair:
				readInputFromFile("./JTraffic/siouxFallsInput.txt");
				
		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,20), 1.0);
		    	break;
		    	
			case SiouxFalls16ODPair:
				readInputFromFile("./JTraffic/siouxFallsInput.txt");
				
		    	demand  = new Demand(network);
		    	demand.add(new ODPair(1,6),1.0);
		    	demand.add(new ODPair(1,7),1.0);
		    	demand.add(new ODPair(1,18),1.0);
		    	demand.add(new ODPair(1,20),1.0);
		    	demand.add(new ODPair(2,6),1.0);
		    	demand.add(new ODPair(2,7),1.0);
		    	demand.add(new ODPair(2,18),1.0);
		    	demand.add(new ODPair(2,20),1.0);
		    	demand.add(new ODPair(3,6),1.0);
		    	demand.add(new ODPair(3,7),1.0);
		    	demand.add(new ODPair(3,18),1.0);
		    	demand.add(new ODPair(3,20),1.0);
		    	demand.add(new ODPair(13,6),1.0);
		    	demand.add(new ODPair(13,7),1.0);
		    	demand.add(new ODPair(13,18),1.0);
		    	demand.add(new ODPair(13,20),1.0);
				
				break;
				
			case SiouxFalls16demand:
			readInputFromFile("./siouxFallsInput.txt");
			
	    	demand  = new Demand(network);
	    	demand.add(new ODPair(1,6),300.0);
	    	demand.add(new ODPair(1,7),500.0);
	    	demand.add(new ODPair(1,18),100.0);
	    	demand.add(new ODPair(1,20),300.0);
	    	demand.add(new ODPair(2,6),400.0);
	    	demand.add(new ODPair(2,7),200.0);
	    	demand.add(new ODPair(2,18),0.0);
	    	demand.add(new ODPair(2,20),100.0);
	    	demand.add(new ODPair(3,6),300.0);
	    	demand.add(new ODPair(3,7),100.0);
	    	demand.add(new ODPair(3,18),0.0);
	    	demand.add(new ODPair(3,20),0.0);
	    	demand.add(new ODPair(13,6),200.0);
	    	demand.add(new ODPair(13,7),400.0);
	    	demand.add(new ODPair(13,18),100.0);
	    	demand.add(new ODPair(13,20),300.0);
			
			break;
			case Anaheim:
				readInputFromFile("./JTraffic/Anaheim_net.txt");
				
			}
    	}	
    	catch (Exception e) {
			System.err.printf(e.getMessage());
			e.printStackTrace();
		} 
		
	}

	private void readInputFromFile(String fileName){
    	try {
			NetworkReader networkReader = new NetworkReader(fileName);
			
			network = networkReader.readTopology();
			demand = networkReader.readDemand();		
	    }
    	catch (Exception e) {
			System.err.printf(e.getMessage());
			e.printStackTrace();
		} 
	}
	
}
