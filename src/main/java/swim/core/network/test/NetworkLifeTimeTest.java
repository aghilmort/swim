/**
 * 
 */
package swim.core.network.test;

import java.io.IOException;
import java.util.Random;

import swim.core.network.NodeType;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.UWSensorNetwork;
import swim.core.network.deployment.DOM;
import swim.core.network.deployment.NodeGenerator;
import swim.core.network.deployment.PlanarRandomNG;
import swim.core.network.deployment.Problem;
import swim.core.network.deployment.SolutionVector;
import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;
import swim.core.network.misc.math.DiscreteFunction;
import swim.core.network.InterferenceModel;

/**
 * @author Saleh Ibrahim, Sherif Tolba
 *
 */
public class NetworkLifeTimeTest {

	//N = sqrt(number of under-water sensor nodes)
	final int N;

	//M = Max Number of Surface Gateway Nodes
	final int M;

	//Alpha is the balance factor between network life and performance requirements
	//Alpha = 0 means ignore network lifetime
	static double Alpha[] = {0.0, 1.0, 0.25,0.5,0.75};
	
	Result[] summaryResult;
	
	PowerLevel lowPowerLevel;
	//PowerLevel highPowerLevel;

	//node type for under-water sensors
	NodeType sensorType;
	
	//node type for surface gateways
	NodeType gatewayType;
	
	UWSensorNetwork network;
	
	//M = Max Number of Surface Gateway Nodes
	public NetworkLifeTimeTest(UWSensorNetwork network, int N, int M){
		this.network = network;
		this.M = M;
		this.N = N;
		summaryResult = new Result[M];
	}
	
	public NetworkLifeTimeTest(UWSensorNetwork network){
		this.network = network;
		this.M = 5;
		this.N = 5;
		summaryResult = new Result[M];
	}
	
	private void createTypes(){
		//1 watt, 120m communication range 
		//and 150m interference range
		lowPowerLevel = PowerLevel.create(10, 120, 150);	

		//PowerLevel highPowerLevel = PowerLevel.create(100, 500, 700);

		//Create a node type (for under-water sensors)
		sensorType = createSensorType();
		
		//Create a node type (for surface gateways)
		gatewayType = createGatewayType(); 
	}
	
	NodeType createSensorType(){
		NodeType sensorType = NodeType.create();
		//Set the node-type properties
		sensorType.setIsSink(false);
		sensorType.addPowerLevel(lowPowerLevel);
//		sensorType.addPowerLevel(highPowerLevel);
		sensorType.setIdleListeningPower(0.01);
		sensorType.setReceivingPower(0.1);
		sensorType.setInitialEnergy(1e4);
//		sensorType.setTransportEnergyRate(0);
//		sensorType.setDeploymentCost(0);
		return sensorType;
	}

	NodeType createGatewayType(){
		NodeType gatewayType = NodeType.create();
		//Set node-type parameters
		gatewayType.setIsSink(true);
		gatewayType.addPowerLevel(lowPowerLevel);
//		gatewayType.addPowerLevel(highPowerLevel);
		gatewayType.setIdleListeningPower(0.01);
		gatewayType.setReceivingPower(0.1);
		gatewayType.setInitialEnergy(1e5);
//		gatewayType.setDeploymentCost(1000);
//		gatewayType.setTransportEnergyRate(10);
		return gatewayType;		
	}

	Problem createProblem(int seed, int T) throws InvalidNodeId, InvalidLinkId{
		
		//Create problem instance
		Problem p = Problem.create(network);

		//Set the problem-wide parameters
		p.setAcousticBitRate(100000);						//100kbps
		p.setAcousticLinkPeakUtilization(1);				//Assume no collisions
		p.setAcousticPropagationVelocity(1500);
		p.setPacketLength(400);								//400bits = 50 bytes

		p.setInterferenceModel(InterferenceModel.create(T));

		{//Generate fixed under-water nodes
			//PlanarMeshNodeGenerator NG(SensorType, 0, 0, 600,7, 0, 600, 7, 100, 1, 0,0);

			Random rnd = new Random((int)seed);
			int[] seeds = {rnd.nextInt(), rnd.nextInt(), rnd.nextInt()};

			PlanarRandomNG NG = new PlanarRandomNG(
					network
					, sensorType.getPowerLevels()
					, sensorType.getDeploymentCost()
					, sensorType.getTransportEnergyRate()
					, sensorType.get()
					, sensorType.getIdleListeningPower()
					, sensorType.getReceivingPower()
					, sensorType.getInitialEnergy()
					, 1							//FirstId
					, 0, 100*(N-1)				//BaseX, ExtentX
					, 0, 100*(N-1)				//BaseY, ExtentY
					, N*N						//Count
					, 100						//BaseZ
					, 0.01, 0					//GBase, GRange
					, seeds);

			p.fixedNodes = NG.generate();
		}

		int nodeId = p.fixedNodes().size();


		{//Create gateway nodes
			NodeGenerator NG = new NodeGenerator(
					network
					, M
					, nodeId + 1		//TODO: or at Linear(N*N,1)? need to check
					, gatewayType.getPowerLevels()
					, DiscreteFunction.constant(gatewayType.getDeploymentCost())
					, DiscreteFunction.constant(gatewayType.getTransportEnergyRate())
					, gatewayType.get()
					, DiscreteFunction.constant(gatewayType.getIdleListeningPower())
					, DiscreteFunction.constant(gatewayType.getReceivingPower())
					, DiscreteFunction.constant(gatewayType.getInitialEnergy())
					//, Linear(N*N,1)				 //Id start at N^2
					, DiscreteFunction.constant(123) 	// x
					, DiscreteFunction.constant(234) 	// y
					, DiscreteFunction.constant(0)	 	// z
					, DiscreteFunction.constant(0));	// G

			NG.generate();
		}

		return p;
	}

	void solveProblem(Problem p, int M, int T, Result summaryResult[]) throws Exception{
		
		Boolean feasible = true;			
		for(int i = M; i <= M; i++){
			
			System.out.print("M=");
			System.out.println(i);

			//Create a node instance and add it to the Gateway list
			//DOM::Node* node = P->GatewayNodes().add(GatewayType, 123,234,345);
			//{node->set_Id(nodeId++);}
			//{node->set_DataGenerationRate(0);}
			//{node->set_AverageChannelAccessDelay(0.5);}

			Result localResult = new Result();

			for(int a = 0; a < Result.length && feasible; a++){
				
				Boolean ignore = false;

				System.out.print("Alpha=");
				System.out.print(Alpha[a]);
				System.out.print(",");

				p.getObjective().reset();

				if(a == 1){
					
					p.getObjective().addComponent(DOM.objectiveComponentId.NetworkLifeTime, 1);
					p.getObjective().addComponent(DOM.objectiveComponentId.AverageDelay, 1e-10);
				
				}else if(a == 0){
					
					p.getObjective().addComponent(DOM.objectiveComponentId.NetworkLifeTime, 1e-10);
					p.getObjective().addComponent(DOM.objectiveComponentId.AverageDelay, 1);
				
				}else if(Math.abs(localResult.o[0].doubleVal()-localResult.o[1].doubleVal()/localResult.o[0].doubleVal())>1e-3 &&
						 Math.abs(localResult.c[1].doubleVal()-localResult.c[0].doubleVal())/localResult.c[0].doubleVal()>1e-3){
					
					p.getObjective().addComponent(DOM.objectiveComponentId.NetworkLifeTime, Alpha[a]/(localResult.o[0].doubleVal()-localResult.o[1].doubleVal()));
					p.getObjective().addComponent(DOM.objectiveComponentId.AverageDelay, (1-Alpha[a])/(localResult.c[1].doubleVal()-localResult.c[0].doubleVal()));
				
				}else{
					//Don't waste your time calculating, if there is no improvement 
					//in lifetime or degradation in delay
					ignore = true;
				}

				if(ignore){
					
					localResult.o[a].plusEquals(localResult.o[a-1].doubleVal()); 
					localResult.c[a].plusEquals(localResult.c[a-1].doubleVal()); 
				}
				else
				{
					double obj = p.findBestGatewayLocations(DOM.solutionMethodId.BranchAndBound);
					feasible = obj>=0;

					if(!feasible)
						break;

					SolutionVector s = ((Problem)p).getSolutionVector();

					localResult.o[a] = s.get(i).o;
					localResult.c[a] = s.get(i).c;
				}
			}
			if(feasible)
			{
				System.out.print("(T=");
				System.out.print(T);
				System.out.print(")");

				for(int a = 0; a <Result.length; a++)
				{
						summaryResult[i-1].o[a].plusEquals((1/localResult.o[a].doubleVal()-1/localResult.o[0].doubleVal())*localResult.o[0].doubleVal());
						summaryResult[i-1].c[a].plusEquals((localResult.c[a].doubleVal() - localResult.c[0].doubleVal())/localResult.c[0].doubleVal());
				}
			}
			System.out.println("");
		}
	}

	void printResult(){
		
		System.out.println();
		System.out.println("Decay");

		//Print Heading
		{
			//td::cout.width(16);	std::cout.precision(6);
			System.out.print(Alpha[0]); 
			for(int a = 2; a < Result.length; a++) {
				
				//std::cout.width(16);	std::cout.precision(6);
				System.out.print(Alpha[a]); 
			}
			//std::cout.width(16);	std::cout.precision(6);
			System.out.print(Alpha[1]);
		}

		//Print Details
		for(int i=0; i<M; i++){
			
			//std::cout.width(16);	std::cout.precision(6);
			System.out.print(summaryResult[i].o[0].doubleVal());

			for(int a = 2; a < Result.length; a++){
				
				//std::cout.width(16);	std::cout.precision(6);
				System.out.print(summaryResult[i].o[a].doubleVal());
			}

			//std::cout.width(16);	std::cout.precision(6);
			System.out.println(summaryResult[i].o[1].doubleVal());	
		}


		System.out.println();
		System.out.println("Delay");
		
		//Print Heading
		//Print Heading
		{
			//std::cout.width(16);	std::cout.precision(6);
			System.out.print(Alpha[0]); 
			for(int a = 2; a < Result.length; a++){
				
				//std::cout.width(16);	std::cout.precision(6);
				System.out.print(Alpha[a]); 
			}
			//std::cout.width(16);	std::cout.precision(6);
			System.out.print(Alpha[1]);
		}

		for(int i = 0; i < M; i++){
			
			//std::cout.width(16);	std::cout.precision(6);
			System.out.print(summaryResult[i].c[0].doubleVal());

			for(int a = 2; a < Result.length; a++){
				//std::cout.width(16);	std::cout.precision(6);
				System.out.println(summaryResult[i].c[a].doubleVal());
			}

			//std::cout.width(16);	std::cout.precision(6);
			System.out.println(summaryResult[i].c[1].doubleVal());
		}		
	}

	public void test() throws Exception{
	
		System.out.println("Optimizing Network Lifetime and/or performance");

		createTypes();

		//For a set of random under-water deployments
		for(int seed = 13103; seed < 13113; seed++){
			
			System.out.print("Seed=");
			System.out.println(seed);

			Boolean feasible = false;

			//Find the minimum feasible schedule length
			for(int T=2; T<=10 && !feasible; T++) //Link Schedule Length
			{
				System.out.print("T=");
				System.out.println(T);

				Problem p = createProblem(seed, T);

				solveProblem(p, M, T, summaryResult);

				//delete P;
			}
		}

		printResult();
	}
	
	static class Result{
		
		swim.core.network.stats.Statistics.Variable[] o;
		swim.core.network.stats.Statistics.Variable[] c;
		static int length;
		
		Result(){
			
			length = Alpha.length;			//TODO: check usage purpose and correct if necessary 
											// (source: Length = sizeof(Alpha)/sizeof(double)) 
			
			o = new swim.core.network.stats.Statistics.Variable[length];
			c = new swim.core.network.stats.Statistics.Variable[length];
			
		}
	
	}
	
}
