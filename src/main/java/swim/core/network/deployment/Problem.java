/**
 * 
 */
package swim.core.network.deployment;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import swim.core.network.InterferenceModel;
import swim.core.network.Network;
import swim.core.network.UWSensorNetwork;
import swim.core.network.deployment.DOM.objectiveComponentId;
import swim.core.network.deployment.DOM.solutionMethodId;
import swim.core.network.deployment.GeometricCandidates;
import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;

import lpsolve.*;

/**
 * @authors Saleh Ibrahim, Sherif Tolba
 *
 */
public class Problem {
	
	Boolean AggregateFlow = true;

	//Derived class should override and Generate the Fixed and Candidate Nodes
	public Vector<Integer> fixedNodes;
	public Vector<Integer> gatewayNodes;
	protected Vector<Integer> candidateNodes;
	
	//All Nodes, Fixed and Candidate
	protected Vector<Integer> nodes;
	protected Vector<Integer> links;
	
	protected int colorCount;
	
	//Number of gateways to be deployed (or redeployed) at a certain stage
	private int deploymentCount;
	
	private FileWriter out;
	private int[] partialSolution;
	private double gTotal;
	
	public Objective objective;
	
	private UWSensorNetwork network;
	
	protected SolutionVector solutionVector;
	
	private static String z;
	private static String x;
	
	public double acousticLinkPeakUtilization, acousticPropagationVelocity, acousticLinkBitRate;
	public double packetLength, gatewayTransportCost;
	public double dvLocationX, dvLocationY, dvLocationZ;
	
	public InterferenceModel interferenceModel;
	
	private int edgeId;
	
	private String newLine;
	
	public Problem(UWSensorNetwork network){
		
		this.network = network; 
		this.acousticLinkPeakUtilization = 1.0;
		this.acousticPropagationVelocity = 1500.0;
		this.acousticLinkBitRate = 10000.0;
		this.packetLength = 400.0;
		this.gatewayTransportCost = 0.0;
		this.dvLocationX = this.dvLocationY = this.dvLocationZ = 0.0;
		this.newLine = System.lineSeparator();
		objective = new Objective();
		candidateNodes = new Vector<Integer>();
		nodes = new Vector<Integer>();
		links = new Vector<Integer>();
		z = "z";
		x = "x";
		
	}
	
	//Create empty problem instance
	public static Problem create(UWSensorNetwork network){
		return new Problem(network);
	}
	
	/** Get the fixed nodes' id's
	 *  Returns an array of id's representing the set of fixed (immovable) nodes.
	 */
	public Vector<Integer> fixedNodes(){
		return this.fixedNodes;
	}
	
	/** Get the set of gateway sensor nodes' id's
	 *	Returns an array of id's representing the set of movable gateway nodes.
	 */
	public Vector<Integer> gatewayNodes(){
		return this.gatewayNodes;
	}
	
	/** Get the objective function
	 *  Returns the Objective object.
	 */
	public Objective getObjective(){
		return this.objective;
	}

	/**
	 * Get the interference model
	 */
	public InterferenceModel getInterferenceModel(){
		return this.interferenceModel;
	}

	public void setInterferenceModel(InterferenceModel model){
		this.interferenceModel = model;
	}
	
	/** Set the length of data packets in bits
	 */
	public void setPacketLength(int bits){
		this.packetLength = bits;
	}
	public void setPacketLength(){
		this.packetLength = 400.0;
	}
	
	/** Set the propagation velocity of acoustic waves in water
	 */
	public void setAcousticPropagationVelocity(double metersPerSecond){
		this.acousticPropagationVelocity = metersPerSecond;
	}
	public void setAcousticPropagationVelocity(){
		this.acousticPropagationVelocity = 1500.0;
	}

	/** Set the maximum transmission bit rate
	 *  The link bit rate is the actual bit rate determined by the physical layer, i.e. the acoustic transducer and the channel coding scheme.
	 */
	public void setAcousticBitRate(double bitsPerSecond){
		this.acousticLinkBitRate = bitsPerSecond;
	}
	public void setAcousticBitRate(){
		this.acousticLinkBitRate = 10000.0;
	}

	/** Set the link maximum achievable utilization
	 *  The link capacity is the effective bandwidth determined by the link bit rate, 
	 *  Medium Access Control protocol and other nodes data generation patterns. For example, 
	 *  when using ALOHA, and the packet inter-arrival time is exponentially distributed, 
	 *  the maximum utilization of the channel is ~0.18.
	 */
	public void setAcousticLinkPeakUtilization(double fraction){
		this.acousticLinkPeakUtilization = fraction;
	}
	public void setAcousticLinkPeakUtilization(){
		this.acousticLinkPeakUtilization = 1.0;
	}

	/** Set the cost of manually transporting gateways to their new locations
	 * When gateway nodes are moved using a vehicle, this parameter denotes the 
	 * average vehicle operation cost per meter of its trip.
	 */
	public void setGatewayTransportCost(double dollarsPerMeter){
		this.gatewayTransportCost = dollarsPerMeter;
	}
	
	/** Set the initial/docking location of the deployment vehicle, 
	 * in order to calculate the trip cost.
	 */
	public void setDeploymentVehicleDockingLocation(double x, double y, double z){
		this.dvLocationX = x;
		this.dvLocationY = y;
		this.dvLocationZ = z;
	}
	public void setDeploymentVehicleDockingLocation(){
		this.dvLocationX = 0.0;
		this.dvLocationY = 0.0;
		this.dvLocationZ = 0.0;
	}

	/** Solve the ILP and find the gateway deployment locations
	 * This method instructs the DOM to solve the deployment optimization problem, 
	 * and find the suggested new gateway locations. It returns the value of the 
	 * objective function.
	 * @throws Exception 
	 */
	public double findBestGatewayLocations(solutionMethodId method) throws Exception{
		
		generateCandidateNodes();
		
		// Added to initialize nodes' residual energy
		for(Integer j : this.candidateNodes){
			network.setResidualEnergy(j.intValue(), 1e5);
		}
		
		//Nodes = FixedNodes U CandidateNodes
		network.resetNodes();
		
		//Add all fixed nodes
		Iterator<Integer> fixedNodeIter = fixedNodes.iterator();
		while(fixedNodeIter.hasNext()){
			nodes.add(fixedNodeIter.next());
		}
		
		//Add all candidate nodes
		Iterator<Integer> candidateNodeIter = candidateNodes.iterator();
		while(candidateNodeIter.hasNext()){
			nodes.add(candidateNodeIter.next());
		}
		
		int maxGatewayCount = gatewayNodes.size();
		
		deleteSolutionVector();
		
		solutionVector = new SolutionVector(maxGatewayCount);
		
		SolutionVector s = getSolutionVector();
		
		switch(method){
			case BranchAndBound: 
				this.optimalSolution();
				break;
			case Randomized:
				this.randomSolution(100);
				break;
			case Greedy:
				this.greedySolution();
				break;
			case GreedyInterchange:
				this.greedyInterchangeSolution();
				break;
		}
		
		for(int i = 0; i < maxGatewayCount; i++){
			if(i >= s.get(maxGatewayCount).getPickedCandidateCount()){
				
				this.network.setActive(i, false); // Important: a piece of code must be inserted 
												  // at an appropriate location to guarantee that 
				                                  // different types of nodes get correct ids and 
				                                  // that no id overlap happens.
			}else{
				double x, y, z;
				Vector<Double> coords = network.getCoordinates(s.get(maxGatewayCount).getPickedCandidate(i));
				x = coords.get(0);
				y = coords.get(1);
				z = coords.get(2);
				this.network.setCoordinates(i, x, y, z);
				this.network.setActive(i, true);
			}
		}
		return s.get(maxGatewayCount).objectiveValue.mean();
		
	}
	
	protected void generateCandidateNodes() throws InvalidNodeId, InvalidLinkId, IOException{
		deleteCandidateNodes();
		
		//TODO: Calculate optimal value of the resolution parameter
		GeometricCandidates GC = new GeometricCandidates(this.network, 0.5 * network.getPowerModel().getCommunicationRange(this.fixedNodes.get(0), 0));
		this.candidateNodes = GC.generateNodes(fixedNodes, candidateNodes, (network.getNodeType(this.gatewayNodes.get(0)) == 1 ? true : false));
		
		//GeometricCandidates gc();
	}
	
	protected void deleteCandidateNodes(){
		candidateNodes = null;
		System.gc();
		candidateNodes = new Vector<Integer>();
	}
	
	public SolutionVector getSolutionVector(){
		return solutionVector;
	}
	
	//Test connectivity based on Parameters
	protected Boolean isConnected(){
		
		for(int node : this.nodes){
			network.resetMark(node);

			if(network.isGateway(node))
				network.setConnectedFlag(node);
			else
				network.resetConnectedFlag(node);
		}

		Boolean newConnected;
		do {
			newConnected = false;
			Iterator<Integer> nodeIter = this.nodes.iterator();
			while(nodeIter.hasNext()){
				int theNode = nodeIter.next();
				if(network.isConnected(theNode)  && !network.isMarked(theNode)){
					network.mark(theNode);

					Vector<Integer> inLinks = (Vector<Integer>) network.getInLinks(theNode);
					Iterator<Integer> linkIter = inLinks.iterator();
					while(linkIter.hasNext()){
						int theLink = linkIter.next();
						if(!(network.isConnected(network.sourceNode(theLink))))
						{
							newConnected = true;
							network.setConnectedFlag(network.sourceNode(theLink));
						}
					}
				}
			}
		} while(newConnected);

		
		Iterator<Integer> nodeIter2 = this.nodes.iterator();
		while(nodeIter2.hasNext()){
			int theNode2 = nodeIter2.next();
			if(!(network.isConnected(theNode2))){
				
				System.out.println("Disconnected");
				return false;
				//System.exit(-1);
			}
		}
		return true;		
	}	
	
	private void refresh() throws Exception{
		this.gTotal=0.0; 
		for(int node : this.fixedNodes)	this.gTotal += network.getG(node);

		constructLinks();
		updateAllLinksCost();
	}
	
	protected double[] solve(int maximumGatewayCount, 
			int partialSolution[],	// inout
			double c,  // out
			double o, // out
			String extraConstraints // in
			) throws Exception{
		
		double[] ret = {-1, -1, -1};
		
		if(maximumGatewayCount == 0) maximumGatewayCount = this.gatewayNodes.size();
		this.deploymentCount = maximumGatewayCount ;


		FileWriter problemFile = new FileWriter("problem.lp");

		if(extraConstraints == null){
			writeProblemLP(problemFile, partialSolution);
		}else{
			writeProblemLP(problemFile, partialSolution, extraConstraints);
		}
		
		problemFile.close();

		if(colorCount > deploymentCount){
			return ret;
		}else{
			String LPFilename = "problem.lp";
			String MPSFilename = "problem.mps";
			String LPTitle = "DOM-Problem";

			LpSolve lp = LpSolve.readLp(LPFilename, 1, LPTitle);
			
			//lp->write_mps("problem.mps");

			//CIP::Problem* lp2 = CIP::Problem::read_LP(MPSFilename, TRUE, LPTitle);

			//lp->print_lp();
			//lp->set_debug(TRUE);

			if( lp.solve() == LpSolve.INFEASIBLE)
				return ret;

			double cost;

			double[] pSol = new double[10000];
			lp.getPrimalSolution(pSol);
			cost = pSol[0];

			int nRows = lp.getNrows();
			int nVar;
			
			if(partialSolution != null){
				
				int nCandidates = candidateNodes.size();
				String sVar;
				sVar = "x";

				for(int i = 0; i< nCandidates; i++)
				{
					System.out.printf("%s%d", sVar, 1000+i);
					nVar = nRows+lp.getNameindex(sVar, false);
					partialSolution[i] = (int)(pSol[nVar]+0.4);
				}
			}

			if(c != 0){
				
				nVar = nRows + lp.getNameindex("C", false);		//TODO: check if it is capital or small O
				c = pSol[nVar];
				ret[1] = c;
			}

			if(o != 0){
				
				nVar = nRows + lp.getNameindex("O", false);		//TODO: check if it is capital or small C
				o = pSol[nVar];
				ret[2] = o;
			}

			//delete lp;
			
			ret[0] = cost;
	
			return ret;
		}
	}
	
	private void writeProblemLP( FileWriter out
			, int partialSolution[] 	// an array of length equal to the number of candidate nodes. If zero, exclude node. If one, include node. If -1 node is candidate
			) throws Exception{
		
			this.out = out;
			this.partialSolution = partialSolution;

			refresh();

			//Compose F = {f(v_k,e)}
			//======================
			//

			if(!AggregateFlow){	
				//Data sources renumbering to skip zeros
				int n_G = 0;
	
				{	//Count Data sources
	
					for(int i : fixedNodes){
						
						network.setG(i, n_G);
						if(network.getG(i)>0)
							n_G++;
					}
				}
	
				//#define f(k, e)		(G[v_k]*n_E+(e))
	
			}

			//Write Problem
			//=============
			printProblem();

			writeObjective();

			writeConstraints();

			writeVariables();
	}
	
	private void writeProblemLP( FileWriter out
			, int partialSolution[] 	// an array of length equal to the number of candidate nodes. If zero, exclude node. If one, include node. If -1 node is candidate
			, String extraConstraints) throws Exception{
		
		this.out = out;
		this.partialSolution = partialSolution;

		refresh();

		//Compose F = {f(v_k,e)}
		//======================
		//

		if(!AggregateFlow){	
			//Data sources renumbering to skip zeros
			int n_G = 0;

			{	//Count Data sources

				for(int i : fixedNodes){
					
					network.setG(i, n_G);
					if(network.getG(i)>0)
						n_G++;
				}
			}

			//#define f(k, e)		(G[v_k]*n_E+(e))

		}

		//Write Problem
		//=============
		printProblem();

		writeObjective();

		writeConstraints();
		
		out.write(extraConstraints);
		out.write(newLine);
		out.write(newLine);

		writeVariables();
		
		
	}	
	
	protected void colorNodes(){
		int c = 1;
		for(int node : nodes) network.resetColor(node);

		for(int node : fixedNodes){
			if(network.setColor(node, c))
				c++;
		}
		colorCount = c - 1;
	}
	
	private void printProblem() throws IOException{
		out.write("/* Problem\n");
		printUnderwaterNodes();
		printCandidateNodes();
		printParameters();
		printLinks();
		out.write("*/");
	}
	
	private void printUnderwaterNodes() throws IOException{
		
		out.write("\nUnderwater nodes <x> <y> <z> <g>\n");

		StringBuilder str;
		for(int node : fixedNodes){
			str = new StringBuilder();
			str.append(Double.toString(network.getX(node)));
			str.append("\t");
			str.append(Double.toString(network.getY(node)));
			str.append("\t");
			str.append(Double.toString(network.getZ(node)));
			str.append("\t");
			str.append(Double.toString(network.getG(node)));
			str.append("\n");
			out.write(str.toString());
		}
		
	}
	
	private void printCandidateNodes() throws IOException{
		out.write("\nSurface candidate positions <x> <y> (z and g are always 0 for surface nodes)\n");
		
		StringBuilder str;
		for(int node : candidateNodes){
			str = new StringBuilder();
			str.append(Double.toString(network.getX(node)));
			str.append("\t");
			str.append(Double.toString(network.getY(node)));
			str.append("\t");
			str.append(Double.toString(network.getZ(node)));
			str.append("\t");
			str.append(Double.toString(network.getG(node)));
			str.append("\n");
			out.write(str.toString());
		}
	}	
	
	private void printParameters(){
		/*    (*m_out)<<"\nCommunication Range = "<<m_Parameters.get_CommunicationRange()<<""<<std::endl;
		(*m_out)<<"\nUnderwater Link Capacity = "<<m_Parameters.get_LinkCapacity()<<""<<std::endl;
		(*m_out)<<"\nMaximum number of surface nodes = "<<m_Parameters.get_MaximumGatewayCount()<<""<<std::endl;
		*/		
	}
	
	private void printLinks() throws IOException{
		
		StringBuilder str;
		for(int link : links){
			str = new StringBuilder();
			str.append(Integer.toString(link));
			str.append(":\t");
			str.append(Integer.toString(network.sourceNode(link)));
			str.append(",");
			str.append(Integer.toString(network.targetNode(link)));
			str.append("\n");
			out.write(str.toString());
		}
			
	}
	
	private void writePerNodeFlowVariables() throws IOException{
		
		out.write("\n//(*) Fin(v) = sum_{e in Ein(v)} f(e), for all v\n");
		out.write("//    Fout(v) = sum_{e in Eout(v)} f(e), for all v\n");
		out.write("//---------------\n");
	
		for(int v : nodes){
			//Write Equations for Fout
			out.write("0");
			if(!AggregateFlow){
				for(int k : fixedNodes){
					if(network.getG(k) > 0){
						for(int i : network.getOutLinks(v)){
							out.write("+f");
							out.write(Integer.toString(i));
						}
					}
				}
			}else{
				for(int i : network.getOutLinks(v)){
					out.write("+f");
					out.write(Integer.toString(i));
				}
			}
			out.write(" = Fout");
			out.write(Integer.toString(v));
			out.write(";\n");

			//Write Equations for Fin
			out.write("0");
			if(!AggregateFlow){
				for(int k : fixedNodes){ 
					if(network.getG(k) > 0){
						for(int i : network.getInLinks(v)){
							out.write("+f");
							out.write(Integer.toString(i));
						}
					}
				}
			}else{
				for(int i : network.getInLinks(v)){
					out.write("+f");
					out.write(Integer.toString(i));
				}
			}
			out.write(" = Fin");
			out.write(Integer.toString(v));
			out.write(";\n");
		}
	}
	
	private void writeNodePowerVariables() throws IOException{
		
		out.write("\n//(*) pi(v) = pi_l + t_s * pi_r * Fin(v) + t_s * pi_s * Fout(v)\n");
		out.write("//---------------\n");

		double t_s = packetLength/ acousticLinkBitRate;

		for(int v : nodes){
			
			StringBuilder sb = new StringBuilder();
			sb.append("pi");
			sb.append(v);
			sb.append("=");
			sb.append(network.getListeningPower(v));
			sb.append("+");
			sb.append(t_s * network.getReceivingPower(v));
			sb.append(" Fin");
			sb.append(v);
			out.write(sb.toString());
				
			StringBuilder sb2;
			if(!AggregateFlow){
				for(int k : fixedNodes){ 
					if(network.getG(k) > 0){
						sb2 = new StringBuilder();
						for(int i : network.getOutLinks(v)){
							sb2.append("+");
							sb2.append(t_s * network.getPowerLevel(i).getTransmissionPower());
							sb2.append(" f");
							sb2.append(i);
							out.write(sb2.toString());
						}
					}
				}
			}else{
				sb2 = new StringBuilder();
				for(int i : network.getOutLinks(v)){
					sb2.append("+");
					sb2.append(t_s * network.getPowerLevel(i).getTransmissionPower());
					sb2.append(" f");
					sb2.append(i);
					out.write(sb2.toString());
				}
			}
			out.write(";\n");
		}
	}
	
	private void writeCandidateNodesBinaryConstraints() throws IOException{
		
		out.write("\n//(1) Candidate Nodes: xi in {0,1}, and fixed nodes xi=0 or xi=1\n");
		out.write("//---------------\n");
		{
			int offset = (this.fixedNodes.size()+this.gatewayNodes.size()+1);
			for(Integer i : candidateNodes){
				if(partialSolution != null  && (partialSolution[i.intValue() - offset] >= 0)){
					out.write(x);
					out.write(i.toString());
					out.write("=");
					out.write(partialSolution[i.intValue() - offset]);
					out.write(";\n\n");
				}
				else
				{
					out.write("0<=");
					out.write(x);
					out.write(i.toString());
					out.write("<=1");
					out.write(";\n");
				}
			}
		}
		out.write("\n//(1') Node disable flags: zi in {0,1}\n");
		out.write("//---------------\n");
		{
			for(Integer i :nodes){
				out.write("0<=");
				out.write(z);
				out.write(i.toString());
				out.write("<=1");
				out.write(";\n");
			}
		}

		out.write("\n//(1'') Schedule flags: hi in {0,1}\n");
		out.write("//---------------\n");
		{
			int T = interferenceModel.getSlotCount();

			if(T>1){
				for(Integer v : nodes){
					for(Integer i : network.getOutLinks(v.intValue())){
						for(int t = 0; t < T; t++){
							out.write("0<= h");
							out.write(i.toString());
							out.write("_");
							out.write(Integer.toString(t));
							out.write("<=1;\n");
						}
					}
				}
			}
		}
	}	
	
	private void writeObjective() throws IOException{
		
		out.write("\n//Objective Function\n");
		out.write("//======================\n");

		if(acousticLinkBitRate > 0){
			out.write("min: Obj;\n\nObj=");
			if(objective.getComponentWeight(objectiveComponentId.NetworkLifeTime) > 0){
				out.write(Double.toString(objective.getComponentWeight(objectiveComponentId.NetworkLifeTime)));
				out.write(" O ");
			}
			if(objective.getComponentWeight(objectiveComponentId.AverageDelay)>0){
				out.write("+");
				out.write(Double.toString(objective.getComponentWeight(objectiveComponentId.AverageDelay)));
				out.write(" C");
			}
			out.write(";\n");

			if(objective.getComponentWeight(objectiveComponentId.NetworkLifeTime)>0)
			{			
				out.write("\n// Dead-node Tolerance Constraints");
				out.write("\n// ===============================");
				out.write("\n// [1] w(v) - z(v) Omax = pi(v) /");
				out.write(" epsilon(v) - z(v) Omax <= O, for all v\n");

				out.write("//---------------\n");
				for(Integer v : nodes){
					out.write("pi");
					out.write(v.toString());
					out.write(" - 1e10 ");
					out.write(z);
					out.write(v.toString());
					out.write(" <=");
					out.write(Double.toString(network.getResidualEnergy(v.intValue())));
					out.write(" O;\n");
				}
				out.write("\n");

				out.write("\n// [2] Sum_{v in V'} z(v) = 0\n");
				out.write("//---------------\n");
				for(Integer v : nodes)
				{
					out.write("+ ");
					out.write(z);
					out.write(v.toString());
				}
				out.write("= 0;\n\n");

			}

			if(objective.getComponentWeight(objectiveComponentId.AverageDelay) > 0){
				out.write("\n// AverageDelay = (Sum (c_e*f_e) for all edges e) / Number Of Sources\n");
				out.write("//---------------\n");
				out.write(Double.toString(gTotal));
				out.write(" C=");
				if(!AggregateFlow){
					for(int k = 0; k < fixedNodes.size(); k++){
						if(network.getG(fixedNodes.get(k)) > 0){
							for(Integer e : links){
								out.write("+");
								out.write(Double.toString(network.getLinkCost(e.intValue())));
								out.write(" f");
								out.write(e.toString());
							}
						}
					} 
				}else{
					for(Integer e : links){
						out.write("+");
						out.write(Double.toString(network.getLinkCost(e.intValue())));
						out.write(" f");
						out.write(e.toString());
					}
				}
				out.write(";\n");
			}
		}
		else
		{
			//TODO: fix the bandwidth minimization objective
			out.write("\nmin: B;\n\n");
			out.write("B < ");
			out.write(Double.toString(acousticLinkBitRate));
			out.write(";\n\n");
		}
	}
	
	private void writeCandidateNodesFlowConstraints() throws IOException{
		writePerNodeFlowVariables();

		out.write("\n//(2) f(vk, (vi, tj)) <=  x(tj ).g(vk), for all vk in FixedNodes, tj in CandidateNodes, (vi, tj) in E.\n");
		out.write("//-----------------\n");

		
		for(Integer j : candidateNodes){
			if(!AggregateFlow){
				for(Integer k : fixedNodes){
					if(network.getG(k) > 0){
						for(Integer i : network.getInLinks(j)){
							out.write("f");
							out.write(i.toString());
							out.write("<=");
							out.write(Double.toString(network.getG(k.intValue())));
							out.write(" ");
							out.write(x);
							out.write(j.toString());
							out.write(";\n");
						}
					}
				}
			}

			if(AggregateFlow){
				out.write("Fin");
				out.write(j.toString());
				out.write("<=");
				out.write(Double.toString(gTotal));
				out.write(" ");
				out.write(x);
				out.write(j.toString());
				out.write(";\n");
			}
		}
		
	}
	
	private void writeMediumAccessConstraints() throws IOException
	{
		//Use the interference model to decide which transmissions are not possible simultaneously
		double ts = interferenceModel.getSlotTime();
		double packetTime = packetLength / acousticLinkBitRate;
		if(ts ==0) ts = packetTime;
		int scheduleLength = interferenceModel.getSlotCount();
		int T = scheduleLength;

		if(scheduleLength > 1){
			out.write("\n// f_e <= (LinkCapacity)/T * Sum_{1<=t<=T}");
			out.write("h_e^t \n");
			out.write("//---------------\n");
			{
				for(int v : nodes){
					for(Integer i : network.getOutLinks(v)){
						out.write(Double.toString(scheduleLength/acousticLinkBitRate));
						out.write(" f");
						out.write(i.toString());
						out.write("<=");
						for(int t = 0; t < T; t++){
							out.write("+h");
							out.write(i.toString());
							out.write("_");
							out.write(Integer.toString(t));
						}
						out.write(";\n");
					}
				}
			}


			out.write("\n// Prevent collision at source \n");
			out.write("//---------------\n");
			{
				for(Integer v : fixedNodes){
					for(int t = 0; t < T; t++){
						for(Integer i : network.getOutLinks(v.intValue())){
							out.write("+ h");
							out.write(i.toString());
							out.write("_");
							out.write(Integer.toString(t));
						}
						out.write("<=1;\n");
					}
				}
			}

			out.write("\n// Prevent collision at receiver\n");
			out.write("//---------------\n");
			for(Integer v : nodes){
				for(Integer w : network.getInterferingNodes(v.intValue())){
					double tp_wv = network.distanceTo(w.intValue(), v.intValue()) / this.acousticPropagationVelocity;

					for(Integer uv : network.getInLinks(v.intValue())){
						int u = network.sourceNode(uv);

						if(u != w){

							double tp_uv = network.distanceTo(u, v.intValue()) / this.acousticPropagationVelocity;
							double dw_uv = tp_wv - tp_uv;

							int lo = (int) Math.floor(dw_uv/ts);
							int hi = (int) Math.ceil(dw_uv/ts);

							for(Integer wz : network.getOutLinks(w.intValue())){
								for(int t = 0; t < T; t++){
									for(int p = lo; p <= hi; p++)
									{
										int DT = (t-p)%T;
										if(DT<0) DT+=T;
										out.write("h");
										out.write(uv.toString());
										out.write("_");
										out.write(Integer.toString(t));
										out.write("<=1-h");
										out.write(wz.toString());
										out.write("_");
										out.write(Integer.toString(DT));
										out.write(";\n");
									}
								}
							}
						}
					}
				}
			}
		}
		else
		{
			out.write("\n//(3) Sum {vk in FixedNodes,(FixedNodes,vi) in Eout(FixedNodes)} f(vk, (FixedNodes, vi)) + \n");
			out.write("//           Sum {vk in FixedNodes,(vi,FixedNodes) in Ein(FixedNodes)} f(vk, (vi, FixedNodes)) < LinkCapacity/PacketLengthInBits, forall FixedNodes in FixedNodes'\n");
			out.write("//---------------\n");
			{
				for(Integer v : nodes){
					if(!AggregateFlow){
						for(Integer k : fixedNodes) if(network.getG(k.intValue()) > 0){
							for(Integer i : network.getOutLinks(v.intValue())){
								out.write("+f");
								out.write(i.toString());
							}
							for(Integer i : network.getInLinks(v.intValue())){
								out.write("+f");
								out.write(i.toString());
							}	
						}
					}else{
						for(Integer i : network.getOutLinks(v.intValue())){
							out.write("+f");
							out.write(i.toString());
						}
						for(Integer i : network.getInLinks(v.intValue())){
							out.write("+f");
							out.write(i.toString());
						}	
					}
					
					if(network.getOutLinks(v).size()+network.getInLinks(v).size() > 0){
						out.write("<");
						if(acousticLinkBitRate > 0){
							out.write(Double.toString(acousticLinkBitRate/packetLength));
						}else{
							out.write(Double.toString(1.0/packetLength));
							out.write(" LinkCapacity");
						}
						out.write(";\n");
					}
				}
			}
		}
	}
	
	private void writePerNodeFlowConservationConstraints() throws IOException{
		out.write("\n//(4) Per-node flow conservation\n");
		out.write("//---------------\n");
		{
			if(!AggregateFlow){
				for(Integer k : fixedNodes){
					if(network.getG(k.intValue()) > 0){
						for(Integer i : fixedNodes){
							for(Integer j : network.getOutLinks(i.intValue())){
								out.write("+f");
								out.write(j.toString());
							}
							
							for(Integer j : network.getInLinks(i.intValue())){
								out.write("-f");
								out.write(j.toString());
							}

							if(network.getOutLinks(i.intValue()).size() + network.getInLinks(i.intValue()).size() > 0){
								out.write("=");
								out.write(Double.toString(((i.intValue() == k.intValue()) ? network.getG(i.intValue()) : 0)));
								out.write(";\n");
							}
						}
					}
				}	
			}else{
				for(Integer i : fixedNodes){
					for(Integer j : network.getOutLinks(i.intValue())){
						out.write("+f");
						out.write(j.toString());
					}
					
					for(Integer j : network.getInLinks(i.intValue())){
						out.write("-f");
						out.write(j.toString());
					}

					if(network.getOutLinks(i.intValue()).size() + network.getInLinks(i.intValue()).size() > 0){
						out.write("=");
						out.write(Double.toString(network.getG(i.intValue())));
						out.write(";\n");
					}
				}
			}
			
		}
	}
	
	private void writeEndToEndFlowConservationConstraints() throws IOException{
		
		out.write("\n//(5) end-to-end flow conservation");
		out.write(newLine);
		out.write("//-------------");
		out.write(newLine);
		
		{
			if(!AggregateFlow){
				for(Integer k : fixedNodes){
					if(network.getG(k.intValue()) > 0){
						for(Integer j : candidateNodes){
							for(Integer i : network.getInLinks(j.intValue())){
								out.write("+f");
								out.write(i.toString());
							}								
						}
						out.write("=");
						out.write(Double.toString(network.getG(k.intValue())));
						out.write(";");
						out.write(newLine);						
					}
				}
			}else{
				for(Integer j : candidateNodes){
					for(Integer i : network.getInLinks(j.intValue())){
						out.write("+f");
						out.write(i.toString());
					}								
				}
				out.write("=");
				out.write(Double.toString(gTotal));
				out.write(";");
				out.write(newLine);		
			}
		}
	}
	
	private void writeCandidateCountConstraint() throws IOException{
		
		out.write("\n//(6) Maximum number of surface nodes");
		out.write(newLine);
		out.write("//---------------");
		out.write(newLine);
		
		//	To simplify editing the number of surface nodes
//		out.write("\n1<= M <=");
//		out.write(deploymentCount);
//		out.write(";");
//		out.write(newLine);

		{
			for(Integer j : candidateNodes){
				out.write("+");
				out.write(x);
				out.write(j.toString());
			}

			out.write("<=");
			out.write(Integer.toString(deploymentCount));
			out.write(";\n");
			out.write(newLine);
		}

		out.write("\n//Choose at least one of each group. ");
		out.write("Number of groups = ");
		out.write(Integer.toString(colorCount));
		out.write(newLine);

		out.write("\nZERO<ONE;\nONE=1;");
		out.write(newLine);

		for(int c = 1; c <= colorCount; c++){
			out.write("ZERO");
			for(Integer j : candidateNodes){
				if(network.getColor(j.intValue()) == c){
					out.write("+");
					out.write(x);
					out.write(j.toString());
				}
			}
			out.write(">=1;");
			out.write(newLine);
		}
	}
	
	private void writeConstraints() throws IOException{
		
		out.write("\n//Constraints");
		out.write(newLine);
		out.write("//=================");
		out.write(newLine);

		writeCandidateNodesBinaryConstraints();

		writeCandidateNodesFlowConstraints();

		writeMediumAccessConstraints();

		writePerNodeFlowConservationConstraints();

		writeEndToEndFlowConservationConstraints();

		writeCandidateCountConstraint();
		
		writePerNodeFlowVariables();

		if(objective.getComponentWeight(DOM.objectiveComponentId.NetworkLifeTime) > 0)
			writeNodePowerVariables();

	}
	
	private void writeMediumAccessVariables() throws IOException{
		
		int T = interferenceModel.getSlotCount();

		if(T > 1){
			for(Integer v : nodes){
				for(Integer i : network.getOutLinks(v.intValue())){
					for(int t = 0; t < T; t++){
						out.write("int h");
						out.write(i.toString());
						out.write("_");
						out.write(Integer.toString(t));
						out.write(";");
						out.write(newLine);
					}
				}
			}
		}
	}
	
	private void writeVariables() throws IOException{
		
		out.write("\n//Define variables:");
		out.write(newLine);
		out.write("//-----------------");
		out.write(newLine);

		{
			if(acousticLinkBitRate < 0){
				out.write("sec B;");
				out.write(newLine);
				out.write(newLine);
			}
			
			for(Integer j : candidateNodes){
				out.write("int ");
				out.write(x);
				out.write(j.toString());
				out.write(";");
				out.write(newLine);
			}
			out.write(newLine);

			for(Integer j : nodes){
				out.write("int ");
				out.write(z);
				out.write(j.toString());
				out.write(";");
				out.write(newLine);
			}
			out.write(newLine);

			if(!AggregateFlow){
				for(Integer k : fixedNodes){
					if(network.getG(k.intValue()) > 0){
						for(Integer e : links){
							out.write("sec f");
							out.write(e.toString());
							out.write(";");
							out.write(newLine);
						}
					}
				}
			}else{
				for(Integer e : links){
					out.write("sec f");
					out.write(e.toString());
					out.write(";");
					out.write(newLine);
				}
			}
			out.write(newLine);

			for(Integer v : nodes){
				out.write("sec Fin");
				out.write(v.toString());
				out.write(";\nsec Fout");
				out.write(v.toString());
				out.write(";");
				out.write(newLine);
			}
			out.write(newLine);

			if(objective.getComponentWeight(DOM.objectiveComponentId.NetworkLifeTime) > 0){
				out.write("sec O;");
				out.write(newLine);

				for(Integer v : nodes){
					out.write("sec pi");
					out.write(v.toString());
					out.write(";");
					out.write(newLine);
				}	
			}

			writeMediumAccessVariables();

			out.write("int ZERO;\nint ONE;");
			out.write(newLine);

		}
	}
	
	protected void optimalSolution() throws Exception{
		
		SolutionVector sol = getSolutionVector();
		int maxGateways = sol.items().size();

		int n = candidateNodes.size();
		int solution[] = new int[n];

//      char problemID[256]; printProblemID(problemID);
//		out.write(problemID);
//		out.write("-Optimal");
//		out.write(newLine);

		for(int i = 1; i <= maxGateways; i++){
			
			for(int j = 0; j < n; j++) solution[j] = -1;

			//getParameters().setMaximumGatewayCount(i);

			long t_before = System.currentTimeMillis();
			double c = 1, o = 1;
			double[] ret = solve(i, solution, c, o, null);
			double cost = ret[0];
			c = ret[1];
			o = ret[2];
			long runtime = System.currentTimeMillis() - t_before;
		
			if(cost>0){
				Solution s = sol.get(i);
				s.objectiveValue.plusEquals(cost);
				s.runtime.plusEquals((double)runtime);
				s.setSolution(candidateNodes, solution);
				s.c.plusEquals(c);
				s.o.plusEquals(o);
				sol.set(i, s);
			}
		}
	}
	
	protected void randomSolution( int numberOfRandomDeployments) throws Exception{
		
		SolutionVector sol = getSolutionVector();
		int MaxGateways = sol.items().size();

	/*	char problemID[256]; printProblemID(problemID);
		out.write(problemID);
		out.write("-Random");
		out.write(newLine);*/
		
		
		Random random = new Random(13103);

		int n= candidateNodes.size();
		int solution[] = new int[n];

		for(int k = 0; k < numberOfRandomDeployments; k++){
			
			for(int i = 0; i < n; i++) solution[i] = -1;

			for(int i = 0; i < MaxGateways; i++){
				
				int j ; do j = random.nextInt(n-1); while(solution[j] > 0);

				solution[j] = 1;

				double c = 1, o = 1;
				double[] ret = solve(i+1, solution, c, o, null);
				double thisCost = ret[0];
				c = ret[1];
				o = ret[2];
	
				if(thisCost>=0) //feasible
				{
					Solution s = sol.get(i);
					s.objectiveValue.plusEquals(thisCost);
					s.setSolution(candidateNodes, solution);
					s.c.plusEquals(c);
					s.o.plusEquals(o);
					sol.set(i, s);
				}
			}
		}

		solution = null;
		System.gc();
	}
	
	protected void greedySolution() throws Exception{
		
		SolutionVector sol = getSolutionVector();
		int MaxGateways = sol.items().size();

		/*char problemID[256]; printProblemID(problemID);
		out.write(problemID);
		out.write("-Greedy");
		out.write(newLIne);*/
		
		int n = candidateNodes.size();

		int partialSolution[] = new int[n];
		for(int j = 0; j < n; j++) partialSolution[j] = -1;

		long t_before = System.currentTimeMillis();

		for(int i=1; i<=MaxGateways; i++)
		{
			for(int j=0; j<n; j++) {if(partialSolution[j]!=1) partialSolution[j] = -1;}

			double c = 1,o = 1;
			double[] ret = solve(i, partialSolution, c, o, null);
			double cost = ret[0];
			c = ret[1];
			o = ret[2];
			
			long runtime = System.currentTimeMillis() - t_before;

        		if(cost>0)
				{
        			Solution s = sol.get(i);
        			s.objectiveValue.plusEquals(cost);
        			s.runtime.plusEquals((double)runtime);
        			s.setSolution(candidateNodes, partialSolution);
        			s.c.plusEquals(c);
        			s.o.plusEquals(o);
        			sol.set(i, s);
        		}
		}
	}
	
	protected void greedyInterchangeSolution() throws Exception{
		
		SolutionVector sol = getSolutionVector();
		int maxGateways = sol.items().size();

		String interchangeConstraint;

	/*	char problemID[256]; printProblemID(problemID);
		out.write(problemID);
		out.write("-GreedyInterchange");
		out.write(newLine); */

		int n = candidateNodes.size();

		int partialSolution[] = new int[n];
		for(int j = 0; j < n; j++) partialSolution[j] = -1;

		//Find a feasible solution
		long t_before = System.currentTimeMillis();
		int min_feasible_i = -1000;
		for(int i = 1; i <= maxGateways; i++){
			double c = 1, o = 1;
			double[] ret = solve(i, partialSolution, c, o, null);
			double cost = ret[0];
			c = ret[1];
			o = ret[2];
			
			long runtime = System.currentTimeMillis() - t_before;
			if(cost > 0){
				Solution s = sol.get(i);
				s.objectiveValue.plusEquals(cost);
				s.runtime.plusEquals((double)runtime);
				s.setSolution(candidateNodes, partialSolution);
				s.c.plusEquals(c);
				s.o.plusEquals(o);
				sol.set(i, s);

				min_feasible_i = i;
				break;
			}
		}

		if(min_feasible_i != -1000){
			for(int i = min_feasible_i + 1; i <= maxGateways; i++)
			{
				//Greedy
				{for(int j = 0; j < n; j++) if(partialSolution[j] != 1) partialSolution[j] = -1;}
	
				solve(i, partialSolution, 0, 0, null);
	
				String buffer;
				//Interchange
				interchangeConstraint = "0";
				for(int j=0; j<n; j++)
					if(partialSolution[j]!=1)
						partialSolution[j] = -1;
					else
					{
						buffer = String.format("+ x%d", j);		// Check if this achieves purpose
						interchangeConstraint.concat(buffer);	// Check if this achieves purpose
					}
	
				buffer = String.format(">= %d;", i-1);		// Check if this achieves purpose
				interchangeConstraint.concat(buffer);	// Check if this achieves purpose
	
				for(int j = 0; j < n; j++) partialSolution[j] = -1;
	
				double c = 1, o = 1;
	
				double[] ret = solve(i, partialSolution, c, o, interchangeConstraint);
				double cost = ret[0];
				c = ret[1];
				o = ret[2];
				
				long runtime = System.currentTimeMillis() - t_before;
	
				if(cost > 0){
					Solution s = sol.get(i);
					s.objectiveValue.plusEquals(cost);
					s.runtime.plusEquals((double)runtime);
					s.setSolution(candidateNodes, partialSolution);
					sol.set(i, s);
	        		}
			}
		}else{
			System.out.println("min_feasible_i not initialized!");
			System.exit(-1);
		}

		partialSolution = null;
		System.gc();
	}
	
	protected void deleteSolutionVector(){

			solutionVector = null;
			System.gc();
			int maxGatewayCount = gatewayNodes.size();
			solutionVector = new SolutionVector(maxGatewayCount);
		
	}
	
	public void setNetwork(UWSensorNetwork net){
		this.network = net;
	}
	
	public Network getNetwork(){
		return this.network;
	}
	
	//Construct edges based on Parameters
	private void constructLinks() throws InvalidNodeId, InvalidLinkId{
		
		this.edgeId = 0;
		
		double interferenceRange = network.getPowerModel().getPowerLevel(this.fixedNodes.get(0), 0).getInterferenceRange();

		for(int node : this.nodes){
			network.getInLinks(node).clear();
			network.getOutLinks(node).clear();
			network.getInterferingNodes(node).clear();
		}

		this.links.clear();
		System.gc();

		// Added to find total number of links
		int totNumLinks = 0;
		for(@SuppressWarnings("unused") int src : this.fixedNodes){
			for(@SuppressWarnings("unused") int dst : this.nodes)
			{
				totNumLinks++;
			}
		}
		
		// Initialize network links
		network.deferredLinkInit(totNumLinks);
		
		for(int src : this.fixedNodes){
			for(int dst : this.nodes)
			{
				constructEdge(src, dst);

				if(src != dst && network.distanceTo(src, dst) <= interferenceRange){
					network.addInterferingNode(dst, src);
				}
			}
		}

		colorNodes();	
	}
	
	private void constructEdge(int src, int dst) throws InvalidNodeId, InvalidLinkId
	{
		if(src != dst){
			
    		//double distance = network.distanceTo(src, dst);

			int e = network.createLink(src, dst, 0.0);   // Check if this achieves the goal (source: Link* e = Link::Create(m_edge_id++, src, dst);)

			if(e > 0){
				links.add(e);
				List<Integer> srcOutLinks = network.getOutLinks(src);
				List<Integer> dstInLinks = network.getInLinks(dst);
				
				srcOutLinks.add(e);
				dstInLinks.add(e);
				
				network.setOutLinks(src, srcOutLinks);    // Check it works as expected (source: src->get_OutLinks().push_back(e);)
				network.setInLinks(dst, dstInLinks); 	// Same here: dst->get_InLinks().push_back(e);
			}
		}
	}
	
	private void updateAllLinksCost() throws Exception{
		
		int T = interferenceModel.getSlotCount();

		double estimatedAverageQueueDelay = packetLength/acousticLinkBitRate //m_InterferenceModel->get_SlotTime() 
			* (T+1.0)/2.0;

		for(int node : nodes)
		{
			network.setAverageChannelAccessDelay(node, estimatedAverageQueueDelay);
		}

		for(int link: links)
			updateLinkCost(link);
		
	}

	private void updateLinkCost(int link) throws Exception {

		double distance = network.getDistance(link);

		double sum = this.objective.getComponentWeight(objectiveComponentId.AverageDelay) + 
			(this.objective.getComponentWeight(objectiveComponentId.AverageEnergy));

		//The cost of communication on edge (vi,vj)
		network.setLinkCost(link, ((
			//delay = transmission delay+propagation delay
			this.objective.getComponentWeight(objectiveComponentId.AverageDelay)*
				( network.getAverageChannelAccessDelay(network.sourceNode(link)) +
				  packetLength/acousticLinkBitRate/acousticLinkPeakUtilization +
				  distance/acousticPropagationVelocity)
			//energy = TransmissionPower*transmission time
			+ (objective.getComponentWeight(objectiveComponentId.AverageDelay))*
		  ( packetLength/acousticLinkBitRate * network.getPowerLevel(link).getTransmissionPower()))/sum));
		
	}
	
}
