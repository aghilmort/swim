package swim.core.network.io;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.LinkedList;
import java.util.Vector;

import swim.core.network.FlowModel;
import swim.core.network.UWSensorNetwork;
import swim.core.network.ODPair;
import swim.core.network.PolynomialFlowModel;
import swim.core.network.Demand;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.err.*;

public class NetworkReader {
	
	private String fileName;
	private FileReader inputFile;
	private LineNumberScanner lineScanner;
	private UWSensorNetwork network;
	
	/*
	 * Open a text file for reading network topology and/or demand
	 * 
	 * fileName: file to open
	 */
	public NetworkReader(String fileName) throws FileNotFoundException{
		this.fileName = fileName;
		inputFile = new FileReader(fileName);
		lineScanner = new LineNumberScanner(inputFile);		
	}
	
	/*
	 * read network topology, i.e. links and link properties
	 */
	public UWSensorNetwork readTopology() throws InvalidInputException{
		network = null;
		
		try{
			//Read network specifications
			//Number of zones
			int zoneCount = lineScanner.nextInt();
			
			//Number of nodes
			int nodeCount = lineScanner.nextInt();

			//First through node
			//TODO: handle non-through nodes
			@SuppressWarnings("unused")
			int firstThroughNode = lineScanner.nextInt();
			
			//read number of links
			int linkCount = lineScanner.nextInt();						

			//construct network
			network = new UWSensorNetwork(nodeCount, zoneCount, linkCount);

			//read and construct links			
			for(int i=0; i<linkCount; i++){
				int 	from = lineScanner.nextInt();
				int 	to = lineScanner.nextInt();
				double 	capacity = lineScanner.nextDouble();
				double 	distance = lineScanner.nextDouble();
				double 	travelTime = lineScanner.nextDouble();
				double 	B = lineScanner.nextDouble();
				double 	power = lineScanner.nextDouble();
				double 	speed = lineScanner.nextDouble();
				int 	linkType = lineScanner.nextInt();
				
				
				//FlowModel flowModel = new PolynomialFlowModel(B, power).new WithCutoff();
				FlowModel flowModel = new PolynomialFlowModel(network, B, power);
				
				network.addLink(from, to, capacity, distance, travelTime, speed, linkType, flowModel);
			}
			
			//read and construct nodes
			for(int j=0; j<nodeCount; j++){
				LinkedList<Integer> inLinks = (LinkedList<Integer>) network.getInLinks(j+1);
				LinkedList<Integer> outLinks = (LinkedList<Integer>) network.getOutLinks(j+1);
				double 	x = lineScanner.nextDouble();
				double 	y = lineScanner.nextDouble();
				double 	z = lineScanner.nextDouble();
				double 	g = lineScanner.nextDouble();
				double transmissionPower;
				double communicationRange;
				double interferenceRange;
				Vector<PowerLevel> powerLevels = new Vector<PowerLevel>();
				int powerLevelCount = lineScanner.nextInt();
				for(int k=0; k<powerLevelCount; k++){
					transmissionPower = lineScanner.nextDouble();
					communicationRange = lineScanner.nextDouble();
					interferenceRange = lineScanner.nextDouble();
					powerLevels.add(new PowerLevel(transmissionPower, communicationRange, interferenceRange));
				}
				double deploymentCost = lineScanner.nextDouble();
				double transportEnergyRate = lineScanner.nextDouble();
				double idleListeningPower = lineScanner.nextDouble();
				double receivingPower = lineScanner.nextDouble();
				double initialEnergy = lineScanner.nextDouble();
				int nodeType = lineScanner.nextInt();

				network.addNode(inLinks, outLinks, x, y, z, g, powerLevels, deploymentCost, transportEnergyRate, nodeType, idleListeningPower, receivingPower, initialEnergy);
			}
			
			return network;
			
		}
		catch(Exception e){
			
			throw new InvalidInputException(fileName, lineScanner.getLineNumber(), e);
			
		}
	}
	/*
	 * network: network topology for the reader to assign demand to 
	 */
	public void readDemand(UWSensorNetwork network)throws InvalidInputException{
		this.network = network;
		readDemand();
	}
	
	/**Reads demand from the input file.
	 * @return Demand object that maps O-D pairs and their corresponding demand 
	 * @throws InvalidInputException
	 */
	public Demand readDemand() throws InvalidInputException{
		if(network==null){
			throw new NullPointerException("Can't read demand because network topology is null!\nYou must first call readTopology or call setTopology with a valid Network object.");
		}
				
		try {
			double previousDemand = 0.0;
			int srcCount = lineScanner.nextInt();
			@SuppressWarnings("unused")
			int pairCount = lineScanner.nextInt();
			Demand demand = new Demand(network);
			for(int i=0; i<srcCount; i++){
				int dstCount = lineScanner.nextInt();
				int initNode = lineScanner.nextInt();
				for(int j=0; j<dstCount; j++){
					int termNode = lineScanner.nextInt();
					double volume = lineScanner.nextDouble();
					ODPair od = new ODPair(initNode, termNode);
					if(initNode==termNode){
						if(volume!=0){
							throw new InvalidDemand(od, volume);
						}
					}else{
						volume = network.getProcessingModel().processData(initNode, volume, previousDemand);
						demand.add(od, volume);	
						previousDemand = volume;
					}
				}
			}

			return demand;
		}
		catch(Exception e){
			
			throw new InvalidInputException(fileName, lineScanner.getLineNumber(), e);
			
		}
	}
	
}
