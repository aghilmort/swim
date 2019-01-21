package swim.core.network;

import java.util.*;

//import edu.uci.ics.jung.graph.DirectedSparseGraph;
//import edu.uci.ics.jung.graph.util.EdgeType;

import swim.core.network.FlowModel;
import swim.core.network.CostModel;
import swim.core.network.MobilityModel;
import swim.core.network.PowerModel.PowerLevel;
import swim.core.network.err.*;
import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.misc.Property;
import swim.core.network.misc.PropertyFunction;
import swim.core.network.misc.PropertyMap;
import swim.core.network.misc.ReadOnlyProperty;
import swim.core.network.misc.PropertyFunction.Function;

/**A Network represents the directed-graph topology , i.e. nodes, directed links
 * and allows properties to be associated with links 
 * @author Saleh Ibrahim 
 */
public class UWSensorNetwork extends Network{

	// ======================================================================================================
	// Link constants:
	// ======================================================================================================
		

	// ======================================================================================================
	// Node constants:
	// ======================================================================================================
		
	//private static final double DEFAULT_POWER_LEVEL = 100.0; // mW
	
	public static double PACKET_LENGTH = 400.0;						//400bits = 50 bytes
	public static double ACOUSTIC_LINK_PEAK_UTILIZATION = 1.0; 		//Assume no collisions
	public static double PROCESSING_TIME = 0.01; 					//Assume processing takes 1 msec
	public static double COMPRESSION_RATIO = 0.6; 					//Assume compression ration = 0.6 
																	// (i.e. new file is 0.6 the size of the
																	//  original file)
	public static double PROCESSING_POWER = 0.025;					//Assume processing takes 25 mWatt
	
	// ======================================================================================================
	// Link properties:
	// ======================================================================================================
		
	/**
	 * Storage for link capacity
	 */
	private Property<Double> lpCapacity;

	/**
	 * Storage for link distance
	 */
	private Property<Double> lpDistance;
	
	private Property<FlowModel> lpFlowModel;	

	/**
	 * Map for link free flow travel time
	 */
	private ReadOnlyProperty<Double> lpFreeFlowTravelTime;
	
	/**
	 * Storage for link type
	 */
	private Property<Integer> lpLinkType;
	
	/**
	 * Map for link flow
	 */
	private DoublePropertyMap lpFlow;
	
	/**
	 * Map for Residual capacity
	 */
	private ReadOnlyProperty<Double> lpResidualCapcity;	
	
	/**
	 * Storage for link cost per unit flow
	 */
	private Property<Double> lpCost;
	
	/**
	 *  Storage for link power level
	 */
	private  Property<PowerLevel> lpPowerLevel;
	
	// ======================================================================================================
	// Node properties:
	// ======================================================================================================
	
	/**
	 * Storage for node power level
	 */
	//private Property<Double> npPowerLevel;
	
	/**
	 * Storage for node deployment cost
	 */
	private Property<Double> npDeploymentCost;
	
	/**
	 * Storage for node type
	 * 1 --> Is sink
	 * 2 --> Regular node
	 * 3 --> Processing node
	 */
	protected Property<Integer> npNodeType;
	
	protected Property<Double> npClockTime;
	
	protected Property<Vector<Double>> npCoordinates;
	
	private Property<Double> npResidualEnergy;
	
	protected Property<Double> npDataGenerationRate;

	private Property<Double> npAverageChannelAccessDelay;
	
	private Property<Double> npQueueingDelay;
	
	protected Property<Boolean> npState;
	
	protected Property<Vector<PowerModel.PowerLevel>> npPowerLevels;
	
	protected Property<Double> npIdleListeningPower;
	
	protected Property<Double> npReceivingPower;
	
	protected Property<Double> npTransmissionPower;

	protected Property<Double> npCompressionRatio;
	
	protected Property<Double> npTransportEnergyRate;
	
	protected Property<Double> npInitialEnergy;
	
	protected Property<Vector<Double>> npVelocity;
	
	protected Property<Vector<Integer>> npInterferingNodes;
	
	//Connectivity Testing
	protected Property<Boolean> npIsConnected;
	
	//Traversal
	protected Property<Boolean> npIsMarked; 
	
	//Graph coloring
	protected Property<Integer> npColor;
        
        protected Property<String> npName;
	
	// ======================================================================================================
	// Link property functions:
	// ======================================================================================================
	
	/**
	 * Link property functions
	 */
	private Function<Double> fTravelTime, fTravelTimeIntegral, fTravelCost, fTravelCostIntegral;

	/**
	 * Link property function maps 
	 * pTravelTime:Map for link travel time (at the given load)
	 * pTravelTimeIntegral: Map for link travel time integral (at the given load)
	 * Integrate with load to calculate travelTime
	 * pTravelCost: link travel cost map
	 * pTravelCostIntegral: Link travel cost integral map 
	 */
	private ReadOnlyProperty<Double> pTravelTime, pTravelTimeIntegral, pTravelCost, pTravelCostIntegral;

	/**
	 * Link cost (multiplied by flow) 
	 */
	private FlowCostFunction lpTotalTravelTime, lpTotalTravelTimeIntegral, lpTotalTravelCost, lpTotalTravelCostIntegral;

	// ======================================================================================================
	// Node property functions:
	// ======================================================================================================
	
	
	
	// ======================================================================================================
	// Network models:
	// ======================================================================================================
	
	/**
	 * The function used for calculating the cost of each link 
	 * as a function of travel time, distance and toll.
	 */
	private CostModel costModel;
	
	private MobilityModel mobilityModel;
	
	private PowerModel powerModel;
	
	private ProcessingModel processingModel;
	
	/**
	 * Default link flow model. Used when no flowModel is assigned to a link
	 */
	private static FlowModel defaultFlowModel = null; //new PolynomialFlowModel(new UWSensorNetwork(), 0, 1);

	
	// ======================================================================================================
	// Model getters:
	// ======================================================================================================

	public PowerModel getPowerModel(){
		return this.powerModel;
	}
	
	public CostModel getCostModel(){
		return this.costModel;
	}
	
	public MobilityModel getMobilityModel(){
		return this.mobilityModel;
	}
	
	public ProcessingModel getProcessingModel(){
		return this.processingModel;
	}
	
	
	// ======================================================================================================
	// Constructors:
	// ======================================================================================================
	
	/**
	 * Construct a network with the given number of nodes 
	 * links, demand and/or assignments are to be added later on 
	 * using addLink(), assign() 
	 */
	public UWSensorNetwork(int nodeCount, int zoneCount, int linkCount){
		super(nodeCount, zoneCount, linkCount);		
		initProperties();
		initFunctionProperties();
	}

	/**Network 'convenience' constructor
	 * Creates an empty network (has no nodes nor links). This allows the
	 * instantiation of a network while deferring the addition of nodes
	 * and links to a later time 
	 */
	public UWSensorNetwork(){
		super();		
		initProperties();
		initFunctionProperties();
	}	
	
	/**
	 * Construct a network with the same number of nodes
	 * and the same links and cost model of a given network
	 * @param o network to copy
	 */
	public UWSensorNetwork(UWSensorNetwork o){
		super(o);

		lpCapacity = o.lpCapacity.clone(this);
		lpFlow = o.lpFlow.clone(this);
		lpDistance = o.lpDistance.clone(this);
		lpFlowModel = o.lpFlowModel.clone(this);
		lpFreeFlowTravelTime =  o.lpFreeFlowTravelTime.clone(this);
		pTravelTime = o.pTravelTime.clone(this);
		pTravelTimeIntegral = o.pTravelTimeIntegral.clone(this);
		pTravelCost = o.pTravelCost.clone(this);
		pTravelCostIntegral = o.pTravelCostIntegral.clone(this);
		lpLinkType = o.lpLinkType.clone(this);
		lpResidualCapcity = o.lpResidualCapcity.clone(this);
		lpPowerLevel = o.lpPowerLevel.clone(this);
		lpCost = o.lpCost.clone(this);
		
		npClockTime = o.npClockTime.clone(this);
		npCoordinates = o.npCoordinates.clone(this);
		npDeploymentCost = o.npDeploymentCost.clone(this);
		npNodeType = o.npNodeType.clone(this);
		npResidualEnergy = o.npResidualEnergy.clone(this);
		npDataGenerationRate = o.npDataGenerationRate.clone(this);
		npAverageChannelAccessDelay = o.npAverageChannelAccessDelay.clone(this);
		npQueueingDelay = o.npQueueingDelay.clone(this);
		npState = o.npState.clone(this);
		npPowerLevels = o.npPowerLevels.clone(this);
		npIdleListeningPower = o.npIdleListeningPower.clone(this);
		npReceivingPower = o.npReceivingPower.clone(this);
		npTransmissionPower = o.npTransmissionPower.clone(this);
		npInitialEnergy = o.npInitialEnergy.clone(this);
		npTransportEnergyRate = o.npTransportEnergyRate.clone(this);
		npVelocity = o.npVelocity.clone(this);
		npInterferingNodes = o.npInterferingNodes.clone(this);
		npIsConnected = o.npIsConnected.clone(this);
		npIsMarked = o.npIsMarked.clone(this);
		npColor = o.npColor.clone(this);
		npCompressionRatio = o.npCompressionRatio.clone(this);
		npName =  o.npName.clone(this);
                        
		costModel = new CostModel(o.costModel);
		mobilityModel = new MobilityModel(o.mobilityModel);
		powerModel = new PowerModel(o.powerModel);
		processingModel = new ProcessingModel(o.processingModel);
	}
	
	// ======================================================================================================
	// Model setters:
	// ======================================================================================================	

	/**Change the cost calculation model
	 * @see tn.CostModel CostModel
	 * @param costModel cost model object
	 */
	public void setCostModel(CostModel costModel) {
		this.costModel = costModel;
	}
	
	public void setMobilityModel(MobilityModel mobilityModel){
		this.mobilityModel = mobilityModel;
	}

	// ======================================================================================================
	// Misc. network functions:
	// ======================================================================================================	

	private void initProperties() {
		lpCapacity = new DoublePropertyMap("link", "Cpcty", this);
		lpFlow = new DoublePropertyMap("link", "Flow", this);
		lpDistance = new DoublePropertyMap("link", "Length", this);
		lpFlowModel = new PropertyMap<FlowModel>("link", "Flow-Model", this, defaultFlowModel);
		lpFreeFlowTravelTime =  new DoublePropertyMap("link", "FFTTime", this); 
		lpLinkType = new PropertyMap<Integer>("link", "LinkType", this, 0);
		lpPowerLevel = new PropertyMap<PowerLevel>("link", "PowerLevel", this, null);
		lpCost = new DoublePropertyMap("link", "LinkCost", this, null);
		
		
		npClockTime = new DoublePropertyMap("node", "ClockTime", this);
		npCoordinates = new PropertyMap<Vector<Double>>("node", "Coordinates", this, null);
		npDeploymentCost = new DoublePropertyMap("node", "DeploymentCost", this);
		npNodeType = new PropertyMap<Integer>("node", "NodeType", this, 2);
		npResidualEnergy = new DoublePropertyMap("node", "ResidualEnergy", this);
		npDataGenerationRate = new DoublePropertyMap("node", "DataGenerationRate", this);
		npAverageChannelAccessDelay = new DoublePropertyMap("node", "AverageChannelAccessDelay", this);
		npQueueingDelay = new DoublePropertyMap("node", "QueueingDelay", this);
		npState = new PropertyMap<Boolean>("node", "NodeState", this, true);
		npPowerLevels = new PropertyMap<Vector<PowerLevel>>("node", "PowerLevels", this, null);
		npIdleListeningPower = new DoublePropertyMap("node", "IdleListeningPower", this);
		npReceivingPower = new DoublePropertyMap("node", "ReceivingPower", this);
		npTransmissionPower = new DoublePropertyMap("node", "TransmissionPower", this);
		npInitialEnergy = new DoublePropertyMap("node", "InitialEnergy", this);
		npTransportEnergyRate = new DoublePropertyMap("node", "TransportEnergyRate", this);
		npVelocity = new PropertyMap<Vector<Double>>("node", "NodeVelocity", this, null);
		npInterferingNodes = new PropertyMap<Vector<Integer>>("node", "InterferingNodes", this, new Vector<Integer>());
		npIsConnected = new PropertyMap<Boolean>("node", "NodeIsConnected", this, true);
		npIsMarked = new PropertyMap<Boolean>("node", "NodeIsMarked", this, false);
		npColor = new PropertyMap<Integer>("node", "NodeColor", this, null);
		npCompressionRatio = new PropertyMap<Double>("node", "CompressionRatio", this, 1.0);
		npName = new PropertyMap<String>("node", "NodeName", this, null);
                        
		costModel = new CostModel(this, 0.3, 0.7);
		mobilityModel = new MobilityModel(npClockTime,npCoordinates);
		powerModel = new PowerModel(npPowerLevels, npIdleListeningPower, npReceivingPower, npInitialEnergy, npTransportEnergyRate);
		processingModel = new ProcessingModel(this, UWSensorNetwork.COMPRESSION_RATIO);		
	}
	
	private void initFunctionProperties() {
		fTravelTime = new TravelTimeFunction();
		fTravelTimeIntegral = new TravelTimeIntegralFunction();
		fTravelCost = new TravelCostFunction();
		fTravelCostIntegral = new TravelCostIntegralFunction();
		
		pTravelTime = new PropertyFunction<Double>("link", "TTime", this, fTravelTime);
		pTravelTimeIntegral = new PropertyFunction<Double>("link", "TTimeI", this, fTravelTimeIntegral);
		pTravelCost = new PropertyFunction<Double>("link", "TCost", this, fTravelCost);
		pTravelCostIntegral = new PropertyFunction<Double>("link", "TCostI", this, fTravelCostIntegral);
		lpResidualCapcity = new PropertyFunction<Double>("link", "Residue", this, new ResidualCapacityFunction());

		lpTotalTravelTime = new FlowCostFunction("f*TTime", fTravelTime);
		lpTotalTravelTimeIntegral = new FlowCostFunction("f*TTimeI", fTravelTimeIntegral);
		lpTotalTravelCost = new FlowCostFunction("f*TCost", fTravelCost);
		lpTotalTravelCostIntegral = new FlowCostFunction("f*TCostI", fTravelCostIntegral);		
	}	
	
	/**Assign a given traffic load to the network, assigning each user to the best path
	 */
	public void loadAssignment(Assignment assignment) {		
		setFlow(assignment.getFlow());
	}
	
	public Assignment combineAssignment(double alpha, Assignment assignment1, double beta,
			Assignment assignment2) {
	
		Assignment result = new Assignment(this, assignment1.getSize());
		if(alpha>0){
			for(ODPairAssignment e: assignment1){
				result.add(e.route, alpha * e.volume);
			}			
		}
		
		if(beta>0){
			for(ODPairAssignment e: assignment2){
				result.add(e.route, beta * e.volume);
			}			
		}
		return result;
	}	

	@Override
	protected Object clone() throws CloneNotSupportedException {
		return new UWSensorNetwork(this);
	}
		
	// ======================================================================================================
	// Link functions:
	// ======================================================================================================	

	/** add a link to the network
	 * 		id: link id
	 * 		from: node id
	 * 		to: node id
	 * 		distance: travel distance of the link
	 * 		capacity: maximum throughput
	 * 		travelTime: ideal travel time
	 * 		speed: posted speed limit
	 * 		toll: toll associated with link
	 * 		flowModel: determines the actual travelTime as a function of flow 
	 * 		linkType: unknown
	 * @throws InvalidLinkId 
	 */
	public void addLink(int from, int to, double capacity, double distance,
			double freeFlowTravelTime, double speed, int linkType,
			FlowModel flowModel) throws InvalidNodeId, InvalidLinkId{
		
		int id = addLink(from, to);
		
		this.lpCapacity.set(id, capacity);
		this.lpDistance.set(id, distance);
		((PropertyMap<Double>) this.lpFreeFlowTravelTime).set(id, freeFlowTravelTime);
		this.lpLinkType.set(id , linkType);
		this.lpFlow.set(id, 0.0);
		this.lpFlowModel.set(id, flowModel);
	}
	
	public int addLink(int from, int to, 
			double distance, PowerLevel powerLevel, double cost) throws InvalidNodeId, InvalidLinkId{
		int id = super.addLink(from, to);
		
		this.lpDistance.set(id, distance);
		this.lpPowerLevel.set(id, powerLevel);
		this.lpCost.set(id, cost);
		
		return id;
		
	}	

	public DoublePropertyMap resetFlow(){
		DoublePropertyMap oldFlow = lpFlow;
		lpFlow = new DoublePropertyMap("link", "Flow", this);
		return oldFlow;
	}

	/** Replace the current flow map with the given one
	 * @param flowMap the new flow map
	 * @return the old flow map
	 */
	public DoublePropertyMap resetFlow(DoublePropertyMap flowMap){
		DoublePropertyMap oldFlow = lpFlow;
		setFlow(flowMap);
		return oldFlow;
	}
	
	public void addFlow(DoublePropertyMap flowMap) {
		lpFlow.add(flowMap);
	}

	public void addFlow(Path path, double additionalFlow) {
		lpFlow.add(path.getLinks(), additionalFlow);
	}

	public DoublePropertyMap cloneFlow() {
		return lpFlow.clone();
	}	
	
	public void addFlow(int linkId, double additionalFlow){
		lpFlow.set(linkId, lpFlow.get(linkId).doubleValue() + additionalFlow);
	}
	
	public int createLink(int from, int to, double cost) throws InvalidNodeId, InvalidLinkId{
		
		double distance = this.distanceTo(from, to);
		PowerLevel level = null;
		
		for(int i = 0; i < powerModel.getPowerLevelCount(from) ; i++){
			PowerLevel p = powerModel.getPowerLevel(from, i);
			if(p.getCommunicationRange() >= distance){
				level = p;
				break;
			}
		}
		
		if(level == null){
			return 0;
		}else{
			return this.addLink(from, to, distance, level, cost);
		}
	}
	
	// ======================================================================================================
	// Node functions:
	// ======================================================================================================	
		
	/** Test whether a gateway node is active
	 *  This method is used as part of obtaining the solution of the deployment 
	 *  optimization problem. It allows the user to check whether a gateway is part 
	 *  of the solution suggested by Problem.findBestGatewayLocations A gateway node 
	 *  that is inactive means it will not be deployed. This method however has no 
	 *  meaning for nodes that are elements of the FixedNodes set.
	 */
	public Boolean isActive(int nodeId){
		return this.npState.get(nodeId);
	}
	
	//true if the node is a gateway node
	public Boolean isGateway(int nodeId){
		return (this.npNodeType.get(nodeId) == 1 ? true : false);
	}
	
	// add a node to the list of interfering nodes (within communication range of destination)
	public void addInterferingNode(int nodeId, int node){
		Vector<Integer> temp = getInterferingNodes(nodeId);
		temp.add(node);
		setInterferingNodes(nodeId, temp);
	}
	
	//Calculate Distance
	public double distanceTo(int thisNode, int otherNode){
		
		double x = this.getCoordinates(thisNode).get(0).doubleValue();
		double y = this.getCoordinates(thisNode).get(1).doubleValue();
		double z = this.getCoordinates(thisNode).get(2).doubleValue();
		
		double vj_x = this.getCoordinates(otherNode).get(0).doubleValue();
		double vj_y = this.getCoordinates(otherNode).get(1).doubleValue();
		double vj_z = this.getCoordinates(otherNode).get(2).doubleValue();
		
		return Math.sqrt((x - vj_x)*(x - vj_x) + 
				(y - vj_y)*(y - vj_y) + 
				(z - vj_z)*(z - vj_z));
	}
	
	//Mark as disconnected
	public void resetConnectedFlag(int nodeId){
		this.npIsConnected.set(nodeId, false);
	}
	
	//True if marked as connected
	public Boolean isConnected(int nodeId){
		return this.npIsConnected.get(nodeId);
	}
	
	// Mark as visited
	public void mark(int nodeId){
		this.npIsMarked.set(nodeId, true);
	}
	
	//Mark as unvisited
	public void resetMark(int nodeId){
		this.npIsMarked.set(nodeId, false);
	}
	
	//True if marked as visited
	public Boolean isMarked(int nodeId){
		return this.isMarked(nodeId);
	}
	
	//Remove node from subgraph
	public void resetColor(int nodeId){
		this.npColor.set(nodeId, 0);
	}
	
	/**
	 * add a node to the network
	 * @throws InvalidLinkId 
	 * @throws InvalidNodeId 
	 */
	public int addNode(LinkedList<Integer> inLinks,	// in links 
			LinkedList<Integer> outLinks, 			// out links
			//Boolean type, 							// node type 
			double x, double y, double z, 			// node coordinates
			double g, 								// node's data generation rate
			//Boolean isGateway,						// node type: true = gateway, false = regular node
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			Integer nodeType,							// node type
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy					// node's initial energy
			) throws InvalidNodeId, InvalidLinkId{
		
		int id = super.addNode(inLinks, outLinks);
		
		this.setCoordinates(id, x, y, z);
		this.npDataGenerationRate.set(id, (Double)g);
		this.npPowerLevels.set(id, powerLevels);
		this.mobilityModel.setNodeCoordinates(id, this.getCoordinates(id));
		this.mobilityModel.setNodeTime(id, 0.0);
		this.npDeploymentCost.set(id, deploymentCost);
		this.powerModel.setTransportEnergyRate(id, transportEnergyRate);
		this.npNodeType.set(id, nodeType);
		this.powerModel.setIdleListeningPower(id, idleListeningPower);
		this.powerModel.setReceivingPower(id, receivingPower);
		this.powerModel.setInitialEnergy(id, initialEnergy);
		
		
		return id;
	}
	
	/**
	 * add a node to the network
	 * @throws InvalidLinkId 
	 * @throws InvalidNodeId 
	 */
	public int addNode(LinkedList<Integer> inLinks,	// in links 
			LinkedList<Integer> outLinks, 			// out links
			//Boolean type, 							// node type 
			double x, double y, double z, 			// node coordinates
			double g, 								// node's data generation rate
			//Boolean isGateway,						// node type: true = gateway, false = regular node
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy					// node's initial energy
			) throws InvalidNodeId, InvalidLinkId{
		
		return addNode(inLinks, outLinks,/* type,*/x, y, z, g, 
				/*isGateway,*/powerLevels, deploymentCost, 
				transportEnergyRate, 1, idleListeningPower,
				receivingPower, initialEnergy);
	}	
	
	/**
	 * add a node to the network
	 * @throws InvalidLinkId 
	 * @throws InvalidNodeId 
	 */
	public int addNode(LinkedList<Integer> inLinks,	// in links 
			LinkedList<Integer> outLinks, 			// out links
			//Boolean type, 							// node type 
			double x, double y, double z, 			// node coordinates
			//Boolean isGateway,						// node type: true = gateway, false = regular node
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			Integer nodeType,							// node type
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy					// node's initial energy
			) throws InvalidNodeId, InvalidLinkId{
		
		return addNode(inLinks, outLinks,/* type,*/x, y, z, 0.0, 
				/*isGateway,*/powerLevels, deploymentCost, 
				transportEnergyRate, nodeType, idleListeningPower,
				receivingPower, initialEnergy);
	}		
	
	/**
	 * add a node to the network
	 * @throws InvalidLinkId 
	 * @throws InvalidNodeId 
	 */
	public int addNode(LinkedList<Integer> inLinks,	// in links 
			LinkedList<Integer> outLinks, 			// out links
			//Boolean type, 							// node type 
			double x, double y, double z, 			// node coordinates
			//Boolean isGateway,						// node type: true = gateway, false = regular node
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy					// node's initial energy
			) throws InvalidNodeId, InvalidLinkId{
		
		return addNode(inLinks, outLinks,/* type,*/x, y, z, 0.0, 
				/*isGateway,*/powerLevels, deploymentCost, 
				transportEnergyRate, 1, idleListeningPower,
				receivingPower, initialEnergy);
	}		
	
	/**
	 * add a node as a candidate node
	 * @throws InvalidLinkId 
	 * @throws InvalidNodeId 
	 */
	public int candidateNode(LinkedList<Integer> inLinks,	// in links 
			LinkedList<Integer> outLinks, 			// out links
			//Boolean type, 							// node type 
			double x, double y, double z, 			// node coordinates
			//Boolean isGateway,						// node type: true = gateway, false = regular node
			Vector<PowerLevel> powerLevels,			// node's power levels
			double deploymentCost,					// node's deployment cost
			double transportEnergyRate,				// node's consumed energy rate during movement (verify)
			double idleListeningPower,				// node's idle listening power
			double receivingPower,					// node's receiving power
			double initialEnergy					// node's initial energy
			) throws InvalidNodeId, InvalidLinkId{
		return this.addNode(inLinks, outLinks, x, y, z, 0.0, powerLevels, deploymentCost, transportEnergyRate, 1, idleListeningPower, receivingPower, initialEnergy);
	}
	
	/**
	 * add a default candidate node
	 * @throws InvalidLinkId 
	 * @throws InvalidNodeId 
	 */
	public int candidateNode( 		
			double x, double y			 			// node x, y coordinates
			) throws InvalidNodeId, InvalidLinkId{
		
		return this.addNode(new LinkedList<Integer>(), 
				new LinkedList<Integer>(),
				x, y, 0.0, 0.0, 
				new Vector<PowerLevel>(), 
				0.0, 0.0, 1, 0.0, 0.0, 0.0);
	}

	//TODO: create an addNode function for adding a default node
	//TODO: VERY IMPORTANT: replace mobility model argument in all addNode 
	//      functions with its two arguments for the node (i.e. time and coordinates). 
	//      The should be done because I replaced the mobility model for individual 
	//      nodes with a single network-wide mobility model that holds the individual 
	//      nodes' mobility properties.
	
	public int sourceNode(int linkId){
		return (int)super.lpFrom.get(linkId);
	}
	
	public int targetNode(int linkId){
		return (int)super.lpTo.get(linkId);
	}
	
	// ======================================================================================================
	// Individual node property getters:
	// ======================================================================================================
	
        public String getName(int nodeId) {
            return npName.get(nodeId);
        }
        
	/** Get type of Node
	*/
	public int getNodeType(int nodeId){
		return npNodeType.get(nodeId);
	}
	
	/** Retrieve node location
	 *  This method allows the user to retrieve the gateway placement suggested by Problem.findBestGatewayLocations
	 */
	public Vector<Double> getCoordinates(int nodeId){
		return this.npCoordinates.get(nodeId);
	}
	
	/** Get the local node time
	*/
	public double getTime(int nodeId){
		return this.npClockTime.get(nodeId).doubleValue();
	}
	
	public double getAverageChannelAccessDelay(int nodeId){
		return this.npAverageChannelAccessDelay.get(nodeId);
	}
	
	public double getQueueingDelay(int nodeId){
		return this.npQueueingDelay.get(nodeId);
	}

	public double getListeningPower(int nodeId){
		return powerModel.getIdleListeningPower(nodeId);
	}
	
	public double getReceivingPower(int nodeId){
		return powerModel.getReceivingPower(nodeId);
	}
	
	public double getResidualEnergy(int nodeId){
		return this.npResidualEnergy.get(nodeId);
	}

	//Node x-coordinate
	public double getX(int nodeId){
		return this.getCoordinates(nodeId).get(0);
	}
	
	//Node y-coordinate
	public double getY(int nodeId){
		return this.getCoordinates(nodeId).get(1);
	}
	
	//Node z-coordinate
	public double getZ(int nodeId){
		return this.getCoordinates(nodeId).get(2);
	}	

	//data generation rate (data units per second)
	public double getG(int nodeId){
		return this.npDataGenerationRate.get(nodeId).doubleValue();
	}
	
	//Get the velocity
	public Vector<Double> getVelocity(int nodeId){
		return this.npVelocity.get(nodeId);
	}
	
	// Get list of edges entering a node
	public List<Integer> getInLinks(int nodeId){
		return super.getInLinks()[nodeId];
	}

	// Set list of edges entering a node
	public void setInLinks(int nodeId, List<Integer> inLinks){
		super.setInLinks(nodeId, inLinks);
	}
	
	//Get list of edges leaving a node
	public List<Integer> getOutLinks(int nodeId){
		return super.getOutLinks()[nodeId];
	}
	
	// Set list of edges leaving a node
	public void setOutLinks(int nodeId, List<Integer> outLinks){
		super.setOutLinks(nodeId, outLinks);
	}
	
	//list of interfering nodes (within communication range of destination)
	public Vector<Integer> getInterferingNodes(int nodeId){
		return this.npInterferingNodes.get(nodeId);
	}

	//Get node's current subgraph id
	public int getColor(int nodeId){
		return this.npColor.get(nodeId);
	}
	
	public double getNodeCompressionRatio(int nodeId) {
		return npCompressionRatio.get(nodeId);
	}

	
	// ======================================================================================================
	// Individual node property setters:
	// ======================================================================================================

        public void setName(int nodeId, String name) {
            npName.set(nodeId, name);
        }        
        
	/** Change coordinates
	 */
	public void setCoordinates(int nodeId, double x, double y, double z){
		Vector<Double> coords = new Vector<Double>();
		coords.add((Double)x);
		coords.add((Double)y);
		coords.add((Double)z);
		this.npCoordinates.set(nodeId, coords);
	}
	
	/** Set/reset the sink flag.
	 * When set, the node doesn't forward any traffic over acoustic links
	 */
	public void setIsSink(int nodeId, Boolean flag){
		this.npNodeType.set(nodeId, (flag == true ? 1 : 2));
	}
	
	public void setNodeType(int nodeId, int type){
		this.npNodeType.set(nodeId, type);
	}
	
	public void setActive(int nodeId, Boolean state){
		this.npState.set(nodeId, state);
	}
	
	/** Set the residual energy level in watt.seconds
	 */
	public void setResidualEnergy(int nodeId, double wattSeconds){
		this.npResidualEnergy.set(nodeId, wattSeconds);
	}
	public void setResidualEnergy(int nodeId){
		this.npResidualEnergy.set(nodeId, 1e10);
	}
	
	/** Set data generation rate in packets per second
	 */
	public void setDataGenerationRate(int nodeId, double g){
		this.npDataGenerationRate.set(nodeId, g);
	}
	
	/** Set average channel access delay
	 */
	public void setAverageChannelAccessDelay(int nodeId, double seconds){
		this.npAverageChannelAccessDelay.set(nodeId, seconds);
	}
	public void setAverageChannelAccessDelay(int nodeId){
		this.npAverageChannelAccessDelay.set(nodeId, 0.0);
		// TODO: performance enhancement: do not store default values, create a constant for default
		// value, and return it when trying to get that link
	}	
	
	/** Set average channel access delay
	 */
	public void setQueueingDelay(int nodeId, double seconds){
		this.npQueueingDelay.set(nodeId, seconds);
	}
	public void setQueueingDelay(int nodeId){
		this.npQueueingDelay.set(nodeId, 0.0);
		// TODO: performance enhancement: do not store default values, create a constant for default
		// value, and return it when trying to get that link
	}

	public void setVelocity(int nodeId, double dx, double dy, double dz){
		Vector<Double> velocity = new Vector<Double>();
		velocity.add(dx);
		velocity.add(dy);
		velocity.add(dz);
		this.npVelocity.set(nodeId, velocity);
	}
	
	//Mark as connected
	public void setConnectedFlag(int nodeId){
		this.npIsConnected.set(nodeId, true);
	}

	//Put node in subgraph
	public Boolean setColor(int nodeId, int color){
		if(this.npColor.get(nodeId).intValue() == 0){
			this.npColor.set(nodeId, color);
			for(int link : this.getInLinks(nodeId)){
				this.setColor(this.getOrigin(link), color);
			}
			for(int link : this.getOutLinks(nodeId)){
				this.setColor(this.getDestination(link), color);
			}
			return true;
		}
		return false;
	}
	
	public void setDeploymentCost(int nodeId, double dollars){
		this.npDeploymentCost.set(nodeId, dollars);
	}	
	
	//data generation rate (data units per second)
	public void setG(int nodeId, double value){
		this.npDataGenerationRate.set(nodeId, value);
	}
	
	public void setNodeCompressionRatio(int nodeId, double value) {
		this.npCompressionRatio.set(nodeId, value);
	}	
	
	//set the list of interfering nodes (within communication range of destination)
	public void setInterferingNodes(int nodeId, Vector<Integer> nodes){
		this.npInterferingNodes.set(nodeId, nodes);
	}
	
	// ======================================================================================================
	// Individual link property getters:
	// ======================================================================================================
	
	public double getCapacity(int linkId){
		return lpCapacity.get(linkId).doubleValue();
	}

	public double getDistance(int linkId) {
		return lpDistance.get(linkId); 
	}

	public double getFlow(int linkId){
		return lpFlow.get(linkId).doubleValue();			
	}

	public double getFreeFlowTravelTime(int linkId) {
		return lpFreeFlowTravelTime.get(linkId);
	}

	public Number getMonitoringCost(int linkId, int sensorClass) {
		//TODO: implement link monitoring cost property 
		return 1;
	}

	public ReadOnlyProperty<Double> getTravelCost(int linkId){
		
		return lpTotalTravelCost;
	}

	public ReadOnlyProperty<Double> getTravelCostIntegral(int linkId){
		return lpTotalTravelCostIntegral;
	}

	public double getTravelTime(int linkId){
		return pTravelTime.get(linkId);
	}	
	
	public double getTravelTimeIntegral(int linkId){
		return pTravelTimeIntegral.get(linkId);
	}
	
	public double getResidualCapacity(int linkId) {
		return lpResidualCapcity.get(linkId);
	}
	
	public double getLinkCost(int linkId){
		return this.lpCost.get(linkId).doubleValue();
	}
	
	public PowerLevel getPowerLevel(int linkId){
		return this.lpPowerLevel.get(linkId);
	}
	
	// ======================================================================================================
	// Individual link property setters:
	// ======================================================================================================
		
	public void setLinkCost(int linkId, double cost){
		this.lpCost.set(linkId, cost);
	}
	
	public void setCapacity(int linkId, double capacity){
		this.lpCapacity.set(linkId, capacity);
	}
	
	// ======================================================================================================
	// Network-wide link property getters:
	// ======================================================================================================
		
	public ReadOnlyProperty<Double> getTotalTravelCost(){
		
		return lpTotalTravelCost;
	}

	public ReadOnlyProperty<Double> getTotalTravelTime(){
		
		return lpTotalTravelTime;
	}
	
	public ReadOnlyProperty<Double> getTotalTravelTimeIntegral(){
		
		return lpTotalTravelTimeIntegral;
	}

	public ReadOnlyProperty<Double> getTravelTime(){	
		return pTravelTime;
	}
	
	public DoublePropertyMap getFlow() {
		return lpFlow;
	}
	
	public ReadOnlyProperty<Double> getFreeFlowTravelTime() {
		return lpFreeFlowTravelTime;
	}
	
	public ReadOnlyProperty<Double> getTravelCost(){
		return pTravelCost;
	}
	
	public Property<Double> getLinkCost(){
		return lpCost;
	}
	
	// ======================================================================================================
	// Network-wide node property setters:
	// ======================================================================================================
			
	/** Set the node deployment cost, i.e. price plus initial deployment cost
	 */
	public void setDeploymentCost(Property<Double> dollars){
		this.npDeploymentCost = dollars;
	}
	
	public void setDeploymentCost(){
		this.npDeploymentCost = new DoublePropertyMap("node", "DeploymentCost", this, 0.0);
	}	

	
	// ======================================================================================================
	// Network-wide link property setters:
	// ======================================================================================================	
	
	public void setTravelTime(ReadOnlyProperty<Double> travelTime){
		this.pTravelTime = travelTime;
	}
	
	/** Makes the given flow map the flow map of the network
	 * @param flowMap the new flow map
	 */
	public void setFlow(DoublePropertyMap flowMap){
		lpFlow = flowMap; 
		addLinkProperty("Flow", flowMap);
	}

	public void setTravelCost(ReadOnlyProperty<Double> travelCost){
		this.pTravelCost = travelCost;
	}
	
	public void setFreeFlowTravelTime(ReadOnlyProperty<Double> freeFlowTravelTime) {
		lpFreeFlowTravelTime = freeFlowTravelTime;
		
	}
	
	// ======================================================================================================
	// Network-wide link property getters:
	// ======================================================================================================	
	
	// ======================================================================================================
	// Network-wide link functions:
	// ======================================================================================================	
	
	// ======================================================================================================
	// Network-wide node functions:
	// ======================================================================================================	
	public void clearNodes(){
		
		super.resetNodes();
		
		this.lpCapacity.reset();
		this.lpFlow.reset();
		this.lpDistance.reset();
		this.lpFlowModel.reset();
		this.lpLinkType.reset();
		this.lpPowerLevel.reset();
		this.lpCost.reset();
		this.lpFrom.reset();
		this.lpTo.reset();
		
	}
		
	
	// ======================================================================================================
	// Inner classes:
	// ======================================================================================================	
	
	class FlowCostFunction extends ReadOnlyProperty<Double>{
		final Function<Double> cost;
		
		public FlowCostFunction(String name, Function<Double> perUnitCost){
			super("link", name, UWSensorNetwork.this, 0.0);
			cost = perUnitCost;
		}

		@Override
		public Double get(int linkId) {
			
			return lpFlow.get(linkId)*cost.get(linkId);
		}

		@Override
		public ReadOnlyProperty<Double> clone(Network network) {
			return ((UWSensorNetwork)network).new FlowCostFunction(this.name, cost.clone(network));
		}
		

	}

	class TravelTimeFunction implements Function<Double>{

		@Override
		public Double get(int linkId) {
			return lpFlowModel.get(linkId).getTravelTime(
					UWSensorNetwork.this.lpFreeFlowTravelTime.get(linkId),
					UWSensorNetwork.this.lpCapacity.get(linkId), 
					UWSensorNetwork.this.lpFlow.get(linkId),
					UWSensorNetwork.this.lpDistance.get(linkId), linkId);
		}

		@Override
		public Function<Double> clone(Network network) {
			UWSensorNetwork uWSNetwork = (UWSensorNetwork) network;
			return uWSNetwork.new TravelTimeFunction();
		}
		
	}
	
	class TravelTimeIntegralFunction implements Function<Double>{
		
		@Override
		public Double get(int linkId) {			
			return lpFlowModel.get(linkId).getTravelTimeIntegral(lpFreeFlowTravelTime.get(linkId),
					lpCapacity.get(linkId), lpFlow.get(linkId), lpDistance.get(linkId), linkId);
		}

		@Override
		public Function<Double> clone(Network network) {
			UWSensorNetwork uWSNetwork = (UWSensorNetwork) network;
			return uWSNetwork.new TravelTimeIntegralFunction();
		}
	}

	class TravelCostFunction implements Function<Double>{

		@Override
		public Double get(int linkId) {
//			return costModel.getCost(pTravelTime.get(linkId),
//					lpDistance.get(linkId));
			return costModel.getCost(linkId);
		}

		@Override
		public Function<Double> clone(Network network) {
			UWSensorNetwork uWSNetwork = (UWSensorNetwork) network;
			return uWSNetwork.new TravelCostFunction();
		}
	}

	class TravelCostIntegralFunction implements Function<Double>{
		
		@Override
		public Double get(int linkId) {
			return costModel.getCostIntegral(pTravelTime.get(linkId),
					lpDistance.get(linkId), lpFlow.get(linkId));
		}

		@Override
		public Function<Double> clone(Network network) {
			UWSensorNetwork uWSNetwork = (UWSensorNetwork) network;
			return uWSNetwork.new TravelCostIntegralFunction();
		}
	}
	
	class ResidualCapacityFunction implements Function<Double>{		
		@Override
		public Double get(int linkId) {
			return lpCapacity.get(linkId).doubleValue() - lpFlow.get(linkId).doubleValue();
		}

		@Override
		public Function<Double> clone(Network network) {
			UWSensorNetwork uWSNetwork = (UWSensorNetwork) network;
			return uWSNetwork.new ResidualCapacityFunction();
		}		
	}

}



