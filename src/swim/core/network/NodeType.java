/**
 * 
 */
package swim.core.network;

import java.util.Vector;

import swim.core.network.PowerModel.PowerLevel;

/**
 * @author Saleh Ibrahim, Sherif Tolba
 *
 */
public class NodeType {
	
	Vector<PowerLevel> levels;
	MobilityModel mobilityModel;

	double deploymentCost;
	double transportEnergyRate;
	Boolean isSink;
	Boolean isRegular;
	Boolean isProcessing;
	double idleListeningPower;
	double receivingPower;
	double initialEnergy;
	
	public NodeType(){
		deploymentCost = 0;
		transportEnergyRate =0;
		isSink = false;
		isRegular = true;
		isProcessing = false;
		idleListeningPower = 0;
		initialEnergy = 0;
		receivingPower = 0;
		levels = new Vector<PowerLevel>();
	}
		
	//////////////////////////
	//Power Levels Management
	//////////////////////////

	/** Add acoustic power level
	 *  Each node type is capable of a set of acoustic communication at a set of preset power levels
	 *  Each power level has an associated reliable communication range and an interference range
	 */
	public void addPowerLevel(PowerLevel powerLevel){
		levels.add(powerLevel);
	}

	/** Set the node deployment cost, i.e. price plus initial deployment cost
	 */
	public void setDeploymentCost(double dollars){
		deploymentCost = dollars;
	}
	public void setDeploymentCost(){
		deploymentCost = 0.0;
	}

	/** Set the idle listening power
	 *  The idle-listening power is the average power consumption when a node is listening or sleeping.
	 */
	public void setIdleListeningPower(double watts){
		idleListeningPower = watts;
	}
	public void setIdleListeningPower(){
		idleListeningPower = 0.0;
	}
	
	public void setReceivingPower(double watts){
		receivingPower = watts;
	}
	public void setReceivingPower(){
		receivingPower = 0.0;
	}

	/** Set the initial energy 
	 * The initial energy is the initial residual energy of nodes
	 */
	public void setInitialEnergy(double wattSeconds){
		initialEnergy = wattSeconds;
	}
	public void setInitialEnergy(){
		initialEnergy = 1e10;
	}

	/* Set/reset the sink flag.
	 * When set, the node doesn't forward any traffic over acoustic links
	 */
	public void setIsSink(Boolean isSink){
		if(isSink){
			this.isSink = true;
			this.isRegular = false;
		}else{
			this.isRegular = true;
			this.isSink = false;
		}	
		this.isProcessing = false;
	}
	public void setIsSink(){
		this.isSink = true;
		this.isRegular = false;
		this.isProcessing = false;
	}
	
	// If not a regular node, default to a sink
	public void setIsRegular(Boolean isRegular){
		if(isRegular){
			this.isRegular = true;
			this.isSink = false;
		}else{
			this.isRegular = false;
			this.isSink = true;
		}
		this.isProcessing = false;
	}
	public void setIsRegular(){
		this.isSink = false;
		this.isRegular = true;
		this.isProcessing = false;
	}
	
	// If not processing, default to a regular node
	public void setIsProcessing(Boolean isProcessing){
		if(isProcessing){
			this.isProcessing = true;
			this.isRegular = false;
		}else{
			this.isRegular = true;
			this.isProcessing = false;
		}
		this.isSink = false;
	}
	public void setIsProcessing(){
		this.isSink = false;
		this.isRegular = false;
		this.isProcessing = true;
	}
	
	/** Set the energy cost of moving the gateway node
	 *  (Optional) If the node is actively mobile, this method sets the energy cost of moving the gateway node in watt.s/m.
	 */
	public void setTransportEnergyRate(double wattSecondPerMeter){
		transportEnergyRate = wattSecondPerMeter;
	}
	public void setTransportEnergyRate(){
		transportEnergyRate = 0;
	}

	public PowerLevel getPowerLevel(int index){
		return levels.get(index);
	}
	
	public Vector<PowerLevel> getPowerLevels(){
		return levels;
	}

	public int getPowerLevelCount(){
		return levels.size();
	}

	public double getTransmissionPower(int index){
		return levels.get(index).getTransmissionPower();
	}

	public double getIdleListeningPower(){
		return idleListeningPower;
	}

	public double getReceivingPower(){
		return receivingPower;
	}
	
	public double getDeploymentCost(){
		return deploymentCost;
	}
	
	public double getTransportEnergyRate(){
		return transportEnergyRate;
	}

	public double getCommunicationRange(int index){
		return levels.get(index).getCommunicationRange();
	}

	public double getInterferenceRange(int index){
		return levels.get(index).getInterferenceRange();
	}

	public double getInitialEnergy(){
		return initialEnergy;
	}

	public void setMobilityModel(MobilityModel mobilityModel){
		this.mobilityModel = mobilityModel;
	}

	public static NodeType create(){
		return new NodeType();
	}
	
	public int get(){
		return (isSink ? 1 : (isRegular ? 2 : 3));
	}

}
