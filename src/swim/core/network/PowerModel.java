/**
 * 
 */
package swim.core.network;

import swim.core.network.misc.Property;
import swim.core.network.misc.PropertyMap;
import swim.core.network.misc.DoublePropertyMap;

import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class PowerModel {

	Property<Vector<PowerLevel>> powerLevelsMap;
	
	Property<Double> transportEnergyRateMap;
	Property<Double> idleListeningPowerMap;
	Property<Double> receivingPowerMap;
	Property<Double> initialEnergyMap;
	
//	double transportEnergyRate = 0.0;
//	double idleListeningPower = 0.0;
//	double receivingPower = 0.0;
//	double initialEnergy = 0.0;
	
	/**
	 * Constructor
	 */
	public PowerModel(Property<Vector<PowerLevel>> powerLevelsMap, 
			Property<Double> idleListeningPowerMap,
			Property<Double> receivingPower,
			Property<Double> initialEnergy, 
			Property<Double> transportEnergyRateMap){
		this.powerLevelsMap = powerLevelsMap;
		this.idleListeningPowerMap = idleListeningPowerMap;
		this.receivingPowerMap = receivingPower;
		this.initialEnergyMap = initialEnergy;
		this.transportEnergyRateMap = transportEnergyRateMap;
	}
	
	/**
	 * Copy constructor 
	 */
	public PowerModel(PowerModel otherMobilityModel){
		this.powerLevelsMap = otherMobilityModel.powerLevelsMap;
		this.idleListeningPowerMap = otherMobilityModel.idleListeningPowerMap;
		this.receivingPowerMap = otherMobilityModel.receivingPowerMap;
		this.initialEnergyMap = otherMobilityModel.initialEnergyMap;
		this.transportEnergyRateMap = otherMobilityModel.transportEnergyRateMap;
	}
	
	public static class PowerLevel{
		
		double transmissionPower;
		double communicationRange;
		double interferenceRange;
		
		public PowerLevel(double powerInWatts, double communicationRangeInMeters, double interferenceRangeInMeters){
			this.transmissionPower = powerInWatts;
			this.communicationRange = communicationRangeInMeters;
			this.interferenceRange = interferenceRangeInMeters;
		}

		public PowerLevel(double powerInWatts, double communicationRangeInMeters){
			this.transmissionPower = powerInWatts;
			this.communicationRange = communicationRangeInMeters;
			this.interferenceRange = 0.0;
		}		
		
		public double getTransmissionPower(){
			return this.transmissionPower;
		}
		
		public double getCommunicationRange(){
			return this.communicationRange;
		}
		
		public double getInterferenceRange(){
			return this.interferenceRange;
		}
		
		public static PowerLevel create(double transmissionPowerInWatts, double communicationRangeInMeters, double interferenceRangeInMeters){
			return new PowerModel.PowerLevel(transmissionPowerInWatts, 
					communicationRangeInMeters, interferenceRangeInMeters);
		}
	}
	
	/** Add acoustic power level
	 *  Each node type is capable of a set of acoustic communication at a set of preset power levels
	 *  Each power level has an associated reliable communication range and an interference range
	 */
	public void addPowerLevel(int nodeId, PowerLevel aPowerLevel){
		this.powerLevelsMap.get(nodeId).add(aPowerLevel);
	}
	
	public int getPowerLevelCount(int nodeId){
		return this.powerLevelsMap.get(nodeId).size();
	}
	
	public PowerLevel getPowerLevel(int nodeId, int index){
		return this.powerLevelsMap.get(nodeId).get(index);
	}
	
	/** Set the idle listening power
	 *  The idle-listening power is the average power consumption when a node is listening or sleeping.
	 */
	public void setIdleListeningPower(int nodeId, double watts){
		this.idleListeningPowerMap.set(nodeId, watts);
	}
	
	public void setReceivingPower(int nodeId, double watts){
		this.receivingPowerMap.set(nodeId, watts);
	}
	
	/** Set the initial energy 
	 * The initial energy is the initial residual energy of nodes
	 */
	public void setInitialEnergy(int nodeId, double wattSeconds){
		this.initialEnergyMap.set(nodeId, wattSeconds);
	}
	
	/** Set the energy cost of moving the gateway node
	 *  (Optional) If the node is actively mobile, this method sets the energy cost of moving the gateway node in watt.s/m.
	 */
	public void setTransportEnergyRate(int nodeId, double wattsSecondPerMeter){
		this.transportEnergyRateMap.set(nodeId, wattsSecondPerMeter);
	}
	
	public double getTransmissionPower(int nodeId, int index){
		return this.powerLevelsMap.get(nodeId).get(index).getTransmissionPower();
	}
	
	public double getIdleListeningPower(int nodeId){
		return this.idleListeningPowerMap.get(nodeId);
	}
	
	public double getReceivingPower(int nodeId){
		return this.receivingPowerMap.get(nodeId);
	}
	
	public double getCommunicationRange(int nodeId, int index){
		return this.powerLevelsMap.get(nodeId).get(index).getCommunicationRange();
	}
	
	public double getInterferenceRange(int nodeId, int index){
		return this.powerLevelsMap.get(nodeId).get(index).getInterferenceRange();
	}
	
	public double getInitialEnergy(int nodeId){
		return this.initialEnergyMap.get(nodeId);
	}
}


