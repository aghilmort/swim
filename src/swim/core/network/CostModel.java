package swim.core.network;

public class CostModel{
	
	private UWSensorNetwork network;
	private double delayFactor;
	private double powerFactor;
	
	public CostModel(UWSensorNetwork network, double delayFactor, double powerFactor){
		this.network = network;
		this.delayFactor = delayFactor;
		this.powerFactor = powerFactor;
	}

	public CostModel(CostModel costModel) {
		this.network = costModel.network;
		this.delayFactor = costModel.delayFactor;
		this.powerFactor = costModel.powerFactor;
	}
	
	public void calculateCosts(){
		double cost;
		for(int l : network.getLinks()){
			cost = getCost(l);
			network.setLinkCost(l, cost);
		}
	}

	public double getCost(int linkId) {
		
		double firstTerm = (network.getAverageChannelAccessDelay(network.getOrigin(linkId)) + UWSensorNetwork.PACKET_LENGTH/network.getCapacity(linkId)/UWSensorNetwork.ACOUSTIC_LINK_PEAK_UTILIZATION + network.getFreeFlowTravelTime(linkId));
		double secondTerm = (UWSensorNetwork.PACKET_LENGTH/network.getCapacity(linkId)*network.getPowerModel().getTransmissionPower(network.getOrigin(linkId), 0));
		
		//delay = transmission delay+propagation delay
		return delayFactor * firstTerm + powerFactor * secondTerm;
            
                // For now, use node distances as link costs
                //return network.getDistance(linkId);
	}
	
	public double getCostIntegral(double travelTimeIntegral, double distance, double flow){
		return 0.0; // temporarily commented out: travelTimeIntegral + flow  * (distanceFactor* distance);
	}
}
