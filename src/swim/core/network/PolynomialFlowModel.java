package swim.core.network;

public class PolynomialFlowModel implements FlowModel{
	double B;		//increase in travel time when link reaches capacity
	double power;	//rate of increase in travel time beyond capacity
	UWSensorNetwork network;
	
	public PolynomialFlowModel(UWSensorNetwork network, double B, double power){
		this.B = B;
		this.power = power;
		this.network = network;
	}

	@Override
	public double getTravelTime(double freeFlowTime,
			double capacity, double actualFlow, double distance, int linkId){
		return innerGetTravelTime(freeFlowTime, capacity, actualFlow, distance, linkId);
	}
	
	@Override
	public double getTravelTimeIntegral(double freeFlowTime, double capacity, 
			double actualFlow, double distance, int srcNode){
		return innerGetTravelTimeIntegral(freeFlowTime, capacity, actualFlow);
	}
	
	@Override
	public FlowModel clone() {		
		return new PolynomialFlowModel(this.network, this.B, this.power);
	}

	public class WithCutoff implements FlowModel{

		@Override
		public double getTravelTimeIntegral(double freeFlowTime, double capacity,
				double actualFlow, double distance, int srcNode) {
			if(actualFlow<capacity)
				return innerGetTravelTimeIntegral(freeFlowTime, capacity, actualFlow);
			else
				return Double.POSITIVE_INFINITY;
		}
		
		@Override
		/*
		 * MUST CHANGE
		 */
		public double getTravelTime(double freeFlowTime, double capacity, 
				double actualFlow, double distance, int srcNode){
			//if(actualFlow<capacity)
				return innerGetTravelTime(freeFlowTime, capacity, 
						actualFlow, distance, srcNode); 
			//else 
				//return Double.POSITIVE_INFINITY;
		}

		@Override
		public FlowModel clone() {
			return new PolynomialFlowModel(network, B, power).new WithCutoff();
		}
	}
	
	/*
	 * CHECK VALIDITY
	 */
	private double innerGetTravelTimeIntegral(double freeFlowTime, double capacity,
			double actualFlow){
		return freeFlowTime*actualFlow*(1+B*Math.pow(actualFlow/capacity, power)/(power+1));
	}
	
	private double innerGetTravelTime(double freeFlowTime,
			double capacity, double actualFlow, double distance, int linkId) {
		//return freeFlowTime*(1+B*Math.pow(actualFlow/capacity, power));
		Boolean dataProcessor = (network.npNodeType.get(network.sourceNode(linkId)) == 3);
		double processingTime = 0.0;
		if(dataProcessor){
			processingTime = UWSensorNetwork.PROCESSING_TIME;
		}
		
		return (actualFlow/UWSensorNetwork.PACKET_LENGTH)*((UWSensorNetwork.PACKET_LENGTH/capacity)+freeFlowTime)+processingTime;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String.format("(%.1f,%.1f)", B, power);
	}
}
	
