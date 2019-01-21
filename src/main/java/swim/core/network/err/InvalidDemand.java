package swim.core.network.err;

import swim.core.network.ODPair;

public class InvalidDemand extends Exception{

	/**
	 * 
	 */
	private static final long serialVersionUID = 1430546495460152124L;


	private ODPair od;
	private double volume;
	
	public InvalidDemand(ODPair od, double volume){
		this.od = od;
		this.volume = volume;
	}
	
	private static final String errorMessage = "Invalid demand!\nVolume must be zero if Origin=Destination and must be greater than 0 otherwise.\nOrigin-Destination = %s, volume=%f\n";
	@Override
	public String getMessage() {
		return String.format(errorMessage, od.toString(), volume);
	}

	private static final String objectID = "InvalidDemand Exception(%s:%f)";
	@Override
	public String toString() {
		return String.format(objectID, od.toString(), volume);
	}		
	
}
