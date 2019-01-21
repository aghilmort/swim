/**
 * 
 */
package swim.core.network;

import java.util.Set;

/**
 * @author Sherif
 *
 */
public class ProcessingModel {
	
	private double dataGenerationRate;
	private UWSensorNetwork network;
	//private double compressionRatio;
	
	public ProcessingModel(UWSensorNetwork network, double compressionRatio){
		this.network = network;
		//this.compressionRatio = compressionRatio;
	}
	
	/**
	 * Copy constructor 
	 */
	public ProcessingModel(ProcessingModel otherProcessingModel){
		this.network = otherProcessingModel.network;
		//this.compressionRatio = otherProcessingModel.compressionRatio;
	}
	
	public void setDataGenerationRate(double packetsPerSecond){
		this.dataGenerationRate = packetsPerSecond;
	}
	
	
	public double processData(int node, double demandValue, double previousDemand){
		
		double processedDemand = 0.0;
		double newDemand = 0.0;
		Boolean dataProcessor;
		
		dataProcessor = (network.getNodeType(node) == 3);
		
		if(dataProcessor){
			processedDemand = network.getNodeCompressionRatio(node) * demandValue;
		}
		
		if(dataProcessor){
			newDemand = processedDemand + previousDemand;
			//previousDemand += processedDemand;
		}else{
			newDemand = demandValue + previousDemand;
		}

		return newDemand;
	}
	
}
