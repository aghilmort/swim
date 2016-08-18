/**
 * 
 */
package swim.core.network;

import swim.core.network.misc.Clock;
import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.misc.Property;
import swim.core.network.misc.PropertyMap;

import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class MobilityModel {
	
	private Property<Double> npTime;
	private Property<Vector<Double>> npCoordinates;
	
	Clock clock;
	
	public MobilityModel(Property<Double> npTime, Property<Vector<Double>> npCoordinates){
		this.npTime = npTime;
		this.npCoordinates = npCoordinates;
	}
	
	public MobilityModel(MobilityModel mobilityModel) {	
		this.npTime = mobilityModel.npTime;
		this.npCoordinates = mobilityModel.npCoordinates;
	}
	
	class Clock extends swim.core.network.misc.Clock {
	
		double time;
		
		public Clock(double time){
			this.time = time;
		}
		
		/**
		 * @override
		 */
		public double getTime(){
			return this.time;
		}
		
		public void setTime(double t){
			this.time = t;
		}
		
	}
	
	public void setClock(final Clock clock){
		this.clock = clock;
	}
	
	public void move(int nodeId){
		double nodeTime = this.npTime.get(nodeId);
		double dt = this.clock.advanceTime(nodeTime);
		double x, y, z;
		Vector<Double> coords = npCoordinates.get(nodeId);
		x = coords.get(0).doubleValue();
		y = coords.get(0).doubleValue();
		z = coords.get(0).doubleValue();
	}

	/**
	 * @return node's time
	 */
	public Double getNodeTime(int nodeId) {
		return npTime.get(nodeId);
	}

	/**
	 * @param npTime node's time
	 */
	public void setNodeTime(int nodeId, Double npTime) {
		this.npTime.set(nodeId, npTime);
	}

	/**
	 * @return node's coordinates
	 */
	public Vector<Double> getNodeCoordinates(int nodeId) {
		return npCoordinates.get(nodeId);
	}

	/**
	 * @param npCoordinates node's coordinates
	 */
	public void setNodeCoordinates(int nodeId, Vector<Double> npCoordinates) {
		this.npCoordinates.set(nodeId, npCoordinates);
	}
}
