/**
 * 
 */
package swim.core.network;

import swim.core.network.Network.Links;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public abstract class InterferenceModel {
	
	protected static Network network;
	
	public static abstract class InterferingLink {
		public abstract int getLink();
		public abstract int getTimeSlotDifference();
	}
	
	public static abstract class CollisionSet {
		public abstract void addInterferingLink(int linkId, int timeSlotDifference);
		public abstract int getInterferingLinkCount();
		public abstract InterferingLink getInterferingLink(int index);
	}
	
	public abstract class Links implements Iterable<Integer> {	
		
		/**
		 * Get the number of fixed links
		 * @return
		 */
		public abstract int getCount();
		
		/**
		 * Get link by index
		 * @param index
		 * @return
		 */
		public abstract int getLink(int index);
	}
	
	// Return zero if transmissions are not synchronized
	public abstract double getSlotTime();
	
	// Return the number of slots in one link-scheduling cycle
	public abstract int getSlotCount();
	
	// Given the set of all links, calculate a list of collision sets
	public abstract void calculateCollisionSets(Links linkSet);
	
	public abstract int getCollisionSetCount();
	
	public abstract CollisionSet getCollisionSet(int index);
	
	public void setNetwork(Network network){
		this.network = network;
	}
	
	public static InterferenceModel create(int scheduleLength){
		return new LinkSchedulingIM(scheduleLength, 0);
		//return new HalfDuplexIM();
	}
	
}
