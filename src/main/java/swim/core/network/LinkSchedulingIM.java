/**
 * 
 */
package swim.core.network;

import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class LinkSchedulingIM extends InterferenceModel {

	private Vector<CollisionSet> collisionSets;
	private int sechduleLength;
	private double slotTime;
	
	public LinkSchedulingIM(int scheduleLength, double slotTime){
		this.slotTime = slotTime;
		this.sechduleLength = scheduleLength;
	}
	
	// Return zero if transmissions are not synchronized
	public double getSlotTime(){
		return 0;
	}
	
	// Return the number of slots in one link-scheduling cycle
	public int getSlotCount(){
		return this.sechduleLength;
	}
	
	// Given the set of all links, calculate a list of collision sets
	public void calculateCollisionSets(Links linkSet){};
	
	public int getCollisionSetCount(){
		return this.collisionSets.size();
	}
	
	public CollisionSet getCollisionSet(int index){
		return this.collisionSets.get(index);
	}
	
	
}
