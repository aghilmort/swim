/**
 * 
 */
package swim.core.network;

import java.util.List;
import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class HalfDuplexIM extends InterferenceModel{

	private Vector<CollisionSet> collisionSets;
	private double slotTime;

	public HalfDuplexIM(){
		this.slotTime = 0;
	}
	
	// Return zero if transmissions are not synchronized
	public double getSlotTime(){
		return 0;
	}
	
	// Return the number of slots in one link-scheduling cycle
	public int getSlotCount(){
		return 1;
	}
	
	// Given the set of all links, calculate a list of collision sets
	public void calculateCollisionSets(Links linkSet){
		
		//For each link, the collision set are those links which are directed to the source of the link in hand		
		for (int l : linkSet){
			List<Integer> interferingLinks = network.getInLinks(network.getOrigin(l));
			if(interferingLinks.size() > 0){
				CollisionSet newCS = new swim.core.network.CollisionSet(l);
				for(int i : interferingLinks){
					newCS.addInterferingLink(i, 0);
				}
				collisionSets.add(newCS);
			}
		}
		
	}
	
	public int getCollisionSetCount(){
		return this.collisionSets.size();
	}
	
	public CollisionSet getCollisionSet(int index){
		return this.collisionSets.get(index);
	}
	
}
