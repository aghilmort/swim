/**
 * 
 */
package swim.core.network;

import java.util.ListIterator;
import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class CollisionSet extends InterferenceModel.CollisionSet {

	public int transmittingLink;
	public Vector<InterferingLink> interferingLinks;
	
	public CollisionSet(int transmittingLink){
		this.transmittingLink = transmittingLink;
	}
	
	public void addInterferingLink(int linkId, int timeSlotDifference){
		interferingLinks.add(new InterferingLink(linkId, timeSlotDifference));
	}
	
	public int getInterferingLinkCount(){
		return interferingLinks.size();
	}
	
	public InterferingLink getInterferingLink(int index){
		return interferingLinks.get(index);
	}
	
	public void clear(){
		interferingLinks.clear();
	}
	
	public int size(){
		return interferingLinks.size();
	}
}
