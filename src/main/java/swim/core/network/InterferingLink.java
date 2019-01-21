/**
 * 
 */
package swim.core.network;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public class InterferingLink extends InterferenceModel.InterferingLink {

	private int linkId;
	private int timeSlotDifference;
	
	public InterferingLink(int linkId, int timeSlotDifference){
		this.linkId = linkId;
		this.timeSlotDifference = timeSlotDifference;
	}
	
	public int getLink(){
		return linkId;
	}
	
	public int getTimeSlotDifference(){
		return timeSlotDifference;
	}
}
