/**
 * 
 */
package swim.core.network.misc;

/**
 * @author Sherif Tolba (manually translated and consolidated from  
 * 		   different files of a C++ code written by Saleh Ibrahim)
 *
 */
public abstract class Clock {
	
	public Clock(){
		
	}
	
	public abstract double getTime();

	public final double advanceTime(double time){
		double t = getTime();
		double dt = t - time;
		time = t;
		return dt;
	}
}
