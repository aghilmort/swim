/**
 * 
 */
package swim.core.network.misc.math;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Sawtooth extends Linear{
	int period;
	
	public Sawtooth(double base, double increment, int period){
	
		super(base, increment);
		this.period = period;
	}

	public double evaluate(int i){
		return super.evaluate(i % period);
	}

}
