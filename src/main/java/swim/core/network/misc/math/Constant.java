/**
 * 
 */
package swim.core.network.misc.math;

import swim.core.network.misc.math.BaseFunction;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Constant implements BaseFunction {
	double value;
	
	public Constant(double value){
		this.value = value;
	}
	
	public double evaluate(int i){
		return value;
	}
}