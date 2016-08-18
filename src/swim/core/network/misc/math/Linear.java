/**
 * 
 */
package swim.core.network.misc.math;

import swim.core.network.misc.math.BaseFunction;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Linear implements BaseFunction {
	double base;
	double increment;
	
	public Linear(double base, double increment){
		this.base = base;
		this.increment = increment;
	}

	public double evaluate(int i){
		return base+i*increment;
	}
}
