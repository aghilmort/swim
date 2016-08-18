/**
 * 
 */
package swim.core.network.misc.math;

import swim.core.network.misc.math.BaseFunction;

/**
 * @author Sherif
 *
 */
public class Function {
	BaseFunction fx;

	public Function(BaseFunction fx){
		this.fx = fx;
	}

	public double evaluate(int i){
		return fx.evaluate(i);
	}
}
