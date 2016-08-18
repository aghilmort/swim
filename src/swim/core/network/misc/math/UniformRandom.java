/**
 * 
 */
package swim.core.network.misc.math;

import swim.core.network.misc.math.BaseFunction;
import swim.core.network.misc.MersenneTwisterFast;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class UniformRandom implements BaseFunction {
	MersenneTwisterFast random;
	double min, range;

	public UniformRandom(double min, double range, int seed){
		this.min = min;
		this.range = range;
		random = new MersenneTwisterFast(seed);
	}

	public double evaluate(int i)
	{
		return min+(range*random.nextDouble());
	}
}
