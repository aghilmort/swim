/**
 * 
 */
package swim.core.network.misc.math;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class DiscreteFunction {
	
	private DiscreteFunction(){
		
	}
	
	public static Function uniformRandom(double min, double range, int seed){
		return new Function(new UniformRandom(min, range, seed));
	}
	
	public static Function constant(double value){
		return new Function(new Constant(value));
	}
	
	public static Function linear(double base, double increment){
		return new Function(new Linear(base, increment));
	}
	
	public static Function sawtooth(double base, double increment, int period){
		return new Function(new Sawtooth(base, increment, period));
	}
	
	public static Function staircase(double base, double increment, int stepWidth){
		return new Function(new Staircase(base,increment, stepWidth));
	}
}
