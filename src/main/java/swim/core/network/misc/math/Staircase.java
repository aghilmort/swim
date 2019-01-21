/**
 * 
 */
package swim.core.network.misc.math;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Staircase extends Linear {
	int stepWidth;
	
	public Staircase(double base, double increment, int stepWidth){
		super(base, increment);
		this.stepWidth = stepWidth;
	}
	
	public double evaluate(int i)
	{
		return super.evaluate(i / stepWidth);
	}
}
