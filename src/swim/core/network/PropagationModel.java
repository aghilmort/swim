/**
 * 
 */
package swim.core.network;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */

/**
Defines the signal power loss as a function of travel distane
*/
public abstract class PropagationModel {

	public abstract double getAttenuation(double distance, double frequency);
}
