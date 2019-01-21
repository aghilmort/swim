/**
 * 
 */
package swim.core.network.deployment;

import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Objective implements DOM.Objective{
	
	Vector<Component> components;
	
	public void addComponent(DOM.objectiveComponentId id, double weight){
		for(Component c : components){
			if(c.id == id){
				c.weight += weight;
				return;
			}
		}
		components.add(new Component(id, weight));
	}
	
	public double getComponentWeight(DOM.objectiveComponentId id){
		for(Component c : components){
			if(c.id == id){
				return c.weight;
			}
		}
		return 0.0; 
	}
	
	public void reset(){
		components.clear();
		System.gc();
	}
	
	/** A component of the objective function
	 *  The objective function is defined as a weighted sum of components.
	 */
	class Component{
		
		/** Objective function component id
		 */
		public DOM.objectiveComponentId id;
		
		/** Objective function component weight
		 */
		public double weight;
		
		/** Constructor
		 */
		public Component(DOM.objectiveComponentId id, double weight){
			this.id = id;
			this.weight = weight;
		}
	}
}
