/**
 * 
 */
package swim.core.network.deployment;

import java.util.Vector;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class SolutionVector {
	
	Vector<Solution> items;
	
	public SolutionVector(int maxGateways){
		items = new Vector<Solution>(maxGateways);
		for(int i = 0; i < maxGateways; i++){
			items.add(new Solution(i+1));
		}
	}
	
	public Solution get(int nGateways){		// implemented in place of the overloaded operator
		return items.get(nGateways - 1);
	}
	
	public Solution set(int nGateways, Solution updatedSol){		// newly added
		return items.set(nGateways - 1, updatedSol);
	}
	
	public Vector<Solution> items(){
		return items;
	}
	
	public void plusEquals(SolutionVector sv){
		for(Solution s : items){
			if(s.gatewayCount > sv.items.size()){
				return;
			}
			
			if(sv.get(s.gatewayCount).isFeasible()){
				s.objectiveValue.plusEquals(sv.get(s.gatewayCount).objectiveValue);
				s.runtime.plusEquals(sv.get(s.gatewayCount).runtime);
			}
		}
	}
	
	
}
