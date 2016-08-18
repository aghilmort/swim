/**
 * 
 */
package swim.core.network.deployment;

import java.util.Vector;

import swim.core.network.Network.Nodes;
import swim.core.network.stats.Statistics;
import swim.core.network.stats.Statistics.Variable;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Solution {
	
	public Vector<Integer> pickedCandidates;
	public int gatewayCount;
	public Statistics.Variable objectiveValue;
	public Statistics.Variable runtime;
	public Statistics.Variable c;
	public Statistics.Variable o;
	
	public Solution(int gatewayCount){
		this.gatewayCount = gatewayCount;
	}
	
	public Solution(int gatewayCount, double cost, long runtime){
		this.gatewayCount = gatewayCount;
		this.objectiveValue = new Variable(cost);
		this.runtime = new Variable((double)runtime);
	}
	
	public int getPickedCandidateCount(){
		return this.pickedCandidates.size();
	}
	
	public Boolean isCandidatePicked(int candidate){
		for(int node : pickedCandidates){
			if(node == candidate){
				return true;
			}
		}
		return false;
	}

	public int getPickedCandidate(int index){
		return pickedCandidates.get(index);
	}
	
	public Boolean isFeasible(){
		return this.objectiveValue.getFrequency() != 0;
	}
	
	public void setSolution(Vector<Integer> candidateNodes, int[] sol){
		for(int node : candidateNodes){
			if(sol[node - 1000] == 1){				// Important: you need to check how to make sure this part was mapped correctly
				pickedCandidates.add(node);
			}
		}
	}
}
