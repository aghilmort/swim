/**
 * 
 */
package swim.core.network.simulator;

import swim.core.network.simulation.SPDelayEnergyEnum;
import swim.core.network.simulation.ScatteredElevenNTenL;
import swim.core.network.simulation.Simulation;
import swim.core.network.simulation.SimulationId;
import swim.core.network.simulation.SinglePathTwentyNodes;
import swim.core.network.simulation.SiouxFallsWNodeProps;
import swim.core.network.simulation.TwoNodeOneLink;
//import uwsn.test.GameMethod.Game.CostProperty;

/**
 * @author Sherif Tolba
 *
 */
public class Simulator {
	
	public Simulator(){
		
	}
	
	public void run(SimulationId sim){
		switch(sim){
			case simple2Nodes1LinkNetwork:
				TwoNodeOneLink twoNodeOneLink = new TwoNodeOneLink();
				twoNodeOneLink.run();
				break;
			case simple3Nodes2LinksNetwork:
				//TODO: implement this simulation
				break;
			case SinglePathNet20N19L:
				Simulation singlePathTwentyNodes = new SinglePathTwentyNodes();
				singlePathTwentyNodes.run();
				break;
			case Sample11Nodes10LinksNetwork:
				Simulation scatteredElevenNodesTenLinks = new ScatteredElevenNTenL();
				scatteredElevenNodesTenLinks.run();
				break;
			case SiouxFallsWNodeProps:
				Simulation modifiedSiouxFalls = new SiouxFallsWNodeProps();
				modifiedSiouxFalls.run();
				break;
			case SinglePathDelayEnergyEnumeration:
				Simulation spDelayEnergyEnum = new SPDelayEnergyEnum();
				spDelayEnergyEnum.run();
				break;
		}
	}
	
	public static void main(String[] args){
		Simulator sim = new Simulator();
		sim.run(SimulationId.SiouxFallsWNodeProps);
	}
	
}
