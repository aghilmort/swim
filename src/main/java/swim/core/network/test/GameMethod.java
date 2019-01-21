package swim.core.network.test;

import java.util.List;
import java.util.Map;
import org.apache.commons.collections15.Transformer;

//import edu.uci.ics.jung.algorithms.shortestpath.DijkstraShortestPath;

import swim.core.network.FlowModel;
import swim.core.network.Network;
import swim.core.network.UWSensorNetwork;
import swim.core.network.ODPair;
import swim.core.network.ODPairAssignment;
import swim.core.network.PolynomialFlowModel;
import swim.core.network.Assignment;
import swim.core.network.Demand;
import swim.core.network.Route;
import swim.core.network.Path;
import swim.core.network.algorithm.ShortestPathRouter;
import swim.core.network.err.InvalidPath;
import swim.core.network.io.NetworkReader;
import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.misc.ReadOnlyProperty;
import swim.core.network.misc.PropertyMap;
import swim.core.network.vulnerability.*;

public class GameMethod {
	private UWSensorNetwork uWSNetwork;//<R>instance of the traffic network class
	private Demand demand;//<R>link demands on each link
	
	/**
	 * Initializes the network.<br />
	 * Displays the {@link UWSensorNetwork}.<br />
	 * Invokes the {@link run} method.
	 */
	public void test(){ 
		init();
		showNetwork();
		new Game().run();
		
	}
	
	/**
	 * Declares and instantiates {@link TestNetwork}.<br />
	 * Populates {@link UWSensorNetwork} by invoking {@link TestNetwork.getNetwork} method.<br />
	 * Assigns demand to the edges in the {@link UWSensorNetwork} by {@link TestNetwork.getDemand}.
	 */
	private void init(){
    	
		TestNetwork testNetwork = new TestNetwork(TestNetwork.NetworkId.Sample4nodes6links);
		uWSNetwork = testNetwork.getNetwork();
		demand = testNetwork.getDemand();
		
	}
	/**
	 * Prints out the UWSensorNetwork nodes and edges with demand and free-flow time.
	 */
	private void showNetwork(){
		System.out.print(uWSNetwork.toString("links", new String[]{"FFTTime"}));
		System.out.print(demand);
		System.out.print("\n\n");
		
	}
	
	/**
	 * Inner class Game that runs the Game Theory algorithm to minimize vulnerability.
	 */
	class Game{
		public class CostProperty extends ReadOnlyProperty<Double> {

			public CostProperty() {
				super("link", "S-Cost", GameMethod.this.uWSNetwork, 0.0);
			}

			@Override
			public Double get(int linkId) {				
				return Game.this.S.get(linkId);
			}

			@Override
			public ReadOnlyProperty<Double> clone(Network network) {
				return this;
			} 
		}
		
		public class CostTransformer implements Transformer<Integer, Number> {

			@Override
			public Double transform(Integer linkId) {				
				return Game.this.S.get(linkId);
			} 
		}
		
		
		ReadOnlyProperty<Double> costProperty;
		CostTransformer costTransformer;
		//DijkstraShortestPath<Integer, Integer> algorithm;
		ShortestPathRouter router;
		List<Integer> shortestPath;
		
		/**
		 * rho = probability the tester disables edge e at iteration n.<br />
		 * gamma = probability the router chooses edge e at iteration n.<br />
		 * S = s-expected cost of edge e at iteration n.<br />
		 * CF = cost of edge e in failure scenario F.<br />
		 * x = edge use probability differential of edge e at iteration n.<br />
		 * rho_n_1 = 
		 */
		DoublePropertyMap rho, gamma, S, CF, x, rho_n_1;
		
		//Assignment h;
		
		/**
		 * failed edge penalty weight
		 */
		final double beta=10;
		/**
		 * Aggressiveness of the tester
		 */
		final double theta=.5;
		/**
		 * sufficiently small convergence criterion
		 */
		final double epsilon = 1e-5;
		/**
		 * vulnerability of the present iteration
		 */
		double V;
		/**
		 * vulnerability of the previous iteration
		 */
		double Vn_1;
		/**
		 * iteration counter
		 */
		int n;
		/**
		 * sum of all demand
		 */
		private double totalDemandVolume;
		
		/**Initialize Game object to process the network and demand in the containing class.
		 * 
		 */
		public Game(){
			totalDemandVolume = demand.getTotal();			
		}
		
		/**
		 * Initializes variables for the Game Theory algorithm.
		 */
		private void GameInit(){
			costTransformer = new CostTransformer();
			costProperty = new CostProperty();
			rho = new DoublePropertyMap("link", "rho", uWSNetwork);
			gamma = new DoublePropertyMap("link", "gamma", uWSNetwork);
			S = new DoublePropertyMap("link", "S", uWSNetwork);
			CF = new DoublePropertyMap("link", "CF", uWSNetwork);
			//h = new Assignment(network);
			
			double initRho = 1.0/uWSNetwork.getLinkCount();
			for(int link: uWSNetwork.getLinks()){
				rho.set(link, initRho);
				gamma.set(link, 0.0);
			}
			
			V = 0;
			Vn_1 = 1e5;
			n = 1;
		}
		
		/**
		 * Calculate s-expected edge costs.
		 */
		private void computeMeanEdgeCosts(){
			for(int link: uWSNetwork.getLinks()){
				double C_ = uWSNetwork.getFreeFlowTravelTime(link);
				S.set(link, C_ * (1-rho.get(link)) + beta * C_ * rho.get(link));
			}
		}
		
		/**
		 * Identify shortest path and travel demand between O-D pair.
		 */
		private void calculateShortestPath(){			
			router = new ShortestPathRouter(uWSNetwork, new CostProperty(), null);
			
			Assignment newA = router.route(demand);
			x = newA.getFlow();
			//h.combine(newA, (1-1.0/n)); //combine using MSA
		}
		
		/**
		 * Update edge use probability.
		 */
		private void updateRouteProbability(){
			for(int link: uWSNetwork.getLinks()){
				gamma.set(link, (1.0/(1*n)) * (x.get(link)/totalDemandVolume) + (1-1.0/(1*n))*gamma.get(link));					
			
			}
		}
		
		/**
		 * Update edge costs.
		 */
		private void updateEdgeCosts(){
			for(int link : uWSNetwork.getLinks()){
				/**
				 * C_ is the cost of edge e in a normal state
				 */
				double C_ = uWSNetwork.getFreeFlowTravelTime(link);
				if(rho.get(link)>0){
					CF.set(link, beta*C_);
				}
				else{
					CF.set(link, C_);
				}
			}
		}
		
		/**
		 * Update the probability of which nodes the tester will fail.
		 */
		private void updateFailureProbability(){
			DoublePropertyMap qF = 
					new DoublePropertyMap("link", "gF", uWSNetwork);
				
			double Q=0;
	
			for(int F:uWSNetwork.getLinks()){
				//old entropy function approach
				double q = Math.exp(theta* gamma.get(F)*CF.get(F)-1);
				//double q = gamma.get(F)*(CF.get(F));
				qF.set(F, q); 
				Q+=q;
			}

			rho_n_1 = rho.clone();

			for(int link: uWSNetwork.getLinks()){				
				rho.set(link, qF.get(link)/Q);
			}
		}
		/**
		 * The difference in vulnerability in two successive iterations.
		 */
		private double error;
		
		/**
		 * The objective function.
		 */
		private void updateVulnerability(){
			Vn_1 = V;
			V = 0;
			for(int link: uWSNetwork.getLinks()){
				V = V + gamma.get(link) * rho.get(link) * CF.get(link);
			}
			
			error = 0.0;
			for(int link:uWSNetwork.getLinks()){
				//error+=Math.abs(rho.get(link)-rho_n_1.get(link));
			}	
			error = Math.abs(V - Vn_1);
		}
		
		/**
		 * Displays the vulnerability, failure probability, Router Probability, and S-Expected Link Cost for the iteration.
		 */
		private void showIteration(){
			System.out.printf("Iteration #%d\nVulnerability = %f\nFailure Probability = \n%s\nRouter Probability = \n%s\n S-Expected Link Cost= \n%s\n",
					n, V, rho.toString(), gamma.toString(), S.toString());
		}
		
		/**
		 * Displays the last iteration and the runtime in milliseconds.
		 */
		private void showResult(){
			showIteration();
			System.out.printf("Runtime =%d ms\n", runtime);
			
		}
		
		/**
		 * The runtime of the entire algorithm in milliseconds.
		 */
		long runtime;
		
		/**
		 * Runs the initialization method {@link Game.GameInit}.<br />
		 * Executes the main Game Theory algorithm loop.<br />
		 * Displays the result including the runtime using the {@link Game.showResult} method.
		 */
		public void run(){
			runtime = System.currentTimeMillis();

			GameInit();
			do{
				computeMeanEdgeCosts();//Eq 6
				showIteration();
				calculateShortestPath();//Eq 7 (part A)
				updateRouteProbability();//Eq 8
				updateEdgeCosts();//Eq 5
				updateFailureProbability();//Eq 10
				updateVulnerability();//Eq 9 (part A)
				n = n + 1; 
			}while (error>epsilon);
			
			runtime = System.currentTimeMillis()-runtime;
			
			showResult();
		}		
	}
	
	/**
	 * Main method; call this method to run the Game Theory algorithm
	 * @param args
	 */
    public static void main(String[] args) {
    	GameMethod networkTest = new GameMethod();
    	networkTest.test();
    }	
}
