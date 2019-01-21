package swim.core.network;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

public class MultipathRoute extends Route {

	final ODPair od;
	Map<Integer, Double> linkProbabilities;

	public MultipathRoute(double coef1, Route route1, double coef2, Route route2) {
		super(route1.network);
		
		this.od = route1.getODPair();		
		linkProbabilities = new HashMap<Integer, Double>(
				route1.getLinkCount()+route2.getLinkCount());

		addComponent(coef1, route1);
		addComponent(coef2, route2);
		
	}

	public MultipathRoute(Network network, ODPair od) {
		super(network);
		this.od = od;		
		linkProbabilities = new HashMap<Integer, Double>(10);
	}

	public MultipathRoute(Route route) {
		super(route.network);
		this.od = route.getODPair();
		linkProbabilities = new HashMap<Integer, Double>(10);
		addComponent(1.0, route);
	}

	public void addComponent(double coef, Route route) {
		assert(od.equals(route.getODPair()));
		assert(network == route.network);
		
		for(int l:route.getLinks()){
			double newProbability = getLinkProbability(l)+
						coef*route.getLinkProbability(l);
			
			if(Math.abs(newProbability)<1e-6)
				linkProbabilities.remove(l);
			else
				linkProbabilities.put(l, newProbability);
		}		
	}

	@Override
	public Route combineWith(Route other, double thisRatio) {
		return new MultipathRoute(thisRatio, this, 1-thisRatio, other);
	}

	@Override
	public int getDestination() {
		od.getDestination(); //==route2.getDestination
		return 0;
	}

	@Override
	public int getLinkCount() {
		return linkProbabilities.size();
	}

	@Override
	public double getLinkProbability(int linkId) {
		Double result =  linkProbabilities.get(linkId);
		if(result==null)
			return 0.0;
		else
			return result;
	}

	@Override
	public Iterable<Integer> getLinks() {
		return linkProbabilities.keySet();
	}

	@Override
	public ODPair getODPair() {
		return od;
	}

	@Override
	public int getOrigin() {		
		return od.getOrigin();
	}

	@Override
	public boolean containsLink(Integer linkId) {
		return linkProbabilities.containsKey(linkId);
	}

	
	public void put(int linkId, double probability){
		linkProbabilities.put(linkId, probability);
	}

	public void addLink(int e, double linkProbability) {
		put(e, getLinkProbability(e)+linkProbability);		
	}
	
}
