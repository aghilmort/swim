package swim.core.network;

import swim.core.network.misc.DoublePropertyMap;
import swim.core.network.misc.ReadOnlyProperty;

/**A general route from an origin node to a destination node. 
 * A route can be a single path or multipath.
 * @author Saleh
 *
 */
public abstract class Route {
	
	/**
	 * The network context in which this path exists
	 */
	protected Network network;
	
	/**
	 * @return iterator of route links
	 */
	public abstract Iterable<Integer> getLinks();
	
	/**
	 * @return origin node id
	 */
	public abstract int getOrigin();
	
	/**
	 * @return destination node id
	 */
	public abstract int getDestination();
	
	/**
	 * @return origin-destination node id
	 */
	public abstract ODPair getODPair();
	
	
	/**Get the probability of the given link. 
	 * @param linkId
	 * @return probability of routing through the given link. 
	 * Zero if the given link is not a route link 
	 */
	public abstract double getLinkProbability(int linkId);
	
	/**
	 * @return number of links in the route
	 */
	public abstract int getLinkCount();

	/**
	 * @param network the network context in which the route id defined
	 */
	public Route(Network network){
		this.network = network;
	}
	
	@Override
	public int hashCode() {
		int hash = 1;
		for(int l: getLinks()){
			hash = (hash*network.getLinkCount()+l)% 921353;
		}
		return hash;
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 * O(|E|)
	 */
	@Override
	public boolean equals(Object other){
		Route otherRoute = (Route)other;
		if(otherRoute==null)
			return false;
		if(this.getLinkCount() != otherRoute.getLinkCount())
			return false;
		for(int linkId: getLinks()){
			if(Math.abs(this.getLinkProbability(linkId)-
				otherRoute.getLinkProbability(linkId))>1e-6)
				return false;
		}
		return true;
	}
	
	/**Linearly combine the given route with this route. The resulting
	 * route satisfies the following formula:
	 * result.getLinkProbability(i) = ratio* this.getLinkProbability(i) 
	 * 							+ (1-ratio) other.getLinkProbability(i)
	 * for every link i in this.getLinks() UNION other.getLinks();
	 * 
	 * @param other the other route which is to be combined with this one 
	 * @param ratio the coefficient for this route probability. (1-ratio) is the 
	 * @return combined route if this route can be linearly combined 
	 * with the other route into one combined route. Otherwise returns
	 * null.
	 */
	public abstract Route combineWith(Route other, double ratio);


	/**Update the flow in each link of the path to reflect the
	 * given additional flow.
	 * @param flow the additional flow to be assigned
	 * @param flowMap 
	 */
	public void addFlowToFlowMap(double flow, DoublePropertyMap flowMap){
		for(int link: getLinks()) 
			flowMap.add(link, flow * getLinkProbability(link));
	}

	/**Check if the given link is part of the route
	 * @param linkId the id of the link to look for
	 * @return true if linkId is part of the route
	 */
	public abstract boolean containsLink(Integer linkId);

	/**sum of individual link costs
	 * @param linkCost Property that contains link costs
	 * @return
	 */
	public double getCost(ReadOnlyProperty<Double> linkCost){
		double travelCost = 0;
		for(int l: getLinks()){
			travelCost += getLinkProbability(l) * linkCost.get(l);
		}
		return travelCost;
	}

	@SuppressWarnings("unused")
	private double getMinPropertyValue(ReadOnlyProperty<Double> linkProperty){

		double minPropValue = Double.POSITIVE_INFINITY;
		
		for(int l:getLinks()){
			double lc = linkProperty.get(l);
			if(lc<minPropValue)
				minPropValue = lc; 
		}
		
		return minPropValue;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("{");		
		for(int l: getLinks()){
			sb.append(String.format("#%d %.3f ->", l, getLinkProbability(l)));
		}
		sb.append("}");
		return sb.toString();
	}
			
}
