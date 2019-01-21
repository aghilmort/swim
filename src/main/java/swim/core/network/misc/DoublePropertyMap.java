package swim.core.network.misc;

import java.util.Map;
import java.util.Set;

import swim.core.network.Network;
import swim.core.network.misc.PropertyMap;

public class DoublePropertyMap extends PropertyMap<java.lang.Double>{

	public DoublePropertyMap(PropertyMap<Double> other) {
		super(other);
	}

	public DoublePropertyMap(ReadOnlyProperty<Double> other) {
		super(other);
	}

	public DoublePropertyMap(String graphComponentType, String name, Network network, Double defaultValue) {
		super(graphComponentType, name, network, defaultValue);
	}

	public DoublePropertyMap(String graphComponentType, Network network, ReadOnlyProperty<Double> other) {
		super(graphComponentType, network, other);
	}

	public DoublePropertyMap(String graphComponentType, Network network, PropertyMap<Double> other) {
		super(graphComponentType, network, other);
	}

	public DoublePropertyMap(String graphComponentType, String name, Network network){
		super(graphComponentType, name, network, 0.0);
	}
	
	public DoublePropertyMap(String graphComponentType, String name, Network network, double defaultValue) {
		super(graphComponentType, name, network, defaultValue);
	}

	/**
	 * Add flow to the flow of the specified link and store the result 
	 * in the map as the current value of the property for that link 
	 * @param i link id
	 * @param flow traffic flowing in the link
	 * @return the new flow
	 */
	public java.lang.Double add(int i, double flow){
		double sum = this.get(i)+flow;
		this.set(i, sum);
		return sum;
	}

	/**
	 * Add flows of the links in the specified {@link DoublePropertyMap} to the
	 * flows of the links in this map and returns the modified map.
	 * @param flows a map of link flows to be added to flows of this map
	 * @return the modified link flow map
	 */
	public DoublePropertyMap add(DoublePropertyMap flows){
		for(int i: flows.map.keySet()){
			double sum = this.get(i)+flows.get(i);
			this.set(i, sum);
		}
		return this;
	}
	
	/**
	 * Add a specified value to a set of links in this {@link DoublePropertyMap}
	 * @param keySet set of links to add the value to
	 * @param value the value to be added
	 * @return this {@link DoublePropertyMap} after adding the value to the specified set of links
	 */
	public DoublePropertyMap add(Iterable<java.lang.Integer> keySet, double value){
		for(int link: keySet) 
			this.add(link, value);
		return this;
	}

	public DoublePropertyMap combine(DoublePropertyMap other, double otherFactor, double thisFactor){
		for(Map.Entry<java.lang.Integer, java.lang.Double> e: other.map.entrySet()){
			double thisFlow = get(e.getKey());
			double otherFlow = e.getValue();
			double newFlow = otherFactor*otherFlow+ thisFactor*thisFlow;
			map.put(e.getKey(), newFlow);
		}
		return this;
	}
	
	/**Generate a linear combination of two properties
	 * @param a coefficient of the first property
	 * @param A first property
	 * @param b coefficient of the second property
	 * @param B second property
	 * @return combined property
	 */
	static public DoublePropertyMap combine(double a, DoublePropertyMap A, double b, DoublePropertyMap B){

		DoublePropertyMap result = new DoublePropertyMap(A.getGraphComponentType(), A.getName(), A.getNetwork(), 0.0);
										
		for(Map.Entry<java.lang.Integer, java.lang.Double> e: A.map.entrySet()){
			Integer l = e.getKey();
			result.add(l, a*A.get(l));
		}
		
		for(Map.Entry<java.lang.Integer, java.lang.Double> e: B.map.entrySet()){
			Integer l = e.getKey();
			result.add(l, b*B.get(l));
		}
		
		return result;
	}

	/* (non-Javadoc)
	 * @see tn.misc.LinkPropertyMap#clone()
	 */
	@Override
	public DoublePropertyMap clone() {
		return new DoublePropertyMap(this);
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		StringBuilder b = new StringBuilder();

		if(this.getGraphComponentType().equals("link")){
			for(int i: network.getLinks()){
				b.append(String.format("#%d\t%10f\n", i, this.get(i)));
			}
		}else if(this.getGraphComponentType().equals("node")){
			for(int i: network.getNodes()){				
				b.append(String.format("#%d\t%10f\n", i, this.get(i)));
			}
		}
		
		return b.toString();
	}

	public String toRowString() {
		StringBuilder b = new StringBuilder();
		if(this.getGraphComponentType().equals("link")){
			for(int i: network.getLinks()){
				b.append(String.format(",%10f", this.get(i)));
			}
		}else if(this.getGraphComponentType().equals("node")){
			for(int i: network.getNodes()){
				b.append(String.format(",%10f", this.get(i)));
			}
		}
		
		return b.toString();
	}
	
	/* (non-Javadoc)
	 * @see tn.misc.LinkPropertyMap#clone(tn.Network)
	 */
	@Override
	public DoublePropertyMap clone(Network network) {
		return new DoublePropertyMap(this.getGraphComponentType(), network, this);
	}
}