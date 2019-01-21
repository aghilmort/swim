package swim.core.network;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import swim.core.network.UWSensorNetwork;
import swim.core.network.misc.DoublePropertyMap;


/**Traffic assignment, i.e. the volume of traffic assigned to 
 * each path
 */
public class Assignment implements Iterable<ODPairAssignment>{
	
	/**
	 * The network in the context of which this assignment is defined
	 */
	UWSensorNetwork network;
	
	/**
	 * Map to store the assignment details
	 */
	private Map<ODPair, ODPairAssignment> assignment;
	
	/**Assignment Constructor
	 * 
	 */
	public Assignment(UWSensorNetwork network){
		this.network=network;
		assignment = new HashMap<ODPair, ODPairAssignment>();
	}		
	
	public Assignment(UWSensorNetwork network, int demandCount) {
		this.network = network;
		assignment = new HashMap<ODPair, ODPairAssignment>(demandCount);
	}

	/**Get the set of all (Path, volume) assignment pairs
	 * @return the set of (Path, volume) Map.Entry pairs
	 */
	@Override
	public Iterator<ODPairAssignment> iterator() {
		return assignment.values().iterator();
	}


	public void set(Map<ODPair, ODPairAssignment> result) {
		assignment.clear();
		assignment.putAll(result);			
	}


	public void add(Route route, double volume) {
		ODPair od = route.getODPair();
		ODPairAssignment oldAssignment = assignment.get(od);
		
		if(oldAssignment!=null){
			double totalVolume = oldAssignment.volume + volume;
			
			if(totalVolume < 1e-6)
				assignment.remove(route);
			else{
				assignment.put(od, new ODPairAssignment(
						route.combineWith(oldAssignment.route, volume/totalVolume), 
						totalVolume));
			}
		}else{
			assignment.put(od, new ODPairAssignment(route, volume));
		}			
	}
	
	
	public void remove(Route route, double volume) {
		add(route, -volume);
	}
	
	public void remove(Assignment other){
		for(ODPairAssignment e: other){
			remove(e.route, e.volume);
		}
	}

	/**Aggregates the assignment flows to links, i.e.
	 * and produces the total flow in each link.
	 */
	public DoublePropertyMap getFlow(){
		DoublePropertyMap flowMap = new DoublePropertyMap("link", "Assignment", network);
		
		for(ODPairAssignment e: this){
			e.route.addFlowToFlowMap(e.volume, flowMap);
		}
		return flowMap;
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return assignment.toString().replace("{", "\t")
			.replace(", ", "\n").replace("}", "\n")
			.replace("=", "\t");
	}	

	public void add(Assignment other) {
		for(ODPairAssignment e: other){
			add(e.route, e.volume);
		}
	}
	
	/**Find the difference between two different assignments of the same demand
	 * @param other
	 * @return
	 */
	static public Assignment getDifference(Demand demand, Assignment a1, Assignment a2){
		Assignment difference = new Assignment(demand.getNetwork());
		
		for(Entry<ODPair, Double> e: demand){
			ODPair od = e.getKey();
			double v = e.getValue();
			
			if(v>0){
				
				Route r1 = a1.getRoute(od);
				double v1 = a1.getVolume(od);
	
				Route r2 = a2.getRoute(od);
				double v2 = a2.getVolume(od);
				
				//make sure the two assignments have a valid mapping 
				//of the exactly the same demand (od pair, volume)
				assert(r1!=null && r2!=null && v1==v && v2==v);
									
				Route r = new MultipathRoute(v1, r1, -v2, r2);
				
				//If the two routes do not cancel each other, add the difference
				if(r.getLinkCount()>0)
					difference.add(r, v);
				
			}
		}
		
		return difference;
	}
	
	/**Combine the given assignment into this assignment such that
	 * this assignment becomes:= 
	 * a.route := [A * a.route , B * b.route)]
	 * , where 
	 * A = (ratio *a.volume)/(a.volume +b.volume),
	 * B = ((1-ratio)* b.volume),
	 * a.volume = ratio *a.volume + (1-ratio)* b.volume,
	 * a = this.assignment.get(od), b= other.assignment.get(od), for all od
	 * 
	 * @param other the other assignment to combine with this one
	 * @param thisFactor ratio for this assignment. (1-ratio) for other assignment 
	 */
	public void combine(Assignment other, double thisFactor){
		double  otherFactor = 1- thisFactor;
		
		for(Entry<ODPair, ODPairAssignment> e: other.assignment.entrySet()){
			ODPair od = e.getKey();
			ODPairAssignment 	otherODPA = e.getValue(),
								thisODPA = assignment.get(od);
			
			if(thisODPA==null)
			{
				assignment.put(od, new ODPairAssignment(otherODPA));				
			
			}else{
				Route combinedRoute = null;
				double combinedVolume=thisFactor*thisODPA.volume + otherFactor*otherODPA.volume ;
				
				if(thisODPA!=null){
					combinedRoute = thisODPA.route.combineWith(otherODPA.route,
							thisFactor*thisODPA.volume/combinedVolume);
				}
					
				if(combinedRoute!=null){
					assignment.put(
							od, new ODPairAssignment(combinedRoute, combinedVolume));
				}
				else{
					assignment.put(od, new ODPairAssignment(new MultipathRoute(thisFactor, thisODPA.route, otherFactor, otherODPA.route), combinedVolume));				
				}
			}
		}
	}
	
	public Assignment[] filterByLink(Integer linkId){
		Assignment result[] = new Assignment[2];
		result[0] = new Assignment(network);
		result[1] = new Assignment(network);
		
		for(ODPairAssignment e: assignment.values()){
			result[e.route.containsLink(linkId)?0:1]
					.add(e.route, e.volume);
		}
		return result;
	}
	
	public AssignmentFilterResult filterByLink(Iterable<Integer> links) {
		AssignmentFilterResult result = new AssignmentFilterResult();
		
		result.included = new Assignment(network);
		result.excluded = new Assignment(network);
		
		for(ODPairAssignment e: assignment.values()){
			boolean in=false;
			for(int linkId: links)
				in |= e.route.containsLink(linkId);
			if(in){
				result.included.add(e.route, e.volume);					
			}
			else{
				result.excluded.add(e.route, e.volume);
			}
		}
		return result;
	}
	
	public Demand toDemand(){
		Demand demand = new Demand(network);
		
		for(Entry<ODPair, ODPairAssignment> e: assignment.entrySet()){
			demand.add(e.getKey(), e.getValue().volume);
		}			
		return demand;
	}

	static public class AssignmentFilterResult{
		public Assignment included;
		public Assignment excluded;
	}

	public int getSize() {
		return assignment.size();
	}

	public double getTotal() {
		double total = 0.0;
		for(Entry<ODPair, ODPairAssignment>e: assignment.entrySet()){
			total+=e.getValue().volume;
		}
		return total;
	}
	
	public ODPairAssignment getODPairAssignment(ODPair od){
		return assignment.get(od);
	}

	public Route getRoute(ODPair od) {
		return assignment.get(od).route;
	}

	public double getVolume(ODPair od) {
		return assignment.get(od).volume;
	}
}

