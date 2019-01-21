package swim.core.network;

import java.util.ArrayList;
import java.util.List;

import swim.core.network.err.InconsistencyException;
import swim.core.network.err.InvalidPath;
import swim.core.network.misc.ReadOnlyProperty;

/*
 * A path from a source node to a destination node 
 *  
 */
public class Path extends Route{

	/*
	 * List of links from origin to destination
	 */
	private List<Integer> path;
	
	/** construct a path
	 * @param path: list of links that form the path, 
	 * starting from the origin to the destination
	 * @throws InvalidPath when any of the links 
	 * @throws InconsistencyException 
	 */
	public Path(UWSensorNetwork network, List<Integer> path) throws InvalidPath{
		super(network);
		
		int disconnection = findDisconnection(path);
		
		if(disconnection>0){
			throw new InvalidPath(path, disconnection);
		}
		this.path = new ArrayList<Integer>(path);
	}

	public Path(UWSensorNetwork network, Integer link){	
		super(network);
		
		this.path = new ArrayList<Integer>(1);
		this.path.add(link);
	}

	/**Construct a path that is a concatenation of two given paths
	 * @param pl path prefix
	 * @param pr path suffix
	 */
	public Path(UWSensorNetwork network, Path pl, Path pr) {
		super(network);

		this.path = new ArrayList<Integer>(pl.getLinkCount()+pr.getLinkCount());
		for(int l: pl.getLinks()){
			this.path.add(l);
		}
		for(int r: pr.getLinks()){
			this.path.add(r);
		}
	}
	

	
	/**Find the first link that doesn't start where its previous 
	 * link ends.
	 * @param path: a list of links that form the path
	 * @return the index of the first link that is disconnected or -1 if the path is connected
	 */
	private int findDisconnection(List<Integer> path2) {
		
		int previousLink=-1;
		
		for(int i:path2){
			if(previousLink>-1)
				if(network.getDestination(previousLink)
						!=network.getOrigin(i))
					return i;
			
			previousLink=i;
		}
		return -1;
	}
	
	/*
	 * origin node id
	 */
	@Override
	public int getOrigin(){
		return network.getOrigin(getPathLink(0));
	}
	
	/*
	 * destination node id
	 */
	@Override
	public int getDestination(){
		return network.getDestination(
				getPathLink(getLinkCount()-1));
	}
		
	
	/*
	 * Get origin-destination pair
	 */
	@Override
	public ODPair getODPair(){
		return new ODPair(getOrigin(), getDestination());
	}

	/*
	 * get number of links in path
	 * note that the number of links = number of nodes - 1
	 */
	@Override
	public int getLinkCount(){
		return path.size();
	}
	
	/**
	 * check if the path contains any links
	 * @return true if the path contains one or more links
	 * return false if path doesn't exist
	 */
	public boolean exists(){
		return path.size() > 0;
	}
	
	/*
	 * return the link at the given index of the path
	 */
	int getPathLink(int index){
		return path.get(index);
	}
	
	/*
	 * check whether a link is part of the path
	 */
	boolean containsLink(int link){
		return path.contains(link);
	}
	
	@Override
	public boolean equals(Object obj) {
		Path other = (Path)obj;
		if(other==null)
			return false;
		if(other.path.size()!=path.size())
			return false;
		for(int i=0; i<path.size(); i++){
			if(other.path.get(i)!=path.get(i))
				return false;
		}
		return true;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append(String.format("(*%d *%d)\t", getOrigin(), getDestination()));
		//sb.append(String.format("$%f\t(", getTravelCost()));
		for(int i: path) sb.append(String.format(" #%d",i));
		sb.append(")}");
		return sb.toString();
	}

	@Override
	public double getLinkProbability(int linkId) {
		if(path.contains(linkId))
			return 1.0;
		else
			return 0.0;
	}

	@Override
	public Iterable<Integer> getLinks() {
		return path;
	}

	@Override
	public Route combineWith(Route other, double ratio) {
		if(equals(other))
			return this;
		else
			return null;
	}

	@Override
	public boolean containsLink(Integer linkId) {		
		return path.contains(linkId);
	}

}
