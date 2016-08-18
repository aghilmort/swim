package swim.core.network;

/*
 * A pair of nodes (origin, destination)
 */
public class ODPair extends Object{
	
	public static final int maxNodeCount = 100000;
	
	private int origin, destination;
	
	public ODPair(int origin, int destination){
		this.origin = origin;
		this.destination = destination;			
	}
	
    @Override
	public boolean equals(Object o){
		ODPair _o = ((ODPair)o);
		if(_o==null)
			return false;
		else
			return _o.origin==origin && _o.destination==destination; 
	}
    
    @Override
    public int hashCode(){
		return (origin* maxNodeCount+ destination);
    }
    
	/**
	 * @return int node id of origin.
	 */		    		
	public int getOrigin(){
		return origin;		
	}

	public int getDestination(){
		return destination;
	}
	
	private final String formatString="(%d -> %d)";

	@Override
	public String toString() {
		return String.format(formatString, origin, destination); 
	}
}

