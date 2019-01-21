package swim.core.network.err;

import java.util.List;

import swim.core.network.UWSensorNetwork;

public class InvalidPath extends Exception{
	
	private static final long serialVersionUID = 3632840906299440677L;		
	
	private List<Integer> path;
	private int causingLink;
	
	public InvalidPath(List<Integer> path2, int causingLink){
		super();
		this.path = path2;
		this.causingLink = causingLink;
	}

	private static final String errorMessage = "Invalid path! All links must form a connected path.\nMake sure that each link starts at the same node at which the previous link in the path ends.\nCurrent error occured at link number %d of the path.\n";
	@Override
	public String getMessage() {
		return String.format(errorMessage, causingLink);
	}

	private static final String objectID = "InvalidPath Exception(%d)";
	@Override
	public String toString() {
		return String.format(objectID, causingLink);
	}		
}