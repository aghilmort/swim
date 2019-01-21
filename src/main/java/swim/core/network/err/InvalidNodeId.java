package swim.core.network.err;

public class InvalidNodeId extends Exception{

	/**
	 * 
	 */
	private static final long serialVersionUID = -3407884305868188640L;
	
	int nodeCount;
	int invalidNodeId;

	public InvalidNodeId(int nodeCount, int invalidNodeId){
		this.nodeCount = nodeCount;
		this.invalidNodeId = invalidNodeId;
	}
	
	private static final String errorMessage = "Invalid node id!\nNode id must be greater than 1 and less than or equal to the node count.\nNode count = %d, you supplied node id = %d.\n";
	@Override
	public String getMessage() {
		return String.format(errorMessage, nodeCount);
	}

	private static final String objectID = "InvalidNodeId Exception(1<= %d <= %d)";
	@Override
	public String toString() {
		return String.format(objectID, invalidNodeId, nodeCount);
	}		
}

