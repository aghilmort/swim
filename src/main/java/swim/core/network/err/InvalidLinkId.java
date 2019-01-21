package swim.core.network.err;

public class InvalidLinkId extends Exception{

	/**
	 * 
	 */
	private static final long serialVersionUID = -3407554305868188640L;
	
	int linkCount;
	int invalidLinkId;

	public InvalidLinkId(int linkCount, int invalidLinkId){
		this.linkCount = linkCount;
		this.invalidLinkId = invalidLinkId;
	}
	
	private static final String errorMessage = "Invalid link id!\nLink id must be greater than 1 and less than or equal to the link count.\nLink count = %d, you supplied link id = %d.\n";
	@Override
	public String getMessage() {
		return String.format(errorMessage, linkCount);
	}

	private static final String objectID = "InvalidLinkId Exception(1<= %d <= %d)";
	@Override
	public String toString() {
		return String.format(objectID, invalidLinkId, linkCount);
	}		
}


