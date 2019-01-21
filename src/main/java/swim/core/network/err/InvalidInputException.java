package swim.core.network.err;

/*
 * Invalid input found in input stream
 */
public class InvalidInputException extends Exception{

	private static final long serialVersionUID = 2962405670854121957L;
	
	private String fileName;
	private int lineNumber;
	private Exception detailedException;
	
	public InvalidInputException(String fileName, int lineNumber, Exception detailedException){
		this.fileName = fileName;
		this.lineNumber = lineNumber;
		this.detailedException = detailedException;
	}
	
	@Override
	public String getMessage() {
		return String.format("Invalid input in file %s at line %d:\n%s", fileName, lineNumber, detailedException.getMessage());
	}
	
	private static final String objectID = "Network.InvalidInputException(%s, %d, %s)";

	@Override
	public String toString() {
		return String.format(objectID, fileName, lineNumber, detailedException.toString());  
	}
	
	public Throwable getCause() {
		return detailedException;
	}
}
