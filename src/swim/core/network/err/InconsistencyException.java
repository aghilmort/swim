package swim.core.network.err;

import swim.core.network.Network;
import swim.core.network.ODPair;

/**<b>Description:</b><br><br>
 * This class creates an {@link Exception} to identify cases when an object is not handled by the same {@link Network} that constructed it.
 * For example, NEED EXAMPLES OF BAD USAGE.
 */
public class InconsistencyException extends Exception{

	/**
	 * The serial version UID
	 */
	private static final long serialVersionUID = -5093234821090794904L;
	
	/**
	 * The {@link Network} that was used to handle the object.
	 */
	private Network network;

	/**
	 * The {@link Object} that was not handled by the {@link Network}, which created it.
	 */
	private Object obj;

	/**
	 * Instantiate {@link Exception} assigning values to identify the the {@link Network} that was used to handle the object and 
	 * the {@link Object} that was not handled by the {@link Network}, which created it.
	 * @param network Network that was used to handle object.
	 * @param obj Object that was mishandled.
	 */
	public InconsistencyException(Network network, Object obj){
		this.network = network;
		this.obj =  obj;
	}

	/**
	 * The error message.
	 */
	private static final String errorMessage = "Inconsistent network object.\nAn object must be handled by the same network that constructed it.\n";

	/* (non-Javadoc)
	 * @see java.lang.Exception#getMessage()
	 */
	@Override
	public String getMessage() {
		return String.format(errorMessage);
	}

	/**
	 * The object identifier string.
	 */
	private static final String objectID = "Inconsistency Exception()";
	
	/* (non-Javadoc)
	 * @see java.lang.Exception#toString()
	 */
	@Override
	public String toString() {
		return String.format(objectID);
	}		

}
