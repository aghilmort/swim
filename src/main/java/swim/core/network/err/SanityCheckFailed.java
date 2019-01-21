/**
 * 
 */
package swim.core.network.err;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */

/**
 * Exception class for failed sanity checks. A sanity check verifies that 
 * an object is in a consistent state.
 */
public class SanityCheckFailed extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 5339947968378348777L;

	private String fullMessage;
	
	public SanityCheckFailed(String errorMessage, String srcFileName, int srcLineNumber){
		super(errorMessage);
		
		fullMessage = String.format("Sanity check failed at\t%s-#%d :\t%s", srcFileName, srcLineNumber, errorMessage);
	}
	
	public String what(){
		return fullMessage;
	}
	
}
