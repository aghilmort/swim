/**
 * 
 */
package swim.core.network.misc;

import swim.core.network.err.SanityCheckFailed;

/**
 * @author Sherif Tolba
 *
 */
public class Util {

	public static void sanityCheck(Boolean cond, String msg) throws SanityCheckFailed{
		if(!cond){
			throw new SanityCheckFailed(msg, new Throwable().getStackTrace()[1].getFileName(), new Throwable().getStackTrace()[1].getLineNumber());
		}
	}
}
