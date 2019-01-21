/**
 * 
 */
package swim.core.network.misc.geometry;

import java.io.IOException;


/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public interface Object {
	
	public abstract void draw(Drawing drawing) throws IOException;
	public abstract Object intersect(Object obj);
	
}
