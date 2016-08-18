package swim.core.network.misc;

import swim.core.network.Network;

/**
 *<b>Description:</b><br><br>
 * This class creates a wrapper for a function to be applied to a certain link property. It
 * enables accessing the function, and making a copy of it. 
 */


public class PropertyFunction<T> extends ReadOnlyProperty<T> {
	
	/** An interface that when implemented represents a function that can be applied 
	 * on a property. It enables getting the function itself and making a copy 
	 * of that function.
	 */
	static public interface Function<T>{
		public T get(int linkId);
		public Function<T> clone(Network network);
	}

	
	/**
	 * A property's function holder (holds a function that can be applied on a certain property) 
	 */
	Function<T> function;
	
	/** Constructor - Creates a {@link PropertyFunction} object by creating a
	 * {@link ReadOnlyProperty} object with the given name and network, and a 
	 * default value of null, and then setting the function of the 
	 * {@link PropertyFunction} object to be the given function
	 * 
	 * @param name property name
	 * @param network the network that the link belongs to
	 * @param function the function to be applied to the property
	 */
	public PropertyFunction(String graphComponentType, String name, Network network, Function<T> function) {
		super(graphComponentType, name, network, (T)null);
		this.function = function;
	}

	
	/**
	 * Gets the {@link Function} that is applied to a specific link 
	 * @return T the function applied to the specified link 
	 */
	@Override
	public T get(int linkId) {
		return function.get(linkId);
	}

	
	/**
	 * Makes a copy of the {@link ReadOnlyProperty} for use in the given network
	 * @return PropertyFunction the copy of the current {@link ReadOnlyProperty}
	 */
	@Override
	public ReadOnlyProperty<T> clone(Network network) {
		return new PropertyFunction<T>(this.getGraphComponentType(), name, network, function);
	}
}
