package swim.core.network.misc;

import java.util.Set;

import swim.core.network.Network;

/**
 *<b>Description:</b><br><br>
 * This class creates a link property that is modifiable (not a read only). It 
 * provides some accessibility functions that can be used to set a property's value
 * for a specific link, reset the values of the properties of a group of links to their 
 * default value, set the properties of some links to some value, and make copies of the 
 * property.
 */


public abstract class Property<T> extends ReadOnlyProperty<T> {
	
	/** Copy constructor - creates a copy of the given {@link ReadOnlyProperty}.
	 * 
	 * @param other the {@link ReadOnlyProperty} to be copied
	 */
	public Property(ReadOnlyProperty<T> other) {
		super(other);
	}

	/** Copy constructor - Creates a copy of the given {@link ReadOnlyProperty}
	 * for another network.
	 * @param network the other network to create the copy for
	 * @param other the {@link ReadOnlyProperty} to be copied 
	 */
	public Property(Network network, ReadOnlyProperty<T> other) {
		super(network, other);
	}

	/** Constructor - create a new property with the given name and default 
	 * value for the given network
	 * 
	 * @param name name of the property
	 * @param network the network the property is created for
	 * @param defaultValue the default value of the property
	 */
	public Property(String graphComponentType, String name, Network network, T defaultValue) {
		super(graphComponentType, name, network, defaultValue);
	}

	/**Set the value of the property of the given linkId to the given value
	 * @param linkId the link to change the value for
	 * @param value the new value
	 */
	public abstract void set(int linkId, T value);
	
	/**Reset the value of the property for all links to its default value
	 */
	public void reset(){
		for(int linkId: network.getLinks()){
			set(linkId, defaultValue);
		}
	}
	
	/**Set the value of the property corresponding to the given
	 * set of link indexes in the keySet to the given value.
	 * @param linkIdSet set of linkIds to update
	 * @param value new value of the property
	 */
	public void set(Set<Integer>linkIdSet, T value){
		for(Integer linkId: linkIdSet){
			set(linkId, value);
		}
	}
	
	/* (non-Javadoc)
	 * @see tn.misc.LinkReadOnlyProperty#clone()
	 */
	@Override
	/*
	 * Make a copy of the Property under consideration
	 */
	public abstract Property<T> clone();

	/* (non-Javadoc)
	 * @see tn.misc.LinkReadOnlyProperty#clone(tn.Network)
	 */
	@Override
	/*
	 * Make a copy of the Property under consideration
	 * for a given network.
	 */
	public abstract Property<T> clone(Network network);	
	
}
