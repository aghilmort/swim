package swim.core.network.misc;

import java.util.HashMap;
import java.util.Set;

import swim.core.network.Network;

/**
 *<b>Description:</b><br><br>
 *This class is responsible for creating a map (hash map) for a specific link property. 
 *The map holds link IDs and the corresponding property values. It also has some 
 *map accessibility and cloning and modification functions. 
 */


public class PropertyMap<T> extends Property<T> {


	/**
	 * Link property internal storage (a hash map that stores link IDs and 
	 * the corresponding properties)
	 */
	protected HashMap<java.lang.Integer, T> map;

	
	
	/**Standard constructor: Initialize {@link PropertyMap} object:
	 * <ul>
	 * <li>Create a {@link ReadOnlyProperty}</li>
	 * <li>Create a hash map and initialize it with link IDs</li>
	 * </ul>
	 * @param name property name
	 * @param network the network with which this property is associated
	 * @param defaultValue default value of the property
	 */
	public PropertyMap(String graphComponenType, String name, Network network, T defaultValue) {
		super(graphComponenType, name, network, defaultValue);
		map = new HashMap<java.lang.Integer, T>(network.getLinkCount());
	}

	/**Copy constructor: Makes a copy of the given property map and stores the 
	 * non-default-value link-property values in this new property map 
	 * @param sourceMap property to copy
	 */
	public PropertyMap(ReadOnlyProperty<T> sourceMap) {
		super(sourceMap);
		map = new HashMap<Integer, T>(network.getLinkCount());
		/* for each link in the network, if the property value is not the default one
		 * store it in the new map
		 */
		for(int i:network.getLinks()){
			if(sourceMap.get(i)!=defaultValue)
				map.put(i, sourceMap.get(i));
		}
	}

	/**Copy constructor: Initialize {@link PropertyMap} object using the 
	 * content of another {@link PropertyMap} object.
	 * @param other the other {@link PropertyMap} to be copied
	 */
	public PropertyMap(PropertyMap<T> other){
		super(other.getGraphComponentType(), other.getName(), other.network, other.defaultValue);
		map = new HashMap<java.lang.Integer, T>(other.map);
	}

	/**Copy constructor: Makes a copy of the given property map for a different network
	 *  and stores the non-default value link-property values in this new property map 
	 * @param network new network
	 * @param sourceMap property to copy
	 */
	public PropertyMap(String graphComponenType, Network network, ReadOnlyProperty<T> sourceMap) {
		super(graphComponenType, sourceMap.getName(), network, sourceMap.defaultValue);
		map = new HashMap<Integer, T>(network.getLinkCount());
		/* for each link in the network, if the property value is not the default one
		 * store it in the new map
		 */		
		for(int i:network.getLinks()){
			if(sourceMap.get(i)!=defaultValue)
				map.put(i, sourceMap.get(i));
		}
	}

	/* (non-Javadoc)
	 * @see tn.LinkReadOnlyProperty#get(int)
	 */
	@Override
	/* Implementation of the get method of the ReadOnlyProperty class.
	 * Return the property value for a given link if the link is stored in 
	 * the link property map, otherwise return the default value of that property. 
	 * The returned value is of type T. 
	 */   
	public T get(int id) {
		if(map.containsKey(id))
			return map.get(id);
		else
			return defaultValue;
	}

	/* (non-Javadoc)
	 * @see tn.LinkProperty#set(int, java.lang.Object)
	 */
	@Override
	/* Remove the links with default values, and store 
	 * the given value as the values for the given links
	 */
	public void set(int linkId, T value) {
		if(value.equals(defaultValue))
			map.remove(linkId);
		else
			map.put(linkId, value);
	}

	/**Update the link property values from the given map. 
	 * Links that do not have a mapping in the given map remain unchanged.
	 * @param other source of link property values
	 */
	public void set(PropertyMap<T> other){
		map.putAll(other.map);
	}
					
	/* (non-Javadoc)
	 * @see tn.misc.LinkProperty#reset()
	 */
	@Override
	// As the name implies, it clears all values in the map
	public void reset() {
		map.clear();
	}

	@Override
	// Make a copy of the PropertyMap
	public PropertyMap<T> clone() {
		return new PropertyMap<T>(this);
	}

	@Override
	// Make a copy for another network
	public PropertyMap<T> clone(Network network) {
		return new PropertyMap<T>(this.getGraphComponentType(), network, this);
	}
	
	
	/**
	 * Returns the set of link IDs contained in the map
	 * @return {@link Set} the set of link IDs
	 */
	public Set<Integer> getNonDefaultLinkIdSet(){
		return map.keySet();
	}
}
