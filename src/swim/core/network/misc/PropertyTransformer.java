package swim.core.network.misc;

import org.apache.commons.collections15.Transformer;

/**
 * Encapsulate a {@link Property<Double>} in a {@link Transformer<Integer, Number>} wrapper
 * So that it can be passed to JUNG functions
 */
public class PropertyTransformer implements Transformer<Integer, Number> {

	final ReadOnlyProperty<Double> property;
	
	public PropertyTransformer(ReadOnlyProperty<Double> property){
		
		this.property = property;
	}
	
	@Override
	public Double transform(Integer linkId) {
		
		return property.get(linkId);
	}
}

