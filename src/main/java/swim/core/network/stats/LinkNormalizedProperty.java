package swim.core.network.stats;

import swim.core.network.Network;
import swim.core.network.misc.ReadOnlyProperty;

public class LinkNormalizedProperty extends ReadOnlyProperty<Double>{
	double mean, stdDev;
	ReadOnlyProperty<Double> grades;
	
	public LinkNormalizedProperty(ReadOnlyProperty<Double> grades){
		super(grades);
		this.grades = grades;
		LinkPropertyStatistics stats = new LinkPropertyStatistics(grades);
		mean = stats.getMean();
		stdDev = stats.getStdDev();
	}

	@Override
	public ReadOnlyProperty<Double> clone(Network network) {
		return new LinkNormalizedProperty(grades.clone(network)); 
	}

	@Override
	public Double get(int linkId) {
		return (grades.get(linkId)-mean)/stdDev;
	}
}

