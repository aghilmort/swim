package swim.core.network.stats;

import java.util.Arrays;

import swim.core.network.Network;
import swim.core.network.misc.ReadOnlyProperty;

public class LinkPropertyStatistics {
	
	private ReadOnlyProperty<Double> grades;
	double mean, stdDev;
	Network network;
	
	int rank[];

	public LinkPropertyStatistics(ReadOnlyProperty<Double> grades){
		this.grades  = grades;
		this.network = grades.getNetwork();
		
		mean = 0;
		for(int linkId:network.getLinks()){
			mean+=grades.get(linkId);
		}
		mean/=network.getLinkCount();

		stdDev = 0;
		for(int linkId:network.getLinks()){
			double dev = grades.get(linkId)-mean;
			stdDev +=dev*dev;
		}
		stdDev = Math.sqrt(stdDev/network.getLinkCount());
	}

	public double getMean() {
		return mean;
	}

	public double getStdDev() {
		return stdDev;
	}
	
	RankProperty rankProperty=null;
	
	public RankProperty getRank(){
		
		if(rankProperty==null){
			
			double[] gradesArray =  new double[network.getLinkCount()+1];
			for(int i:network.getLinks()){
				gradesArray[i] = grades.get(i);
			}
			
			rank = new int[network.getLinkCount()+1];
			
			Arrays.sort(gradesArray, 1, network.getLinkCount()+1);
			for(int i:network.getLinks()){
				rank[i] = Arrays.binarySearch(gradesArray, 1, network.getLinkCount()+1, grades.get(i));
			}
			rankProperty = new RankProperty();
		}
		
		return rankProperty;
	}
	
	class RankProperty extends ReadOnlyProperty<Double>{

		public RankProperty() {
			super("link", "Rank", LinkPropertyStatistics.this.network, 0.0);
		}

		@Override
		public ReadOnlyProperty<Double> clone(Network network) {
			return this;
		}

		@Override
		public Double get(int linkId) {			
			return (double)(rank[linkId]);
		}		
	}
	
	public static double getCovariance(ReadOnlyProperty<Double> p1, ReadOnlyProperty<Double> p2){
		double cov = 0;
		Network network = p1.getNetwork();
		
		for(int i:network.getLinks()){
			cov+=p1.get(i) * p2.get(i);
		}
		cov/=network.getLinkCount();
		
		return cov;		
	}
}
