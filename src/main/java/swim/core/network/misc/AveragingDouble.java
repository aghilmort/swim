package swim.core.network.misc;

import java.util.HashMap;

public class AveragingDouble{

	double sum;
	int count;
	
	public AveragingDouble(){
		sum=0;
		count=0;
	}
	
	public double add(double value){
		count++;
		sum+=value;
		return getValue();
	}	

	public double getValue() {
		return sum/count;
	}

	public int getCount() {
		return count;
	}
	
	static AveragingDouble[] makeArray(int size){
		AveragingDouble array[] = new AveragingDouble[size];
		for(int i=0; i<size; i++)
			array[i]=new AveragingDouble();
		return array;		
	}
	
	@SuppressWarnings({ "serial" })
	public static class Map extends HashMap<Integer, AveragingDouble>{

		public Map() {
			super();
		}

		public Map(int initialCapacity, float loadFactor) {
			super(initialCapacity, loadFactor);
		}

		public Map(int initialCapacity) {
			super(initialCapacity);
		}

		public Map(java.util.Map<? extends Integer, ? extends AveragingDouble> m) {
			super(m);
		}

		/* (non-Javadoc)
		 * @see java.util.HashMap#get(java.lang.Object)
		 */
		@Override
		public AveragingDouble get(Object key) {

			if(!super.containsKey(key))
				super.put((Integer)key, new AveragingDouble());
			
			return super.get(key);
		}			
	}
}
