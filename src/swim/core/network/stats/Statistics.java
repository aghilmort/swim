/**
 * 
 */
package swim.core.network.stats;

/**
 * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
 *
 */
public class Statistics {
	
	public static class Variable{
		
		double sum, sum2, min;
		int frequency;
		
		public Variable(double value){
			this.sum = value;
			this.sum2 = Math.pow(value, 2);
			this.min = value;
			this.frequency = 1;
		}
		
		public Variable(){
			this.sum = 0;
			this.sum2 = 0;
			this.min = 0;
			this.frequency = 0;
		}
		
		public void addSample(double value){
			if(frequency == 0 || ((Integer)frequency).equals(null)){
				min = value;
			}else if(min > value){
				min = value;
			}
			
			sum += value;
			sum2 += Math.pow(value, 2);
			frequency++;
		}
		
		public double plusEquals(double value){
			addSample(value);
			return value;
		}
		
		public void plusEquals(Variable v){
			sum += v.sum;
			sum2 += v.sum2;
			frequency += v.frequency;
		}
		
        public double mean(){
            return sum/frequency;
        }
        
        public double stdDev()
        {
            return Math.sqrt(sum2/frequency - Math.pow(mean(), 2));
        }
        
        public int getFrequency(){
            return frequency;
        }
        
        public double doubleVal(){
            return mean();
        }
		
	}
	
}
