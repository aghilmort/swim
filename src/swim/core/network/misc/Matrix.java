package swim.core.network.misc;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public abstract class Matrix<T> {
	
	/**
	 * Number of columns
	 */
	protected final int M;

	/**
	 * Number of rows
	 */
	protected final int N;
	
	/**Initialize the matrix dimensions
	 * @param N number of rows
	 * @param M number of columns
	 */
	public Matrix(int N, int M){
		this.M = M;
		this.N = N;		
	}

	/**Set the value of the cell at row i and column j
	 * @param i row index in [1 , n]
	 * @param j column index in [1, m]
	 * @param value
	 */
	public abstract void set(int i, int j, T value);

	/**Get the value of the cell at row i and column j 
	 * @param i row index in [1, n]
	 * @param j column index in [1, m]
	 * @return the value at row i and column j
	 */
	public abstract T get(int i, int j);

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		StringBuilder b = new StringBuilder();
		for(int i=1;i<=N; i++){
			for(int j=1; j<=M; j++){
				b.append(String.format("%s, ", get(i,j).toString()));
			}
			b.append('\n');
		}
		return b.toString();
	}

	public void setRow(int i, Map<Integer, T> row) {
		for(int j=1; j<=M; j++){
			set(i,j, row.get(j));
		}
	}	

	public void setRow(int i, T[] row) {
		for(int j=1; j<=M; j++){
			set(i,j, row[j]);
		}
	}
	
	static class IntegerArrayMatrix extends Matrix<Integer>{
		
		int[][] array;
		
		public IntegerArrayMatrix(int N, int M){
			super(N, M);
			array = new int[N][M];
		}
		
		/* (non-Javadoc)
		 * @see tn.algorithm.Martix#set(int, int, int)
		 */
		public void set(int i, int j, Integer value){
			array[i-1][j-1] = value;
		}
		
		/* (non-Javadoc)
		 * @see tn.algorithm.Martix#get(int, int)
		 */
		public Integer get(int i, int j){
			return array[i-1][j-1];
		}
	}

	public static class DoubleArrayMatrix extends Matrix<Double>{
		
		double[][] array;
		
		public DoubleArrayMatrix(int N, int M){
			super(N, M);
			array = new double[N][M];
		}
		
		/* (non-Javadoc)
		 * @see tn.algorithm.Martix#set(int, int, int)
		 */
		public void set(int i, int j, Double value){
			array[i-1][j-1] = value;
		}
		
		/* (non-Javadoc)
		 * @see tn.algorithm.Martix#get(int, int)
		 */
		public Double get(int i, int j){
			return array[i-1][j-1];
		}
		
	}
	
	public static class IntegerListArrayMatrix extends Matrix<List<Integer>>{
		
		List<Integer>[][] array;
		
		@SuppressWarnings("unchecked")
		public IntegerListArrayMatrix(int N, int M){
			super(N, M);
			array = new LinkedList[N][M];
		}
		
		/* (non-Javadoc)
		 * @see tn.algorithm.Martix#set(int, int, int)
		 */
		public void set(int i, int j, List<Integer> value){
			array[i-1][j-1] = value;
		}
		
		/* (non-Javadoc)
		 * @see tn.algorithm.Martix#get(int, int)
		 */
		public List<Integer> get(int i, int j){
			return array[i-1][j-1];
		}
		
	}


}