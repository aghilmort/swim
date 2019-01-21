package swim.core.network.misc;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class SparseMatrix<T> extends Matrix<T>{
	
	/**
	 * Storage for sparse matrix elements
	 */
	Map<Integer, T>[] array;
	
	/**Initialize sparse matrix with expected 4 non-zero elements per row
	 * @param N number of rows (= number of columns) 
	 */
	public SparseMatrix(int N){
		super(N, N);
		init(4);
	}
	
	/**Initialize sparse matrix
	 * @param N number of columns
	 * @param M number of rows
	 * @param R expected number of non-zero elements per row
	 */
	public SparseMatrix(int N, int M, int R){
		super(N, M);
		init(R);
	}
	
	public void init(int R){
		array = new HashMap[N];
		for(int i=0; i<N; i++){
			array[i] = new HashMap<Integer, T>(R);
		}		
	}

	@Override
	public T get(int i, int j) {
		return array[i-1].get(j);
	}

	@Override
	public void set(int i, int j, T value) {
		array[i-1].put(j, value);
	}
	
	public Set<Integer> getRowNonZeroColumns(int i){
		return array[i-1].keySet();
	}
	
	public Collection<T> getRowNonZeroValues(int i){
		return array[i-1].values();
	}
	
	public static class IntegerSparseMatrix extends SparseMatrix<Integer>{

		public IntegerSparseMatrix(int N) {
			super(N);
		}

		/* (non-Javadoc)
		 * @see tn.algorithm.SparseMatrix#get(int, int)
		 */
		@Override
		public Integer get(int i, int j) {
			Integer result = super.get(i, j);
			if(result==null) result=0;

			return result;
		}		
	}	
}

