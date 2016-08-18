/**
 * 
 */
package uwsn.math.optim.glpk;

import org.gnu.glpk.*;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/**
 * @author Saleh Ibrahim, Sherif Tolba
 *
 */
public class Problem {

	final int Result_INFEASIBLE = -1;
	private static glp_prob lp;			/** problem instance */
	private glp_smcp parm;				/** simplex solver constraint parameters (smcp) */
	private double sol[];				//TODO: make it dynamic according to number of input variables
	private int nrows;
	private int ret;
	
	public static glp_prob readLP(String LPFilename, int verbose, String LPTitle){
		
		// Create problem
		lp = GLPK.glp_create_prob();
		GLPK.glp_read_lp(lp, null, LPFilename);
		System.out.println("Problem created");
		
		GLPK.glp_set_prob_name(lp, LPTitle);
		
		return lp;
	}

	public int solve() throws IOException{
		
		/*******************
		* Problem Solving *
		*******************/
		
		sol = new double[1000];

		/* solve problem */
		System.out.println("solve problem");
		System.out.println("=============");
		
		try{
			ret = GLPK.glp_simplex(lp , parm );
			
			System.out.println("primal solution:");
			System.out.println("================");
			
			int i ;
			int n;
			String name;
			double val ;
			name = GLPK.glp_get_obj_name(lp);
			val = GLPK.glp_get_obj_val(lp);
			System.out.print(name);
			System.out.print(" = ");
			System.out.println(val);
			n = GLPK.glp_get_num_cols(lp);
			for ( i = 1; i <= n; i++) {
				name = GLPK.glp_get_col_name(lp, i);
				val = GLPK.glp_get_col_prim(lp, i);
				
				if(name.charAt(0) == 'x'){
					char[] numPart = new char[name.length()];
					name.getChars(1, name.length(), numPart, 0);
					int j = Integer.parseInt(new String(numPart));
					if(j > nrows)
						nrows = j;
					this.sol[j+1] = val;
				}
				
				if(name.charAt(0) == 'J')
					this.sol[0] = val;

				System.out.print(name);
				System.out.print(" = ");
				System.out.println(val);
			}
			
			/**************
			* Statistics *
			**************/
			
			System.out.print("Statistics");
			System.out.print("==========");
			
			// Write the sensitivity analysis report to a file then read it
			// (glpk library doesn't provide a direct way to output these 
			// results to the console directly)
			GLPK.glp_print_ranges(lp, 0, null, 0, "Sensitivity_Analysis_Report.txt");
			
			BufferedReader bReader = new BufferedReader(new FileReader("Sensitivity_Analysis_Report.txt"));
			
			String sensAnalysisRes = "", line = null;
			while((line = bReader.readLine()) != null){
				sensAnalysisRes += line;
			}
			System.out.println(sensAnalysisRes);
			
			
			/* only call heuristic, if an optimal LP solution is at hand */
			int stat = GLPK.glp_get_status(lp);
			
			if(stat!= GLPKConstants.GLP_OPT)
				return Result_INFEASIBLE;

			return 0;
			
		}catch(GlpkException ex) {
			ex.printStackTrace();
		}
		
		return -1;
	}

	public double[] getPtrPrimalSolution(){
		return sol;
	}

	public int getNRows(){
		return nrows;
	}

	public int getNameIndex(String name, Boolean isrow){
		if(name.charAt(0) == 'x'){
			char[] numPart = new char[name.length()];
			name.getChars(1, name.length(), numPart, 0);
			int j = Integer.parseInt(new String(numPart));
			return j+1;
		}
		else
			return 0;
	}
	
	private int readParams(
				String filename	/**< parameter file name, or NULL */
			){
		// Did not find a method for reading parameters from file
		//TODO: Check if there is a way to work around this.
		
		return 0;
	}

	private int fromCommandLine(
			String filename            /**< input file name */
			){
		
		/********************
		* Problem Creation *
		********************/

		System.out.println("read problem <");
		System.out.println(filename);
		System.out.println(">");
		System.out.println("============");
		
		try{
			int retCode;
			retCode = GLPK.glp_simplex(lp , parm );
			return 0;
		}catch(GlpkException ex) {
			ex.printStackTrace();
		}
		
		return -1;
	}

	private Problem(){
		init();
	}
	
	private Problem(String paramFile){
		//TODO: Find if it is possible to read prameters from a parm file.
	}
	
	private int init(){
		
		/***********************
		* Version information *
		***********************/
		System.out.println(GLPK.glp_version());
		
		/**************
		* Parameters *
		**************/
		parm = new glp_smcp();
		GLPK.glp_init_smcp(parm);	// initialize the simplex solver with the default parameters
		
		return 0;
		
	}

	private int init(char paramFile){
		//TODO: Check if there is a way to store parameters in a file and read them from there. 
		return 0;
	}

}
