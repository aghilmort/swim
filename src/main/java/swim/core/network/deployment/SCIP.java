///**
// * 
// */
//package uwsn.deployment;
//
///**
// * @author Sherif
// *
// */
//public class SCIP {
//
//	final int Result_INFEASIBLE = -1;
//	
//	class Problem {
//		
//		public static Problem readLP(String LPFilename, int verbose, String LPTitle){
//			
//			Problem p = new Problem();
//
//			p.fromCommandLine(LPFilename);
//			
//			return p;
//		}
//
//		public int solve(){
//			
//			return Result_INFEASIBLE;
//
//			/*******************
//			* Problem Solving *
//			*******************/
//
//			/* solve problem */
//			System.out.println("solve problem");
//			System.out.println("=============");
//			SCIP_CALL( SCIPsolve(scip) );
//
//			std::cout << std::endl << "primal solution:" << std::endl;
//			std::cout << "================" << std::endl << std::endl;
//			SCIP_CALL( SCIPprintBestSol(scip, NULL, FALSE) );
//
//			/**************
//			* Statistics *
//			**************/
//
//			std::cout << std::endl << "Statistics" << std::endl;
//			std::cout << "==========" << std::endl << std::endl;
//
//			SCIP_CALL( SCIPprintStatistics(scip, NULL) );
//
//			/* only call heuristic, if an optimal LP solution is at hand */
//			SCIP_LPSOLSTAT stat = SCIPgetLPSolstat(scip); 
//
//			if(stat!= SCIP_LPSOLSTAT_OPTIMAL)
//				return Result_INFEASIBLE;
//		
//			SCIP_CALL( SCIPgetVarsData(scip, &vars, &nvars, NULL, NULL, NULL, NULL) );
//			nrows =0;
//			for(int i=0; i<nvars; i++)
//			{
//				const char* varName = SCIPvarGetName(vars[i]);
//
//				if(varName[0]=='x')
//				{
//					int j = atoi(varName+1);
//					if(j>nrows)
//						nrows = j;
//					this->sol[j+1] = SCIPvarGetLPSol(vars[i]);
//				}
//				
//				if(varName[0]=='J')
//					this->sol[0] = SCIPvarGetLPSol(vars[i]);
//			}
//
//			return 0;
//			
//		}
//
//		public void getPtrPrimalSolution(double pSol);
//
//		public int getNrows();
//
//		public int getNameIndex(String name, Boolean isrow);
//	}
//}
