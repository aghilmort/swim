///**
// * 
// */
//package uwsn.math.opt.lpsolve;
//
//import lpsolve.*;
//
///**
// * @author Sherif Tolba (manually translated a C++ code written by Saleh Ibrahim)
// *
// */
//public class LPSolve {
//
//	enum ConstraintType
//	{
//		Constr_LE (1),
//		Constr_EQ (3),
//		Constr_GE (2),
//		Constr_FR (0);
//		
//		private final int value;
//		
//		ConstraintType(int value){
//			this.value = value;
//		}
//		
//	};
//
//	enum ScaleType
//	{
//		Scale_EXTREME 		(1),
//		Scale_RANGE 		(2),
//		Scale_MEAN 			(3),
//		Scale_GEOMETRIC 	(4),
//		Scale_CURTISREID 	(7),
//		Scale_QUADRATIC 	(8),
//		Scale_LOGARITHMIC 	(16),
//		Scale_USERWEIGHT 	(31),
//		Scale_POWER2 		(32),
//		Scale_EQUILIBRATE 	(64),
//		Scale_INTEGERS 		(128),
//		Scale_DYNUPDATE 	(256),
//		Scale_ROWSONLY 		(512),
//		Scale_COLSONLY 		(1024);
//		
//		private final int value;
//		
//		ScaleType(int value){
//			this.value = value;
//		}
//	};
//
//	enum ImproveType
//	{
//		Improve_NONE	  (0),
//		Improve_SOLUTION  (1),
//		Improve_DUALFEAS  (2),
//		Improve_THETAGAP  (4),
//		Improve_BBSIMPLEX (8),
//		Improve_DEFAULT   (Improve_DUALFEAS.value + Improve_THETAGAP.value),
//		Improve_INVERSE   (Improve_SOLUTION.value + Improve_THETAGAP.value);
//		
//		private final int value;
//		
//		ImproveType(int value){
//			this.value = value;
//		}		
//	};
//
//	enum PivotingType
//	{
//		Pricer_FIRSTINDEX 		(0),
//		Pricer_DANTZIG 			(1),
//		Pricer_DEVEX 			(2),
//		Pricer_STEEPESTEDGE 	(3),
//		Price_PRIMALFALLBACK 	(4),
//		Price_MULTIPLE 			(8),
//		Price_PARTIAL 			(16),
//		Price_ADAPTIVE 			(32),
//		Price_HYBRID 			(64),
//		Price_RANDOMIZE 		(128),
//		Price_AUTOPARTIALCOLS 	(256),
//		Price_AUTOPARTIALROWS 	(512),
//		Price_LOOPLEFT 			(1024),
//		Price_LOOPALTERNATE 	(2048),
//		Price_AUTOPARTIAL (Price_AUTOPARTIALCOLS.value + Price_AUTOPARTIALROWS.value);
//		
//		private final int value;
//		
//		PivotingType(int value){
//			this.value = value;
//		}	
//	};
//
//	enum PresolveType
//	{
//		Presolve_NONE (0),
//		Presolve_ROWS (1),
//		Presolve_COLS (2),
//		Presolve_LINDEP (4),
//		Presolve_SOS (32),
//		Presolve_REDUCEMIP (64),
//		Presolve_KNAPSACK (128),
//		Presolve_ELIMEQ2 (256),
//		Presolve_IMPLIEDFREE (512),
//		Presolve_REDUCEGCD (1024),
//		Presolve_PROBEFIX (2048),
//		Presolve_PROBEREDUCE (4096),
//		Presolve_ROWDOMINATE (8192),
//		Presolve_COLDOMINATE (16384),
//		Presolve_MERGEROWS (32768),
//		Presolve_IMPLIEDSLK (65536),
//		Presolve_COLFIXDUAL (131072),
//		Presolve_BOUNDS (262144),
//		Presolve_DUALS (524288),
//		Presolve_SENSDUALS (1048576);
//		
//		private final int value;
//		
//		PresolveType(int value){
//			this.value = value;
//		}	
//	};
//
//	enum AntiDegenerationType
//	{
//		AntiDegen_NONE 			(0),
//		AntiDegen_FIXEDVARS 	(1),
//		AntiDegen_COLUMNCHECK 	(2),
//		AntiDegen_STALLING 		(4),
//		AntiDegen_NUMFAILURE 	(8),
//		AntiDegen_LOSTFEAS 		(16),
//		AntiDegen_INFEASIBLE 	(32),
//		AntiDegen_DYNAMIC 		(64),
//		AntiDegen_DURINGBB 		(128),
//		AntiDegen_RHSPERTURB 	(256),
//		AntiDegen_BOUNDFLIP 	(512);
//		
//		private final int value;
//		
//		AntiDegenerationType(int value){
//			this.value = value;
//		}			
//	};
//
//	enum BasisCrashType
//	{
//		Crash_NOTHING (0),
//		Crash_MOSTFEASIBLE (2);
//		
//		private final int value;
//		
//		BasisCrashType(int value){
//			this.value = value;
//		}	
//	};
//
//	enum SimplexType
//	{
//		Simplex_PRIMAL_PRIMAL 	(5),
//		Simplex_DUAL_PRIMAL 	(6),
//		Simplex_PRIMAL_DUAL 	(9),
//		Simplex_DUAL_DUAL 		(10);
//		
//		private final int value;
//		
//		SimplexType(int value){
//			this.value = value;
//		}	
//	};
//
//	enum BrachAndBoundType
//	{
//		Node_FIRSTSELECT 		(0),
//		Node_GAPSELECT 			(1),
//		Node_RANGESELECT 		(2),
//		Node_FRACTIONSELECT 	(3),
//		Node_PSEUDOCOSTSELECT 	(4),
//		Node_PSEUDONONINTSELECT (5),
//		Node_PSEUDORATIOSELECT 	(6),
//		Node_USERSELECT 		(7),
//		Node_WEIGHTREVERSEMODE 	(8),
//		Node_BRANCHREVERSEMODE 	(16),
//		Node_GREEDYMODE 		(32),
//		Node_PSEUDOCOSTMODE 	(64),
//		Node_DEPTHFIRSTMODE 	(128),
//		Node_RANDOMIZEMODE 		(256),
//		Node_GUBMODE 			(512),
//		Node_DYNAMICMODE 		(1024),
//		Node_RESTARTMODE 		(2048),
//		Node_BREADTHFIRSTMODE 	(4096),
//		Node_AUTOORDER 			(8192),
//		Node_RCOSTFIXING 		(16384),
//		Node_STRONGINIT 		(32768);
//		
//		private final int value;
//		
//		BrachAndBoundType(int value){
//			this.value = value;
//		}	
//	};
//
//	enum ResultType
//	{
//		Result_NOMEMORY 	(-2),
//		Result_OPTIMAL 		(0),
//		Result_SUBOPTIMAL 	(1),
//		Result_INFEASIBLE 	(2),
//		Result_UNBOUNDED 	(3),
//		Result_DEGENERATE 	(4),
//		Result_NUMFAILURE 	(5),
//		Result_USERABORT 	(6),
//		Result_TIMEOUT 		(7),
//		Result_PRESOLVED 	(9),
//		Result_PROCFAIL 	(10),
//		Result_PROCBREAK 	(11),
//		Result_FEASFOUND 	(12),
//		Result_NOFEASFOUND 	(13);
//		
//		private final int value;
//		
//		ResultType(int value){
//			this.value = value;
//		}			
//	};
//
//	enum BranchType
//	{
//		Branch_CEILING 	 (0),
//		Branch_FLOOR 	 (1),
//		Branch_AUTOMATIC (2);
//		
//		private final int value;
//		
//		BranchType(int value){
//			this.value = value;
//		}			
//	};
//
//	enum MessageType
//	{
//		Message_PRESOLVE (1),
//		Message_LPFEASIBLE (8),
//		Message_LPOPTIMAL (16),
//		Message_MILPEQUAL (32),
//		Message_MILPFEASIBLE (128),
//		Message_MILPBETTER (512);
//		
//		private final int value;
//		
//		MessageType(int value){
//			this.value = value;
//		}	
//	};
//
//}
