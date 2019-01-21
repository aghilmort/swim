package swim.algorithm.taskalloc;

/**
 *
 * @author Sherif
 */
public enum TaskAllocAlgorithms {
    EXP_RESP_THRESHOLD  (0, " Exponential Resp Threshold (ERT)"),
    POLY_RESP_THRESHOLD (1, " Polynomial Resp Threshold (PRT)"),
    MINI_BRAIN_BASED    (2, " MiniBrain Based TA (MBB)"),
    BEACON_BASED_TA     (3, " Beacon Based TA (BBTA)"),
    HYBRID_TA           (4, " Hybrid TA (HTA)"),
    BLIND_TA            (5, " Blind TA (BLTA)");
        
        
    private final int index;        // Index in DropDown menus
    private final String longName;  // Long name of the algorithm
    
    TaskAllocAlgorithms(int index, String longName) {
        this.index = index;
        this.longName = longName;
    }
    
    public int index() { return index; }
    
    public String longName() { return longName; }
}
