package swim.algorithm.search;

/**
 *
 * @author Sherif
 */
public enum SearchAlgorithms {
   
    RPSO  (0, " Robotic PSO (RPSO)"),
    CSF   (1, " Constrained Spiral Flocking (CSF)"),
    VTS   (2, " Virtual Tether Search (VTS)"),
    RDPSO (3, " Robotic Darwinian PSO (RDPSO)"), 
    BFM   (4, " Bacterium Flocking Model (BFM)"),
    BDFSO (5, " Baterium Darwinian Flocking Swarm Optimization (BDFSO)"),
    DHCP  (6, " Divided Hexagonal Close Packing (DHCP)"),
    SDHCP (7, " Swirling Divided Hexagonal Close Packing (SDHCP)"),
    SSDHCP(8, " Seven Sweeps Divided Hexagonal Close Packing (SSDHCP)"),
    SRW   (9, " Simple Random Walk (SRW)"),
    SSW   (10, " Simple Sweeping (SSW)"),
    FSRW  (11, " Flocked Simple Random Walk (FSRW)");
    
    
    private final int index;        // Index in DropDown menus
    private final String longName;  // Long name of the algorithm
    
    SearchAlgorithms(int index, String longName) {
        this.index = index;
        this.longName = longName;
    }
    
    public int index() { return index; }
    
    public String longName() { return longName; }
}
