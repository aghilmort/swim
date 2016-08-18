package swim.algorithm.integration;

/**
 *
 * @author Sherif
 */
public enum IntegrationAlgorithms {
    
    BRAINLESS_ALL_STAGE_INTEGRATION (0, " Brainless All-stage Integration"),
    TWO_STAGE_INTEGRATION           (1, " Two-stage Integration"),
    ALL_STAGE_INTEGRATION           (2, " All-stage Integration");   
        
    private final int index;        // Index in DropDown menus
    private final String longName;  // Long name of the algorithm
    
    IntegrationAlgorithms(int index, String longName) {
        this.index = index;
        this.longName = longName;
    }
    
    public int index() { return index; }
    
    public String longName() { return longName; }
}
