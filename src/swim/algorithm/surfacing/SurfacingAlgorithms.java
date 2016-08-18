package swim.algorithm.surfacing;

/**
 *
 * @author Sherif
 */
public enum SurfacingAlgorithms {
    
    SSRF  (0, " Simple Surfacing (SSRF)");

    private final int index;        // Index in DropDown menus
    private final String longName;  // Long name of the algorithm
    
    SurfacingAlgorithms(int index, String longName) {
        this.index = index;
        this.longName = longName;
    }
    
    public int index() { return index; }
    
    public String longName() { return longName; }
}
