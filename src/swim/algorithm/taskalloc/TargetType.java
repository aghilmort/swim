package swim.algorithm.taskalloc;

/**
 *
 * @author Sherif
 */
public enum TargetType {
    
    // Order is important
    RED_BARREL   (0),
    GREEN_BARREL (1),
    BLUE_BARREL  (2),
    SHIP         (3);
        
    private final int index; // order
    
    TargetType(int index) {
        this.index = index;
    }
    
    public int index() { return index; }
    
    public static TargetType getType(int index) {
        for(TargetType type : values()) {
            if( type.ordinal() == index ){
                return type;
            }
        }
        return null;
    }
}
