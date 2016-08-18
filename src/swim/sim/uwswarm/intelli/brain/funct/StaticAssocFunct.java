package swim.sim.uwswarm.intelli.brain.funct;

import java.util.HashMap;
import java.util.Map;
import swim.exception.StaticMapModificationException;
import swim.sim.uwswarm.intelli.brain.constr.BrainConstruct;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;

/**
 *
 * @author Sherif
 */
public class StaticAssocFunct implements BrainFunct {
    
    private Map<Integer, BrainConstruct> staticAssocMap;
    
    private boolean built = false;

    public StaticAssocFunct() {
        this.staticAssocMap = new HashMap<Integer, BrainConstruct>();
    }
    
    public BrainConstruct getAssociation(Integer key) {
        return staticAssocMap.get(key);
    }
    
    public void addAssociation(Integer key, BrainConstruct value) throws StaticMapModificationException {
        if( !built ) {
            staticAssocMap.put(key, value);
        } else {
            throw new StaticMapModificationException();
        }
    }
    
    public void build() {
        built = true;
    }
    
}
