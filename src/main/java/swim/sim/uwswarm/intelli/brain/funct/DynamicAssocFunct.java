package swim.sim.uwswarm.intelli.brain.funct;

import java.util.HashMap;
import java.util.Map;
import swim.sim.uwswarm.intelli.brain.constr.BrainConstruct;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;

/**
 *
 * @author Sherif
 */
public class DynamicAssocFunct {
    
    private Map<Integer, BrainConstruct> dynamicAssocMap;

    public DynamicAssocFunct() {
        this.dynamicAssocMap = new HashMap<Integer, BrainConstruct>();
    }
    
    public BrainConstruct getAssociation(Integer key) {
        return dynamicAssocMap.get(key);
    }
    
    public void addAssociation(Integer key, BrainConstruct value) {
        dynamicAssocMap.put(key, value);
    }
    
    public void updateAssociation(Integer key, BrainConstruct value) {
        dynamicAssocMap.put(key, value);
    }
    
}
