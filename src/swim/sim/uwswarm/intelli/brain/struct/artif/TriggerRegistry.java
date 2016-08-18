package swim.sim.uwswarm.intelli.brain.struct.artif;

import java.util.ArrayList;
import java.util.HashMap;

/**
 *
 * @author Sherif
 */
public class TriggerRegistry {
    
    private HashMap<Integer, ArrayList<Trigger>> preBehaviorTriggers, postBehaviorTriggers;
    
    public TriggerRegistry() {
        preBehaviorTriggers = new HashMap<Integer, ArrayList<Trigger>>();
        postBehaviorTriggers = new HashMap<Integer, ArrayList<Trigger>>();
    }
    
    public void setPreBehaviorTriggers(Integer time, ArrayList<Trigger> triggers) {
        preBehaviorTriggers.put(time, triggers);
    }
    
    public void setPostBehaviorTriggers(Integer time, ArrayList<Trigger> triggers) {
        postBehaviorTriggers.put(time, triggers);
    }
    
    public ArrayList<Trigger> getPostBehaviorTriggers(int triggersTime) {
        return postBehaviorTriggers.get(triggersTime);
    }
    
    public ArrayList<Trigger> getPreBehaviorTriggers(int triggersTime) {
        return preBehaviorTriggers.get(triggersTime);
    }
    
    public boolean preBehaviorTrigsSet() {
        return preBehaviorTriggers.size() > 0;
    }
    
    public boolean postBehaviorTrigsSet() {
        return postBehaviorTriggers.size() > 0;
    }
    
    public boolean preBehaviorTriggersTrackedFor(int time) {
        return preBehaviorTriggers.containsKey(time);
    }
    
    public boolean postBehaviorTriggersTrackedFor(int time) {
        return preBehaviorTriggers.containsKey(time);
    }
    
}
