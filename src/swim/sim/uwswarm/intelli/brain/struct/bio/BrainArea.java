/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.bio;

import java.util.HashMap;
import java.util.Map;

/**
 *
 * @author Sherif
 */
public abstract class BrainArea {
 
    protected Map<Enum,SubBrainArea> subBrainAreas;
    
    public BrainArea( int numSubAreas ) {
        subBrainAreas = new HashMap<Enum,SubBrainArea>(numSubAreas);
    }
    
    public SubBrainArea getSBArea( Enum areaName ) {
        return subBrainAreas.get(areaName);
    }
    
    /**
     * Core function of this brain area. Implementation should use it to execute
     * the main operations performed by this area.
     */
    protected abstract void function();
    
    /**
     * Takes an exciation signal (used to excite this specific brain area) 
     * as input and implements how the excitation process takes place.
     * @param signal input excitation signal
     */
    public abstract void excite(ExcitationSignal signal);
    
    /**
     * Any signals that this brain area may fire/send to other brain areas after
     * functioning and takening the necessary actions.
     */
    protected abstract void fire();
    
}
