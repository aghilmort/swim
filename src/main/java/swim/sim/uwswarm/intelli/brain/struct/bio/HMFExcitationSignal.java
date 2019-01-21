package swim.sim.uwswarm.intelli.brain.struct.bio;

import java.util.ArrayList;
import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;

/**
 *
 * @author Sherif
 */
public class HMFExcitationSignal extends ExcitationSignal {
    
    private ArrayList<Trigger> stimuli;
    
    public HMFExcitationSignal(ArrayList<Trigger> stimuli) {
        this.stimuli = stimuli;
    }

    public ArrayList<Trigger> extractStimuli() {
        return stimuli;
    }
    
}
