package swim.sim.uwswarm.intelli.brain;

import swim.sim.uwswarm.intelli.brain.struct.bio.ActionSignal;
import swim.sim.uwswarm.intelli.brain.struct.bio.ExcitationSignal;
import swim.sim.uwswarm.intelli.Nutrient;

/**
 *
 * @author Sherif
 */
public interface Brain {
    
    /**
     * Takes an exciation signal (used to excite one or more brain areas) 
     * as input and implements how the excitation process takes place.
     * @param signal input excitation signal
     */
    public void excite(ExcitationSignal signal);
    
    /**
     * Uses the brain to decide proper actions.
     */
    public void makeDecision();
    
    /**
     * Used internally to implement the learning process; should be 
     * automatically invoked on every brain excitation.
     */
    void learn();
    
    /**
     * Used internally to check the current state of mind in order to affect the
     * behavior based on emotions.
     * @return a mood signal that can be used for affecting the action to be 
     * taken
     */
    //public MoodSignal getMood();
    
    /**
     * Given a specific nutrient, provide nutrition to brain areas that 
     * benefit from that nutrient.
     * @param nutrient one of four types of nutrients that determines the which
     * brain areas to nurture
     */
    public void nurture(Nutrient nutrient);
    
}
