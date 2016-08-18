package swim.sim.uwswarm.intelli.brain.struct.bio;

import swim.sim.uwswarm.intelli.brain.constr.Action;

/**
 *
 * @author Sherif
 */
public class ActionSignal extends ExcitationSignal {

    private Action action;
    
    public ActionSignal(Action action) {
       this.action = action;
    }
    
    public Action getAction() {
        return action;
    }
}
