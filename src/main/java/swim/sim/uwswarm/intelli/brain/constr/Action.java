package swim.sim.uwswarm.intelli.brain.constr;

import swim.sim.uwswarm.intelli.brain.struct.artif.Trigger;
import swim.sim.uwswarm.intelli.brain.struct.bio.MotorFunction;

/**
 *
 * @author Sherif
 */
public class Action extends Trigger implements BrainConstruct {
    
    private ActionType actionType;
    private Trigger trigger;
    private String name = "Action";
    private float score = 0;
    
    private MotorFunction motorFunction;
    
    public Action(ActionType actionType) {
        super(new Enum[0]); // action does not have signals
        this.actionType = actionType;
    }
    
    public Action(ActionType actionType, Trigger trigger) {
        super(new Enum[0]); // action does not have signals
        this.actionType = actionType;
        this.trigger = trigger;
    }
    
    public void setMotorFunction(MotorFunction motorFunction) {
        this.motorFunction = motorFunction;
    }
    
    public MotorFunction getMotorFunction() {
        return motorFunction;
    }
    
    public String getMotorFunctionName() {
        return motorFunction.getName();
    }
    
    public float getScore() {
        return score;
    }
    
    public void setScore(float score) {
        this.score = score;
    }    
        
    @Override
    public String getConstructName() {
        return name;
    }

    @Override
    public void setConstructName(String name) {
        this.name = name;
    }

    public ActionType getType() {
        return actionType;
    }
  
}
