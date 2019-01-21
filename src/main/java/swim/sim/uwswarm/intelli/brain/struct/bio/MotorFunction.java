package swim.sim.uwswarm.intelli.brain.struct.bio;

import swim.sim.uwswarm.intelli.brain.constr.BrainConstruct;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotorFuncParams;

/**
 *
 * @author Sherif
 */
public class MotorFunction implements BrainConstruct {
    
    private MotorFuncParams motorFuncParams;
    private String constrName = "Motor Function";
    private String name;
    
    public MotorFunction(MotorFuncParams motorFuncParams) {
        this.motorFuncParams = motorFuncParams;
        this.name = motorFuncParams.name;
    }
    
    public String getName() {
        return name;
    }

    @Override
    public String getConstructName() {
        return constrName;
    }

    @Override
    public void setConstructName(String name) {
        this.constrName = name;
    }
    
    public MotorFuncParams getParameters() {
        return motorFuncParams;
    }

}
