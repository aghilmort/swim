package swim.sim.uwswarm.intelli.brain.struct.artif;

import swim.core.motion.CompositeTurnSequence;

/**
 *
 * @author Sherif
 */
public class MotorFuncType {
    
    private MotionType type;
    private MotionAlgorithm motionAlg;
    private String sequenceName;
    
    public MotorFuncType(MotionType type, String sequenceName) {
        this.type = type;
        this.sequenceName = sequenceName;
    }
    
    public MotorFuncType(MotionType type, MotionAlgorithm motionAlg) {
        this.type = type;
        this.motionAlg = motionAlg;
    }
    
    public MotorFuncType(MotionType type, MotionAlgorithm motionAlg, String sequenceName) {
        this.type = type;
        this.sequenceName = sequenceName;
        this.motionAlg = motionAlg;
    }
    
    public MotionType getMotionType() {
        return this.type;
    }

    public MotionAlgorithm getMotionAlgorithm() {
        return motionAlg;
    }

    public String getSequenceName() {
        return sequenceName;
    }
    
    
}
