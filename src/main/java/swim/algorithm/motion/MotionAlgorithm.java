package swim.algorithm.motion;

import com.jme3.math.FastMath;
import java.util.Deque;
import java.util.LinkedList;
import swim.algorithm.common.AlgorithmState;
import swim.core.motion.MotionGoal;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.struct.artif.Duration;

/**
 *
 * @author Sherif
 */
public abstract class MotionAlgorithm {
        
    protected long startTime, stopTime;
    
    protected Deque<AlgorithmState> savedStates;
    
    protected boolean started = false, stopped = false;
    
    protected Duration duration;
    protected int presetDur = -1, durLength = 0;
    protected boolean durLengthSet = false;
    
    protected MotionGoal goal;
    
    protected boolean interrupted = false;
    
    protected UWVehicleControl vehControl;
    protected Simulator sim;
    
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
        vehControl.setActiveMotionAlgKey(startTime);
    }
    
    protected abstract void move();
    
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;    
    }
    
    public MotionAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        this.sim = sim;
        this.vehControl = vehControl;
        this.savedStates = new LinkedList<AlgorithmState>();
    }
    
    public void applyUpdateRules() {
        if( started ) {         
            if( !stopped ) {
                move();
            } else {
                vehControl.reportMotionAlgStopped(startTime);
            }
        } else {
            System.out.println("Motion algorithm must be started first!");
        }
    }
    
    public boolean started() {
        return started;
    }
    
    public boolean stopped() {
        return stopped;
    }
    
    protected abstract void saveState();
    protected abstract void restoreState();
    
    public void pause() {
        saveState();
    }

    public void reactivate() {
        restoreState();
    }
    
    public void setDuration(Duration duration) {
        this.duration = duration;
    }
    
    public void setDuration(Duration duration, int numTicks) {
        this.duration = duration;
        this.durLength = numTicks;
        this.presetDur = numTicks;
        durLengthSet = true;
    }
    
    public int getPresetDuration() {
        return presetDur;
    }

    public void setGoal(MotionGoal goal) {
        this.goal = goal;
    }

    public MotionGoal getGoal() {
        return goal;
    }
    
    public void interrupt() {
        interrupted = true;
    }
    
    protected void checkDurationConditions() {
        if( duration != null ) {
            
            if( duration.equals(Duration.INDEFINITE) ) {
                return;
            }
            
            // Handle the special case of "OF_BEST_REWARDED_ACTION" duration
            if( duration.equals(Duration.OF_BEST_REWARDED_ACTION) ) {
                duration = vehControl.getDurOfBestRewardedAction();
            }
            
            switch(duration) {
                case PRESET:
                    decrementDurCounter();
                    break;
                case RANDOM:
                    if( !durLengthSet ) {
                        durLength = FastMath.rand.nextInt(sim.getMaxSimTickCount());
                    }
                    decrementDurCounter();
                    break;
                case TASK_CAP:
                    if( !durLengthSet ) {
                        durLength = FastMath.rand.nextInt(vehControl.getMissionTimeConstraint());
                    }
                    decrementDurCounter();
                    break;
                case TIME_TO_SATISFY_GOAL:
                    if( goal.equals(MotionGoal.FIND_VEHICLES) ) {
                        if( vehControl.getSensedNeighCount() > 0 ) {
                            stop();
                        }
                    } else if( goal.equals(MotionGoal.FIND_TARGETS) ) {
                        if( vehControl.getSensedTargetsCount() > 0 ) {
                            stop();
                        }
                    } else if( goal.equals(MotionGoal.ESCAPE_CROWD) ) {
                        if( vehControl.getSensedNeighCount() <= 3 ) {
                            stop();
                        }
                    }
                break;
                case TIME_TO_SENSE_TARGETS:
                    if( vehControl.getSensedTargetsCount() > 0 ) {
                        stop();
                    }
                    break;
                case TIME_TO_SENSE_VEHICLES:
                    if( vehControl.getSensedNeighCount() > 0 ) {
                        stop();
                    }
                    break;
                case WHILE_TARGETS_BELOW_THRES:
                    if( vehControl.getSensedTargetsCount() > 3 ) {
                        stop();
                    }
                    break;
                case UNTIL_INTERRUPTED:
                    if( interrupted ) {
                        stop();
                    }
                    break;
                case WHILE_REWARD_LVL_PERSISTS:
                    if( !vehControl.isRewardLevelPersistent() ) {
                        stop();
                    }
                    break;
                case DUR_OF_SEQUENCE:
                case TIME_TO_REACH_TARGETS:
                case TIME_TO_REACH_VEHICLES:
                case TIME_TO_ESCAPE_NEIGHBORS:
                    System.out.println("Attempt to use a motion sequence duration with a motion algorithm!");
            }
        }
    }
    
    private void decrementDurCounter() {
        if(  durLength > 0 ) {
            --durLength;
            if( durLength == 0 ) { stop(); }
        }
    }
    
    public long getKey() {
        return startTime;
    }
    
}
