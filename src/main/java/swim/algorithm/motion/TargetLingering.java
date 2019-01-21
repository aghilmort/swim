package swim.algorithm.motion;

import com.jme3.math.Vector3f;
import swim.algorithm.common.AlgorithmState;
import swim.core.StateChangeReason;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.struct.artif.Duration;

/**
 *
 * @author Sherif
 */
public class TargetLingering extends MotionAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final float MIN_ALLOW_DIST_TO_TARG = 2;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private String[] targetsList;
    
    private boolean motionFirstEntered = true;
    
    private String oldTargName, currTargName;
    private Vector3f oldTargPosition;
    private boolean paused = false;
    // =========================================================================
    
    public TargetLingering(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    @Override
    protected void move() {
        
        if( paused ) return;
        
        if(motionFirstEntered) {
            // Do something here ...
            motionFirstEntered = false;
        }
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        // Get the list of sensed targets from the vehicle controller
        targetsList = vehControl.getSensedTargetsList();
        
        Vector3f estTargPosition, estVehPosition, newVelVector;
        
        if( targetsList != null && targetsList.length > 0 ) {
            
            // Take the first target as our current target
            currTargName = targetsList[0];
            
            // Estimate the location of that target
            estTargPosition = vehControl.simpleEstTargetPos(currTargName);
        
            // Estimate vehicle position
            estVehPosition = vehControl.estimatePos();
            
            // Find a vector from the vehicle to the target
            newVelVector = estTargPosition.subtract(estVehPosition);
            
            // Check target is the one encountered in previous step and has become
            // closer than a certain threshold
            if( oldTargName != null && currTargName.equals(oldTargName) ) {
                if( newVelVector.length() < MIN_ALLOW_DIST_TO_TARG ) {
                    if( targetsList.length > 1 ) {
                        
                        // Take the next target as our current target
                        currTargName = targetsList[1];
                        
                        // Estimate location of the next target
                        estTargPosition = vehControl.simpleEstTargetPos(currTargName);
                        
                        // Recalculate the vector from the vehicle to the target
                        newVelVector = estTargPosition.subtract(estVehPosition);
                    } else {
                        // No other targets currently sensed ... switch of short random jumps
                        vehControl. requestFSMStateChange(
                                StateChangeReason.RUN_MOTION_ALG, 
                                swim.sim.uwswarm.intelli.brain.struct.artif.MotionAlgorithm.RANDOM_WALK_SMALL_JUMPS, 
                                Duration.TIME_TO_SATISFY_GOAL);
                        
                        return;
                    }
                }
            }
            
            // Update vehicle velocity
            vehControl.updateVehicleVelocity(newVelVector);
            
            // Save target's name and estimated position
            oldTargName = currTargName;
            oldTargPosition = estTargPosition;
            
        } else {
            if( oldTargName != null ) {
                // Estimate vehicle position
                estVehPosition = vehControl.estimatePos();
                
                // Find a vector from the vehicle to the target
                newVelVector = oldTargPosition.subtract(estVehPosition);
                
                if( newVelVector.length() < MIN_ALLOW_DIST_TO_TARG ) {
                    // No other targets currently sensed ... switch of short random jumps
                    vehControl. requestFSMStateChange(
                            StateChangeReason.RUN_MOTION_ALG, 
                            swim.sim.uwswarm.intelli.brain.struct.artif.MotionAlgorithm.RANDOM_WALK_SMALL_JUMPS,
                            Duration.TIME_TO_SATISFY_GOAL);
                }
            }
        }
    }
    
    @Override
    protected void saveState() {
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        
        state.storeStringVar("oldTargName", oldTargName);
        state.storeStringVar("currTargName", currTargName);
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        state.storeFlagVar("motionFirstEntered", motionFirstEntered);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeVectorVar("oldTargPosition", oldTargPosition);
        
        state.storeStringArray("targetsList", targetsList);
        
        savedStates.push(state);
        
        paused = true;
    }

    @Override
    protected void restoreState() {
        AlgorithmState state = savedStates.pop();
        useState(state);
    }
    
    protected void useState(AlgorithmState state) {
       
        duration = state.getDuration();
        goal = state.getGoal();
        
        presetDur = state.getIntVar("presetDur");
        durLength = state.getIntVar("durLength");
        
        oldTargName = state.getStringVar("oldTargName");
        currTargName = state.getStringVar("currTargName");
        
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        motionFirstEntered = state.getFlagVar("motionFirstEntered");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        oldTargPosition = state.getVectorVar("oldTargPosition");
        
        targetsList = state.getStringArray("targetsList");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}