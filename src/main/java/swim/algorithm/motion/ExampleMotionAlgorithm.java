package swim.algorithm.motion;

import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class ExampleMotionAlgorithm extends MotionAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    
    
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private long startTime, stopTime;
    
    private boolean motionFirstEntered = true;
    private boolean paused = false;
    // =========================================================================   
    
    public ExampleMotionAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }
    
    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
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
        
        // Core functionality goes here ...
    }

    @Override
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
    }
    
    @Override
    protected void saveState() {
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
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
        
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}
