package swim.algorithm.motion;

import com.jme3.math.Vector3f;
import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class NeighborsAverage extends MotionAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final float POS_VICINITY_LIMITS = 2f;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private Vector3f newVehVelocityVect, avgNeigPos;
    
    private boolean motionFirstEntered = true;
    private boolean paused = false;
    // =========================================================================   
    
    public NeighborsAverage(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    @Override
    protected void move() {
        
        if( paused ) return;
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        if(motionFirstEntered) {
            
            // Get vehicles in sensing range
            String[] sensedVehicles = vehControl.getSensedVehiclesList();

            // Hold current neighbor's estimated position
            Vector3f estNeighborPos;

            // Will hold the average neighbor position
            avgNeigPos = new Vector3f(0, 0, 0);

            if(sensedVehicles != null && sensedVehicles.length > 0) {

                // Build neighbor position vector
                for(String neigh : sensedVehicles) {
                    estNeighborPos = vehControl.simpleEstNeighborPos(neigh);
                    avgNeigPos = avgNeigPos.add(estNeighborPos);
                }
                avgNeigPos = avgNeigPos.mult(1/sensedVehicles.length);  

                // Estimate vehicle position
                Vector3f vehPos = vehControl.estimatePos();
                
                // Find new vehicle velocity
                newVehVelocityVect = avgNeigPos.subtract(vehPos);
                
                // Update vehicle velocity
                vehControl.updateVehicleVelocity(newVehVelocityVect);
            }
                    
            motionFirstEntered = false;
        } else {
            
            // Estimate vehicle position
            Vector3f vehPos = vehControl.estimatePos();

            if( avgNeigPos.subtract(vehPos).length() < POS_VICINITY_LIMITS  ) {
                stop();
                vehControl.notifyFSMStateChangeReqEnded();
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
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        state.storeFlagVar("motionFirstEntered", motionFirstEntered);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeVectorVar("newVehVelocityVect", newVehVelocityVect);
        state.storeVectorVar("avgNeigPos", avgNeigPos);
        
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
        motionFirstEntered = state.getFlagVar("motionFirstEntered");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        newVehVelocityVect = state.getVectorVar("newVehVelocityVect");
        avgNeigPos = state.getVectorVar("avgNeigPos");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
    
}
