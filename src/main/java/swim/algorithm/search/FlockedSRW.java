/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.search;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import swim.algorithm.common.AlgorithmState;
import swim.algorithm.flocking.FlockingAlgorithm;
import swim.algorithm.flocking.ModifiedReynolds;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class FlockedSRW extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    //rivate boolean SRW_DEBUG_ENABLED = false;
    private final int INTER_TURN_TICK_COUNT = 1;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private FlockingAlgorithm reynoldsFlocking;
    private Vector3f flockingDirection;
    private boolean searchFirstEntered = true;
    private int gracePeriodInTicks = 0;  
    
    private int tickCounter = INTER_TURN_TICK_COUNT + 1;
    private boolean paused = false;
    // =========================================================================   
    
    public FlockedSRW(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        reynoldsFlocking = new ModifiedReynolds(vehControl, sim);
    }

    @Override
    protected void search() {
        
        if( paused ) return;
        
        if(searchFirstEntered) {
            reynoldsFlocking.start();
            searchFirstEntered = false;
        }
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        flockingDirection = ((ModifiedReynolds)reynoldsFlocking).calculateFlockDirection();
        
        if( gracePeriodInTicks > 0 ) {
            --gracePeriodInTicks;
        }
        
        if( flockingDirection == null ) {
            if( gracePeriodInTicks == 0 ) {
                if( tickCounter > INTER_TURN_TICK_COUNT ) {
                    vehControl.generateRandomTurn();
                    tickCounter = 0;
                 } else {
                     tickCounter++;
                 }
            }

        } else {
            
            gracePeriodInTicks = (int)(FastMath.ceil(((ModifiedReynolds)reynoldsFlocking).getCohVecLength()/
                    vehControl.getActiveVelocity().length()));
            vehControl.updateVehicleVelocity(flockingDirection);
            
            //System.out.println("Flock direction: " + flockingDirection);
            //System.out.println("Grace period: " + gracePeriodInTicks);
        }
        
    }

    @Override
    protected boolean targetFound() {
        return false;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {}
    
    @Override
    protected void saveState() {
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        state.storeIntVar("tickCounter", tickCounter);
        state.storeIntVar("gracePeriodInTicks", gracePeriodInTicks);
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        state.storeFlagVar("searchFirstEntered", searchFirstEntered);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeVectorVar("flockingDirection", flockingDirection);
        
        reynoldsFlocking.pause();
        
        savedStates.push(state);
        
        paused = true;
    }   
    
    @Override
    protected void restoreState() {
        AlgorithmState state = savedStates.pop();
        useState(state);
    }
    
    protected void useState(AlgorithmState state) {
        
        reynoldsFlocking.reactivate();
        
        duration = state.getDuration();
        goal = state.getGoal();
        
        presetDur = state.getIntVar("presetDur");
        durLength = state.getIntVar("durLength");
        tickCounter = state.getIntVar("tickCounter");
        gracePeriodInTicks = state.getIntVar("gracePeriodInTicks");
        
        
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        searchFirstEntered = state.getFlagVar("searchFirstEntered");

        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");

        flockingDirection = state.getVectorVar("flockingDirection");
        
        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}
