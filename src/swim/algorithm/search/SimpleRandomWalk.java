/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.search;

import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.Random;
import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.DebugUtils;

/**
 *
 * @author Sherif
 */
public class SimpleRandomWalk extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    //private boolean SRW_DEBUG_ENABLED = false;
    private int INTER_TURN_TICK_COUNT = 1;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private int tickCounter = INTER_TURN_TICK_COUNT + 1;
    private boolean paused = false;
    // =========================================================================   
    
    public SimpleRandomWalk(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    @Override
    protected void search() {
        
        if( paused ) return;
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        if(tickCounter > INTER_TURN_TICK_COUNT) {
           vehControl.generateRandomTurn();
           tickCounter = 0;
        } else {
            tickCounter++;
        }
        
    }

    @Override
    protected boolean targetFound() {
        return false;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {}
    
    public void setJumpLength(int simTicks) {
        INTER_TURN_TICK_COUNT = simTicks;
    }
    
    @Override
    protected void saveState() {
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        state.storeIntVar("INTER_TURN_TICK_COUNT", INTER_TURN_TICK_COUNT);
        state.storeIntVar("tickCounter", tickCounter);
        
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
        INTER_TURN_TICK_COUNT = state.getIntVar("INTER_TURN_TICK_COUNT");
        tickCounter = state.getIntVar("tickCounter");
        
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

