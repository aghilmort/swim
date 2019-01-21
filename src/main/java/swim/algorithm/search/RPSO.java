/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.search;

import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import swim.algorithm.common.AlgorithmState;
import swim.sim.SimulationMode;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.DebugUtils;

/**
 *
 * @author Sherif
 */
public class RPSO extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean PSO_DEBUG_ENABLED = false,
                          DEBUG_LOCAL_BEST_ANCHOR = false,
                          DEBUG_SELF_BEST_ANCHOR = false,
                          DEBUG_PREV_SELF_BEST_ANCHOR = false;
    
    private final int PART_BEST_WEIGHT = 2,
                      LOCAL_BEST_WEIGHT = 2;
//    private final float PART_BEST_WEIGHT = 0.2f,
//                      LOCAL_BEST_WEIGHT = 0.5f;
    private final float INERTIA_WEIGHT = 0.8f;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private Vector3f p_i_minus_x_i, p_l_minus_x_i,
            particleBestComp, localBestComp,
            p_l, p_i, x_i, v_i,
            rand_1, rand_2;
    
    private Vector3f prevSelfBestPos;
    private boolean paused = false;
    // =========================================================================
    
    public RPSO(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    @Override
    protected void search() {
        
        if( vehControl.isSimModeEnabled(SimulationMode.INTEGRATION_MODE) &&
            vehControl.isTargetFound() ) {
            vehControl.solicitTurnSeqBreak(true);
            stop();
            return;
        }

        if( paused ) return;
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        vehControl.broadcastSelfBestPos();
        
        if(vehControl.compositeTurnInProgress()) {
           return; 
        }

        v_i = vehControl.getCurrentVehicleVelocity();
        
        x_i = vehControl.getCurrentVehiclePosition();

        p_i = vehControl.getVehicleBestPos();
        
        if(p_i == null) {
            
            if(prevSelfBestPos != null) {
                p_i = prevSelfBestPos.clone();
            }
            if(p_i != null && DEBUG_PREV_SELF_BEST_ANCHOR) {
                System.out.println("Previous Best is: "+p_i.toString());
                DebugUtils.plotArrow(x_i, p_i.subtractLocal(x_i), ColorRGBA.Green, 2, sim);
            }
        }
        
        if(p_i == null) {
            p_i_minus_x_i = new Vector3f(0,0,0);         
        } else {
            p_i_minus_x_i = p_i.subtract(x_i);
        }
        
        if(DEBUG_SELF_BEST_ANCHOR) {
            DebugUtils.plotArrow(x_i, p_i_minus_x_i, ColorRGBA.Red, 2f, sim);
        }
        
        rand_1 = new Vector3f(FastMath.rand.nextFloat(), FastMath.rand.nextFloat(), FastMath.rand.nextFloat());
        
        particleBestComp = p_i_minus_x_i.mult(rand_1.mult(PART_BEST_WEIGHT));
        
        if( PSO_DEBUG_ENABLED && particleBestComp.length() != 0 ) {
            System.out.println("(PSO) Particle best update component: "+particleBestComp);
        }
        
        p_l = vehControl.getNeighborhoodBestPosition();
        
        if(p_l == null) {
            p_l = vehControl.getMostRecentNeighPosPointer();
            if(PSO_DEBUG_ENABLED && p_l != null) {
                System.out.println("Most recent neighborhood best position for ("+vehControl.getVehicleName()+"): "+p_l);
            }
        }
        
        if(p_l == null) {
            p_l_minus_x_i = new Vector3f(0,0,0);
        } else {
            p_l_minus_x_i = p_l.subtract(x_i);
        }
        
        rand_2 = new Vector3f(FastMath.rand.nextFloat(), FastMath.rand.nextFloat(), FastMath.rand.nextFloat());
        
        localBestComp = p_l_minus_x_i.mult(rand_2.mult(LOCAL_BEST_WEIGHT));
        
        if( PSO_DEBUG_ENABLED && localBestComp.length() != 0 ) {
            System.out.println("(PSO) Local best update component: "+localBestComp);
        }
        
        if(DEBUG_LOCAL_BEST_ANCHOR) {
            DebugUtils.plotArrow(x_i, p_l_minus_x_i, ColorRGBA.Cyan, 2f, sim);
        }

        v_i = (v_i.mult(INERTIA_WEIGHT)).add(particleBestComp).add(localBestComp);
        
        vehControl.updateVehicleVelocity(v_i);
        
        vehControl.solicitTurnNeeded();

        vehControl.setCommInitiated(false);
        
        if(p_i != null) {
            prevSelfBestPos = p_i;
        }
    }
    
    public long getExecutionTime() {
        if(stopped) {
            return stopTime - startTime;
        } else {
            if(PSO_DEBUG_ENABLED) { 
                System.out.println("Algorithm is still running!");
            }
            return -1;
        }
    }

    @Override
    protected boolean targetFound() {
        return true;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {
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
        
        state.storeVectorVar("p_i_minus_x_i", p_i_minus_x_i);
        state.storeVectorVar("p_l_minus_x_i", p_l_minus_x_i);
        state.storeVectorVar("particleBestComp", particleBestComp);
        state.storeVectorVar("localBestComp", localBestComp);
        state.storeVectorVar("p_l", p_l);
        state.storeVectorVar("p_i", p_i);
        state.storeVectorVar("x_i", x_i);
        state.storeVectorVar("v_i", v_i);
        state.storeVectorVar("rand_1", rand_1);
        state.storeVectorVar("rand_2", rand_2);
        state.storeVectorVar("prevSelfBestPos", prevSelfBestPos);
        
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
        
        p_i_minus_x_i = state.getVectorVar("p_i_minus_x_i");
        p_l_minus_x_i = state.getVectorVar("p_l_minus_x_i");
        particleBestComp = state.getVectorVar("particleBestComp");
        localBestComp = state.getVectorVar("localBestComp");
        p_l = state.getVectorVar("p_l");
        p_i = state.getVectorVar("p_i");
        x_i = state.getVectorVar("x_i");
        v_i = state.getVectorVar("v_i");
        rand_1 = state.getVectorVar("rand_1");
        rand_2 = state.getVectorVar("rand_2");
        prevSelfBestPos = state.getVectorVar("prevSelfBestPos");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}
