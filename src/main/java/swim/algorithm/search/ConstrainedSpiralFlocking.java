/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.search;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import swim.algorithm.common.AlgorithmState;
import swim.algorithm.flocking.FlockingAlgorithm;
import swim.algorithm.flocking.ModifiedReynolds;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class ConstrainedSpiralFlocking extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final float FLOCKING_WEIGHT = 0.2f,    //0.38196601125f,  // 
                        SPIRAL_WEIGHT = 0.8f,      //0.61803398875f;    //
                        NEIGHBOR_ATTRACTION_PERIOD_LENGTH = 30000F; // 30 secs
    private boolean DEBUG_ENABLED = false;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private boolean broadcast = true,
                    targetJustFound = true;
    private long targetAnncounceStartTime;
    
    private final float[] wrappingFibonacci 
            = {1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 
                610, 377, 233, 144, 89, 55, 34, 21, 13, 8, 5, 3, 2, 1};
//            = {1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 55, 34, 21, 13, 8, 5, 3, 2, 1};
    
    private int fibPointer = 0;
    private float adaptiveRadius = 0.5f, baseTimeStep = 0.1f, timeStep = 0.1f;
    private boolean growRadius = true;
    
    private float cummulativeAngle = 0;
    
    private boolean searchFirstEntered = true;
    private Vector3f vehActiveVelAtStart;
    
    ArrayList<Pair<String, Object>> compositeTurnSeq;
    
    FlockingAlgorithm reynoldsFlocking;
    
    private Quaternion rotQuat;
    private Vector3f spiralTurnDirection, flockingDirection, cSFDirection;
    
    private boolean paused = false;
    // =========================================================================   
    
    public ConstrainedSpiralFlocking(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        reynoldsFlocking = new ModifiedReynolds(vehControl, sim);
        
        compositeTurnSeq = new ArrayList<Pair<String, Object>>();
        rotQuat = new Quaternion();
    }
   
    @Override
    protected void search() {
        
        if( paused ) return;
        
        if(searchFirstEntered) {
            reynoldsFlocking.start();
            vehActiveVelAtStart = vehControl.getActiveVelocity();
            searchFirstEntered = false;
        }
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
//        boolean aNeighborFoundTarget = vehControl.checkNeighForFoundTarget();
//        vehControl.setCommInitiated(false);
//        boolean broadcasted = vehControl.broadcastTargetFound();
//        vehControl.setCommInitiated(false);
        
        if( !vehControl.isTargetFound() ) {
            // Check if any neighbor found target
            vehControl.setCommInitiated(false);
            boolean aNeighborFoundTarget = vehControl.checkNeighForFoundTarget();

            if(aNeighborFoundTarget) {
                vehControl.setTargetFound(true);
                return;
            }
        }
        
        // Check if target has been found before doing any work
        if( vehControl.isTargetFound() ) {
            
            if(DEBUG_ENABLED) System.out.println("Target has been found.");
            
            if( targetJustFound ) {
                targetAnncounceStartTime = System.currentTimeMillis();
                targetJustFound = false;
            }
            
            vehControl.setCommInitiated(false);
            
            if(broadcast) { 
                vehControl.broadcastTargetFound();
            } else {
                vehControl.checkNeighForFoundTarget();
            }
            broadcast = !broadcast;
                
            //if( System.currentTimeMillis() - targetAnncounceStartTime >= NEIGHBOR_ATTRACTION_PERIOD_LENGTH ) {
                vehControl.solicitTurnSeqBreak(true);
                vehControl.updateTurningParams(UWVehicleControl.getMinTurnRadius());
                stop();
            //}
            
            return;
        }
        
//        if( fibPointer < wrappingFibonacci.length ) {
//            // Get next radius to use (from a wrapping Fibonacci sequence)
//            float currRadius = wrappingFibonacci[fibPointer] * MIN_TURN_RADIUS; // could use any other base radius > MTR
        if( adaptiveRadius >= 0 || adaptiveRadius <= 40) {
            
            cummulativeAngle += 0.1f * FastMath.PI;
            
            // Rotate initial active velocity around Y-axis in CCW direction by
            // an angle of "cummulativeAngle"
            rotQuat.fromAngleAxis(cummulativeAngle, Vector3f.UNIT_Y);
            spiralTurnDirection = rotQuat.mult(vehActiveVelAtStart.normalize());
            
            //DebugUtils.plotArrow(forceApplRelPos[2], forceVec, ColorRGBA.Orange, 2, sim);
            
            vehControl.updateTurningParams(adaptiveRadius);
            
            //System.out.println("Spiral direction: "+spiralTurnDirection.toString());
            
            flockingDirection = ((ModifiedReynolds)reynoldsFlocking).calculateFlockDirection();
            
            //System.out.println("Flocking direction: "+flockingDirection);
            
            //DebugUtils.plotArrow(forceApplRelPos[2], forceVec, ColorRGBA.Orange, 2, sim);
            
            if(spiralTurnDirection != null && flockingDirection != null) {
                cSFDirection = (spiralTurnDirection.mult(SPIRAL_WEIGHT)).add(flockingDirection.mult(FLOCKING_WEIGHT));
            } else {
                cSFDirection = spiralTurnDirection.clone();
            }
            
            
            vehControl.updateVehicleVelocity(cSFDirection);
            
            
//            vehControl.solicitTurnNeeded();
//            vehControl.announceCompTurnInProgress();
//            compositeTurnSeq.clear();
//            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", "left"));
////            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.QUARTER_PI));
//            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", 0.1f * FastMath.PI));
//            
//            vehControl.execCompositeTurn(compositeTurnSeq);
//            fibPointer++;
            
        } 
        
        if( growRadius && adaptiveRadius <= 40 ) {
            baseTimeStep += timeStep;
        } else if( adaptiveRadius >= 0 ) {
            growRadius = false;
            baseTimeStep -= timeStep;
        }
        adaptiveRadius = (float) Math.exp(0.5f*baseTimeStep);
        
        if( !growRadius && adaptiveRadius <= UWVehicleControl.getMinTurnRadius() ) {
            //sim.reportCompletedMission(vehControl.getVehicleName());
            stop();
        }
        //System.out.println("Radius: "+adaptiveRadius);
        
    }

    @Override
    protected boolean targetFound() {
        return true;
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
        
        state.storeFlagVar("broadcast", broadcast);
        state.storeFlagVar("growRadius", growRadius);
        state.storeFlagVar("searchFirstEntered", searchFirstEntered);
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeFloatVar("adaptiveRadius", adaptiveRadius);
        state.storeFloatVar("baseTimeStep", baseTimeStep);
        state.storeFloatVar("cummulativeAngle", cummulativeAngle);
        
        state.storeVectorVar("vehActiveVelAtStart", vehActiveVelAtStart);
        state.storeVectorVar("spiralTurnDirection", spiralTurnDirection);
        state.storeVectorVar("flockingDirection", flockingDirection);
        state.storeVectorVar("cSFDirection", cSFDirection);

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
        
        broadcast = state.getFlagVar("broadcast");
        growRadius = state.getFlagVar("growRadius");
        searchFirstEntered = state.getFlagVar("searchFirstEntered");
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        adaptiveRadius = state.getFloatVar("adaptiveRadius");
        baseTimeStep = state.getFloatVar("baseTimeStep");
        cummulativeAngle = state.getFloatVar("cummulativeAngle");
        
        vehActiveVelAtStart = state.getVectorVar("vehActiveVelAtStart");
        spiralTurnDirection = state.getVectorVar("spiralTurnDirection");
        flockingDirection = state.getVectorVar("flockingDirection");
        cSFDirection = state.getVectorVar("cSFDirection");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}
