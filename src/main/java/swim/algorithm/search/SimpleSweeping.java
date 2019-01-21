package swim.algorithm.search;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.Pair;

/**
 *
 * @author Sherif
 * Divide-and-Flock Hexagonal Close Packing Search Algorithm 
 */
public class SimpleSweeping extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean SSW_DEBUG_ENABLED = false,
                          COMMUNICATION_ENABLED = false;
    private final int NUM_OF_SWEEPS = 17;// was 15
    private final float SWEEP_LENGTH = 100;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private Vector3f measuredInitMagNorth;
    private Quaternion rotQuat;
    private float travelDist;
    private ArrayList<Pair<String, Object>> compositeTurnSeq;
    private boolean turnRight = false, 
            firstSweep = true, 
            searchStarted = false, searchCompleted = false,
            turnSeqSet = false,
            inPlaceTurnSolicited = false;
    private String turnDir;
    private boolean broadcast = true;
    
    private int numTurns = NUM_OF_SWEEPS - 1,
                turnCounter = 0;
    
    private Vector3f divideAndFlockDirection;
    
    private boolean paused = false;
    // =========================================================================   
    
    public SimpleSweeping(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        rotQuat = new Quaternion();
        compositeTurnSeq = new ArrayList<Pair<String, Object>>();
    }

    @Override
    protected void search() {
       
        if( paused ) return;
        
        if(!searchStarted) {
            // Get measured magnetic north direction
            measuredInitMagNorth = vehControl.measureMagNorthDir().mult(vehControl.getVehicleSpeed());

            // Reiortient the vehicle in that direction
            vehControl.updateVehicleVelocity(measuredInitMagNorth);
            
            // Mark search as 'started'
            searchStarted = true;
            return;
        }
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        if(!turnSeqSet) {
            // Notify vehicle control of a required turn
            vehControl.solicitTurnNeeded();
            vehControl.announceCompTurnInProgress();
            compositeTurnSeq.clear();
            
            // Generate composite turn sequence and pass it to vehicle's controller
            for(int i = 1; i <= NUM_OF_SWEEPS; i++) {

                turnDir = (turnRight ? "right" : "left");
                    
                if(firstSweep) {
                    travelDist = SWEEP_LENGTH/2; 
                    firstSweep = false;
                } else {
                    travelDist = SWEEP_LENGTH;
                }
               
                compositeTurnSeq.add(new Pair<String,Object>("travel", travelDist));

                if(i < NUM_OF_SWEEPS) {
                    compositeTurnSeq.add(new Pair<String,Object>("turn_direction", turnDir));
                    compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.PI));
                    turnRight = !turnRight;
                } else {
                    turnRight = !turnRight;
                    turnDir = (turnRight ? "right" : "left");
                    compositeTurnSeq.add(new Pair<String,Object>("turn_direction", turnDir));
                    compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.HALF_PI));
                    
                    if(NUM_OF_SWEEPS % 2 == 0) {    // Do further investigation here
                        travelDist = (numTurns - 1) * vehControl.getActiveTurnRadius() * vehControl.getMinVehicleSpeed();
                    } else {
                        travelDist = numTurns * vehControl.getActiveTurnRadius() * vehControl.getMinVehicleSpeed(); 
                    }
                    
                    compositeTurnSeq.add(new Pair<String,Object>("travel", travelDist));
                    compositeTurnSeq.add(new Pair<String,Object>("turn_direction", turnDir));
                    compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.HALF_PI));
                    compositeTurnSeq.add(new Pair<String,Object>("travel", SWEEP_LENGTH/2));
                }                

            }

            // Pass the sequence to the controller
            vehControl.execCompositeTurn(compositeTurnSeq);
        
            turnSeqSet = true;
            return;
        }
            
        if(COMMUNICATION_ENABLED) {
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

                vehControl.setCommInitiated(false);

                if(broadcast) { 
                    vehControl.broadcastTargetFound();
                } else {
                    vehControl.checkNeighForFoundTarget();
                }
                broadcast = !broadcast;   

                return;
            }
        }
        
        if( searchStarted && !vehControl.compositeTurnInProgress() && !searchCompleted ) {
            searchCompleted = true;
        }
        
        if(searchCompleted && !inPlaceTurnSolicited) {
            vehControl.solicitInPlaceTurn();
            inPlaceTurnSolicited = true;
        }
        
    }

    public long getExecutionTime() {
        if(stopped) {
            return stopTime - startTime;
        } else {
            if(SSW_DEBUG_ENABLED) { 
                System.out.println("Algorithm is still running!");
            }
            return -1;
        }
    }
    
    @Override
    protected boolean targetFound() {
        return false;
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
        state.storeIntVar("numTurns", numTurns);
        state.storeIntVar("turnCounter", turnCounter);
        
        state.storeFlagVar("broadcast", broadcast);
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        state.storeFlagVar("turnRight", turnRight);
        state.storeFlagVar("firstSweep", firstSweep);
        state.storeFlagVar("searchStarted", searchStarted);
        state.storeFlagVar("searchCompleted", searchCompleted);
        state.storeFlagVar("turnSeqSet", turnSeqSet);
        state.storeFlagVar("inPlaceTurnSolicited", inPlaceTurnSolicited);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeFloatVar("travelDist", travelDist);
        
        state.storeStringVar("turnDir", turnDir);
        
        state.storeVectorVar("measuredInitMagNorth", measuredInitMagNorth);
        state.storeVectorVar("divideAndFlockDirection", divideAndFlockDirection);

        state.storeCompositeTurnSeq(compositeTurnSeq);
        
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
        state.storeIntVar("numTurns", numTurns);
        state.storeIntVar("turnCounter", turnCounter);
        
        broadcast = state.getFlagVar("broadcast");
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        turnRight = state.getFlagVar("turnRight");
        firstSweep = state.getFlagVar("firstSweep");
        searchStarted = state.getFlagVar("searchStarted");
        searchCompleted = state.getFlagVar("searchCompleted");
        turnSeqSet = state.getFlagVar("turnSeqSet");
        inPlaceTurnSolicited = state.getFlagVar("inPlaceTurnSolicited");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        travelDist = state.getFloatVar("travelDist");

        turnDir = state.getStringVar("turnDir");
        
        measuredInitMagNorth = state.getVectorVar("measuredInitMagNorth");
        divideAndFlockDirection = state.getVectorVar("divideAndFlockDirection");

        compositeTurnSeq = state.getCompositeTurnSeq();

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}


