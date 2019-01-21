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
 * Divide-and-Flock Hexagonal Close Packing Search Algorithm 
 */
public class DHCP extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean DHCP_DEBUG_ENABLED = false;
    private final int NUM_DIVIDE_DIRECTIONS = 6;
    private final float[] DIVIDE_ANGLES = {0, 
                                        FastMath.PI/3, 
                                        FastMath.TWO_PI/3, 
                                        FastMath.PI, 
                                        4*FastMath.PI/3,
                                        5*FastMath.PI/3};
    private final float HEX_SIDE_LEN,
                        DIVERGENCE_STEP_LEN,
                        NUM_DIVERGENCE_STEP_MULTIPLES = 6,
                        SENSE_RANGE_MULTIPLES_USED = 1.5f;
    
    private final float FLOCKING_WEIGHT = 0.2f,
                        HCP_WEIGHT = 0.8f;    
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private Vector3f measuredInitMagNorth, divideDirection;
    private Quaternion rotQuat;
    private float travelDistOne, travelDistTwo;
    private ArrayList<Pair<String, Object>> compositeTurnSeq;
    private boolean turnRight = true, lastTurn = false, 
            searchStarted = false, searchCompleted = false,
            turnSeqSet = false,
            inPlaceTurnSolicited = false;
    private String turnDir, oppTurnDir;
    private boolean broadcast = true;
    
    FlockingAlgorithm reynoldsFlocking;
    
    private Vector3f flockingDirection, divideAndFlockDirection;
    private boolean paused = false;
    // =========================================================================   
    
    public DHCP(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        reynoldsFlocking = new ModifiedReynolds(vehControl, sim);
        
        rotQuat = new Quaternion();
        DIVERGENCE_STEP_LEN = SENSE_RANGE_MULTIPLES_USED * this.vehControl.getSensingRange();
        HEX_SIDE_LEN = DIVERGENCE_STEP_LEN * NUM_DIVERGENCE_STEP_MULTIPLES;
        //HEX_SIDE_LEN = 2 * vehControl.getActiveTurnRadius() * NUM_DIVERGENCE_STEP_MULTIPLES;
        compositeTurnSeq = new ArrayList<Pair<String, Object>>();
    }

    @Override
    protected void search() {
       
        if( paused ) return;
        
        if(!searchStarted) {
            // Get measured magnetic north direction
            measuredInitMagNorth = vehControl.measureMagNorthDir().mult(vehControl.getVehicleSpeed());

            // Select an angle that is a multiple of 60 degrees randomly and rotate 
            // the measured magnetic north direction by that angle
            float angle = DIVIDE_ANGLES[(int)Math.floor(Math.random()*NUM_DIVIDE_DIRECTIONS)];
            rotQuat.fromAngleAxis(angle, Vector3f.UNIT_Y);
            divideDirection = rotQuat.mult(measuredInitMagNorth);
            
            // Calculate flock direction
            reynoldsFlocking.start();
            flockingDirection = ((ModifiedReynolds)reynoldsFlocking).calculateFlockDirection();
            
            // Calculate a weighted direction
            if(divideDirection != null && flockingDirection != null) {
                divideAndFlockDirection = (divideDirection.mult(HCP_WEIGHT)).add(flockingDirection.mult(FLOCKING_WEIGHT));
            } else {
                divideAndFlockDirection = divideDirection.clone();
            }
            
            // Reiortient the vehicle in that direction
            vehControl.updateVehicleVelocity(divideAndFlockDirection);
            
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
            for(int i = 1; i <= NUM_DIVERGENCE_STEP_MULTIPLES; i++) {

                if(i == NUM_DIVERGENCE_STEP_MULTIPLES) lastTurn = true;

                turnDir = (turnRight ? "right" : "left");
                oppTurnDir = (turnRight ? "left" : "right");
                    
                travelDistOne = DIVERGENCE_STEP_LEN 
                        - (vehControl.getActiveTurnRadius() / FastMath.tan(FastMath.PI/6))
                        - (vehControl.getActiveTurnRadius() * FastMath.tan(FastMath.PI/6));
                
                //travelDistTwo = (i * DIVERGENCE_STEP_LEN);
                
                if(!lastTurn) {
                    travelDistTwo = (i * DIVERGENCE_STEP_LEN) 
                            - (vehControl.getActiveTurnRadius() * FastMath.tan(FastMath.PI/6))
                            - (vehControl.getActiveTurnRadius() / FastMath.tan(FastMath.PI/6));
                } else {
                    travelDistTwo = (i * DIVERGENCE_STEP_LEN);
                }

                compositeTurnSeq.add(new Pair<String,Object>("travel", travelDistOne));
                compositeTurnSeq.add(new Pair<String,Object>("turn_direction", turnDir));
                compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.TWO_PI/3));
                compositeTurnSeq.add(new Pair<String,Object>("travel", travelDistTwo));
                if(!lastTurn) {
                    compositeTurnSeq.add(new Pair<String,Object>("turn_direction", oppTurnDir));
                    compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.PI/3));
                } else {
                    compositeTurnSeq.add(new Pair<String,Object>("turn_direction", turnDir));
                    compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.TWO_PI/3));
                    compositeTurnSeq.add(new Pair<String,Object>("travel", HEX_SIDE_LEN));
                }

                turnRight = !turnRight;

            }

            // Pass the sequence to the controller
            vehControl.execCompositeTurn(compositeTurnSeq);
        
            turnSeqSet = true;
            return;
        }
            
        
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
            if(DHCP_DEBUG_ENABLED) { 
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
        
        state.storeFlagVar("broadcast", broadcast);
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        state.storeFlagVar("turnRight", turnRight);
        state.storeFlagVar("lastTurn", lastTurn);
        state.storeFlagVar("searchStarted", searchStarted);
        state.storeFlagVar("searchCompleted", searchCompleted);
        state.storeFlagVar("turnSeqSet", turnSeqSet);
        state.storeFlagVar("inPlaceTurnSolicited", inPlaceTurnSolicited);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeFloatVar("travelDistOne", travelDistOne);
        state.storeFloatVar("travelDistTwo", travelDistTwo);
        
        state.storeStringVar("turnDir", turnDir);
        state.storeStringVar("oppTurnDir", oppTurnDir);
        
        state.storeVectorVar("measuredInitMagNorth", measuredInitMagNorth);
        state.storeVectorVar("divideDirection", divideDirection);
        state.storeVectorVar("flockingDirection", flockingDirection);
        state.storeVectorVar("divideAndFlockDirection", divideAndFlockDirection);

        state.storeCompositeTurnSeq(compositeTurnSeq);
        
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
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        turnRight = state.getFlagVar("turnRight");
        lastTurn = state.getFlagVar("lastTurn");
        searchStarted = state.getFlagVar("searchStarted");
        searchCompleted = state.getFlagVar("searchCompleted");
        turnSeqSet = state.getFlagVar("turnSeqSet");
        inPlaceTurnSolicited = state.getFlagVar("inPlaceTurnSolicited");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        travelDistOne = state.getFloatVar("travelDistOne");
        travelDistTwo = state.getFloatVar("travelDistTwo");
        
        turnDir = state.getStringVar("turnDir");
        oppTurnDir = state.getStringVar("oppTurnDir");
        
        measuredInitMagNorth = state.getVectorVar("measuredInitMagNorth");
        divideDirection = state.getVectorVar("divideDirection");
        flockingDirection = state.getVectorVar("flockingDirection");
        divideAndFlockDirection = state.getVectorVar("divideAndFlockDirection");

        compositeTurnSeq = state.getCompositeTurnSeq();

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}

