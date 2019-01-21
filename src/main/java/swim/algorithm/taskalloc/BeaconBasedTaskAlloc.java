package swim.algorithm.taskalloc;

import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
//import com.oat.utils.ArrayUtils;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map.Entry;
import swim.algorithm.common.TargetConfig;
import swim.core.motion.CompositeTurnSequence;
import swim.core.motion.OrientMode;
import swim.sim.IntegrationMode;
import swim.sim.SimulationMode;
import swim.sim.Simulator;
import swim.sim.TaskAllocMode;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class BeaconBasedTaskAlloc extends TaskAllocAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean 
                          USE_SINGLE_BARREL_GROUP,
                          USE_MULTIPLE_BARREL_GROUPS,
                          USE_SHIP;
    
    private boolean DEBUG_ENABLED = false;
    
    private final int NUM_OF_SWEEPS = 6;
    private final float SWEEP_WIDTH;
    private final float SWEEP_LENGTH = 30;
    private final int LARGE_TARGET_COUNT = 10;
    
    private final int MIN_ACCEPT_AVG_VEH_COUNT = 3,
                      AVERAGING_PERIOD = 2000;
    
    // =========================================================================
    // Algorithm variables
    // =========================================================================
    private boolean firstEntered = true, joinedABeacon = false;
    private boolean turnRight = true, firstSweep = true;
    private String turnDir = "left", lastTurnDir;
      
    private int numTurns = NUM_OF_SWEEPS - 1,
                turnCounter = 0;
    
    private float travelDist;
    
    private boolean sweepStarted = false, sweepCompleted = false,
            inPlaceTurnSolicited = false, isBeacon = false,
            decidedToSweep = false,
            didSufficientWork = false;
    
    private CompositeTurnSequence compTurnSeq;
    
    private Deque<UWVehicleControl> nearbyBeacons;
    
    private float averageNearbyVehCount = 0,
                  elapsedTime = 0,
                  periodVehicleCount = 0;
    // =========================================================================
    
    public BeaconBasedTaskAlloc(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        if( sim.getStateManager().getState(IntegrationMode.class) != null && 
                sim.getStateManager().getState(IntegrationMode.class).isEnabled() ) {
            
            USE_SHIP = sim.getStateManager().getState(IntegrationMode.class).usingTargetConfiguration(TargetConfig.SHIP);
            USE_SINGLE_BARREL_GROUP = sim.getStateManager().getState(IntegrationMode.class).usingTargetConfiguration(TargetConfig.SINGLE_BARREL_GROUP);
            USE_MULTIPLE_BARREL_GROUPS = sim.getStateManager().getState(IntegrationMode.class).usingTargetConfiguration(TargetConfig.MULTIPLE_BARREL_GROUPS);
        
        } else if( sim.getStateManager().getState(TaskAllocMode.class) != null && 
                sim.getStateManager().getState(TaskAllocMode.class).isEnabled() ) {
            
            USE_SHIP = sim.getStateManager().getState(TaskAllocMode.class).usingTargetConfiguration(TargetConfig.SHIP);
            USE_SINGLE_BARREL_GROUP = sim.getStateManager().getState(TaskAllocMode.class).usingTargetConfiguration(TargetConfig.SINGLE_BARREL_GROUP);
            USE_MULTIPLE_BARREL_GROUPS = sim.getStateManager().getState(TaskAllocMode.class).usingTargetConfiguration(TargetConfig.MULTIPLE_BARREL_GROUPS);
        
        } else {
            USE_SHIP = true;
            USE_SINGLE_BARREL_GROUP = false;
            USE_MULTIPLE_BARREL_GROUPS = false;
        }
                                  
        SWEEP_WIDTH = 2 * UWVehicleControl.getMinTurnRadius();
        nearbyBeacons = new LinkedList<UWVehicleControl>();
       
    }

    @Override
    protected void takeAction(float tpf) {
        
//        if( vehControl.sequenceBreakWasSolicited() || inPlaceTurnSolicited ) {
//            DEBUG_ENABLED = true;
//        }
  
        checkAmountOfWorkDone();
        if( DEBUG_ENABLED ) System.out.println("BBTA: Checked amount of work ...");
        
        if( (vehControl.getResidualEnergy() < vehControl.getFailureEnergy() || didSufficientWork) && 
                !vehControl.isSimModeEnabled(SimulationMode.TASK_ALLOC_MODE) ) {
            
            // Some cleaning
            vehControl.setTargetFound(false);
            vehControl.solicitTurnSeqBreak(false);
            
            stop();
            if( DEBUG_ENABLED ) System.out.println("BBTA: stopped ...");
            return;
        }
        
        // Keep track of nearby vehicle counts. If very low for a period,
        // break the in-place turn and start over
        if(inPlaceTurnSolicited) {
            if( DEBUG_ENABLED ) System.out.println("BBTA: In-place turn entered ...");
            ++elapsedTime;
            
            if( DEBUG_ENABLED && elapsedTime % 100 == 0 ) {
                System.out.println("BBTA: " + vehControl.getVehicleName()+": Tracking nearby vehicles ... elapsed time: " + elapsedTime);
            }
            
            periodVehicleCount += vehControl.getSensedNeighCount();
            
            if(elapsedTime >= AVERAGING_PERIOD) {
                averageNearbyVehCount = (int)Math.ceil(periodVehicleCount/AVERAGING_PERIOD);
                if( DEBUG_ENABLED ) System.out.println("BBTA: Average per-tick vehicle count: "+averageNearbyVehCount);
                if( averageNearbyVehCount < MIN_ACCEPT_AVG_VEH_COUNT ) {
                    vehControl.solicitTurnSeqBreak(true);
                    vehControl.forceCompTurnSeqReset();
                    vehControl.cancelInPlaceTurn();
                    vehControl.updateTurningParams(UWVehicleControl.getMinTurnRadius());
                    vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                    inPlaceTurnSolicited = false;
                    
                    sim.removeFromTargBeaconingAUVsMap(vehControl.getVehicleName());
                } else {
                    //System.out.println("Came here (2) ...");
                }
            }
            return;
        }
        
        if( vehControl.sequenceBreakWasSolicited() && !vehControl.compositeTurnInProgress() ) {
            if( DEBUG_ENABLED ) System.out.println("BBTA: cleanup ...");
            sweepStarted = false;
            sweepCompleted = false;
            isBeacon = false;
            decidedToSweep = false;
            vehControl.setTargetSweepingModeActive(false);
            vehControl.solicitTurnSeqBreak(false);
            vehControl.forceCompTurnSeqReset();
            vehControl.setTargetFound(false);
            
            OrientMode savedMode = vehControl.getSavedOrientMode();
            if( savedMode != null ) {
                vehControl.enableReorientationMode(savedMode);
            }
            
            //System.out.println("Came here (3) ...");
        }

        // When this is entered for the first time, it means that a target has 
        // already been encountered (in the context of the full mission)
        if( firstEntered ) {
            if( DEBUG_ENABLED ) System.out.println("BBTA: sequence building ...");
            
            // Prepare the turn sequence
            compTurnSeq = new CompositeTurnSequence();
            
            // Travel a distance proportional to the number of sweeps to 
            // be done
            //compTurnSeq.add("travel", 2 * SWEEP_WIDTH * NUM_OF_SWEEPS);
            
            // Turn right to start the sweep sequence
            compTurnSeq.add("turn_direction", "right");
            compTurnSeq.add("turn_angle", FastMath.HALF_PI);
            
            // Build the sweep sequence
            for(int i = 1; i <= NUM_OF_SWEEPS; i++) {

                turnDir = (turnRight ? "right" : "left");
                    
                if(firstSweep) {
                    travelDist = SWEEP_LENGTH/2; 
                    firstSweep = false;
                } else {
                    travelDist = SWEEP_LENGTH;
                }
               
                compTurnSeq.add("travel", travelDist);

                if(i < NUM_OF_SWEEPS) {
                    compTurnSeq.add("turn_direction", turnDir);
                    compTurnSeq.add("turn_angle", FastMath.PI);
                    turnRight = !turnRight;
                } else {
                    turnRight = !turnRight;
                    turnDir = (turnRight ? "right" : "left");
                    compTurnSeq.add("turn_direction", turnDir);
                    compTurnSeq.add("turn_angle", FastMath.HALF_PI);
                    
                    if(NUM_OF_SWEEPS % 2 == 0) {    // Do further investigation here
                        travelDist = 3 * (NUM_OF_SWEEPS - 1) * SWEEP_WIDTH;
                    } else {
                        travelDist = 3 * NUM_OF_SWEEPS * SWEEP_WIDTH; 
                    }
                    
                    compTurnSeq.add("travel", travelDist);
                    compTurnSeq.add("turn_direction", turnDir);
                    compTurnSeq.add("turn_angle", FastMath.HALF_PI);
                    compTurnSeq.add("travel", 0.5f * SWEEP_LENGTH);
                    lastTurnDir = turnDir;
                }                
            }
            
            firstEntered = false;

        }
        
        if( sweepStarted && !vehControl.compositeTurnInProgress() && !sweepCompleted ) {
            if( DEBUG_ENABLED ) System.out.println("BBTA: sweep completed ...");
            sweepCompleted = true;
        }
        
        if(sweepCompleted && !vehControl.sequenceBreakWasSolicited() && !inPlaceTurnSolicited) {
            
            if( DEBUG_ENABLED ) System.out.println("BBTA: Soliciting in-place turn ...");
            
            // Time elapsed since start of beaconing behavior
            elapsedTime = 0;
            periodVehicleCount  = 0;
            averageNearbyVehCount = 0;
            
            // Disable vehicle controller target sweeping mode
            vehControl.setTargetSweepingModeActive(false);
            
            if( DEBUG_ENABLED ) {
                System.out.println("BBTA: Should start the in-place turn");
            }
            
            vehControl.solicitTurnNeeded();
            vehControl.announceCompTurnInProgress();
            vehControl.solicitInPlaceTurn(0.3f*SWEEP_LENGTH, lastTurnDir);
            
            // Prepare task descriptor list to be used by vehicle to turn task
            // indicator lights on
            ArrayList<String> sweepedTargets = vehControl.getSweepedTargList();
            
            // Pass the list to the controller
            vehControl.turnTaskIndicatorLightsOn(getTaskDescriptorList(sweepedTargets));
            
            // Add the AUV to the list beaconing AUVs
            sim.addToTargBeaconingAUVsMap(vehControl);
            
            if( DEBUG_ENABLED ) {
                vehControl.printTaskIndicatorLightNames();
            }
            
            inPlaceTurnSolicited = true;
            //vehControl.enableStateTracker();
            return;
        }
        
        // To avoid having a previously set joinedABeacon true all the time
        boolean beaconsNearby = beaconVehiclesNearby();
        if( !beaconsNearby ) {
            joinedABeacon = false;
            vehControl.limitBubbleChainJumps(false);
            vehControl.changePathDebugColor(ColorRGBA.Black);
            vehControl.changePathDebugColor(1f);
        }
        
        // Check if target has been found before doing any work
        if( vehControl.isTargetFound() || (decidedToSweep && !sweepStarted) ) {
            
            if( DEBUG_ENABLED ) System.out.println("BBTA: target found ...");
        
            // Get target name
            String closestTarget = vehControl.getClosestTargetName();
            
            // Get the list of sensed targets
            String[] nearbyTargets = vehControl.getSensedTargetsList();
            
            TargetType preferredTargType = vehControl.getPreferredTargType();
            
            // An occasional selection of other types
            boolean randSelProbCondAchieved = FastMath.nextRandomFloat() > 0.75;
            boolean isPreferredTargType = sim.getTargetType(closestTarget).equals(preferredTargType);
            boolean noVehiclesAround = vehControl.getSensedVehiclesList().length == 0;
                    
            if( !decidedToSweep && ( isPreferredTargType || noVehiclesAround || randSelProbCondAchieved || joinedABeacon ) ) {
                
                if( DEBUG_ENABLED ) System.out.println("BBTA: Targets sensed (1) ...");

                if( !closestTarget.equals("") && !vehControl.getProcessedTargetsList().contains(closestTarget) ) {
                    vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                    
                    if( isPreferredTargType ) {
                        vehControl.setTypeOfTargToProcess(preferredTargType);
                        if( DEBUG_ENABLED ) System.out.println("BBTA: Preferred type selected (1) ( "+vehControl.getVehicleName()+ "):" + preferredTargType);
                    } else {
                        TargetType selType = preferredTargType;
                        for( String target : nearbyTargets ) {
                            selType = sim.getTargetType(target);
                            if( !selType.equals(preferredTargType) ) break;
                        }
                        vehControl.setTypeOfTargToProcess(selType);
                         if( DEBUG_ENABLED ) System.out.println("BBTA: Another type selected (1) ( "+vehControl.getVehicleName()+ "):" + selType);
                    }
                    
                    if( DEBUG_ENABLED ) System.out.println("BBTA: Exploitation solicitation (1): "+vehControl.getVehicleName());
                } else {
                    vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                    if( DEBUG_ENABLED ) System.out.println("BBTA: Next target search solicitation (1): "+vehControl.getVehicleName());
                }
                
                return;
            }
            
            boolean vehiclesAreAround = vehControl.getSensedVehiclesList().length > 0;
            boolean lowProbCondAchieved = FastMath.rand.nextFloat() < 0.05f;
            randSelProbCondAchieved = FastMath.nextRandomFloat() > 0.5f;
            
            // No neighbors and no beacon vehicles around
            if( 
                (decidedToSweep && !sweepStarted) ||
                ( ( noVehiclesAround || (vehiclesAreAround && lowProbCondAchieved) ) && !beaconsNearby && randSelProbCondAchieved )
              ) {
                
                if( DEBUG_ENABLED ) System.out.println("BBTA: No neighbors and no beacon vehicles around ...");
                
                // Make sure proper orientation mode is active
                vehControl.enableReorientationMode(OrientMode.ADVANCED_REORIENT_ALG);
                
                // Decide whether to continue traveling or start sweeping
                if( vehControl.getSensedTargetsCount() > 0 ) {
                    //System.out.println("Number of sensed targets: " + vehControl.getSensedTargetsCount());
                    if( DEBUG_ENABLED ) System.out.println("BBTA: Should continue straight ...");
                    vehControl.solicitStraightTravel(true);
                    decidedToSweep = true;
                    return;
                } else {
                    if( DEBUG_ENABLED ) System.out.println("BBTA: straight travel reset (1) ...");
                    vehControl.solicitStraightTravel(false);
                }
                
                // Start sweeping
                if( !sweepStarted ) {
                    
                    if( DEBUG_ENABLED ) System.out.println("BBTA: Sweep not started yet ...");
                    
                    if( USE_SINGLE_BARREL_GROUP || USE_MULTIPLE_BARREL_GROUPS ) {
                        // Notify vehicle control of a required turn-sequence execution
                        vehControl.solicitTurnNeeded();
                        vehControl.announceCompTurnInProgress();

                        // Pass the sequence to the controller
                        vehControl.execCompositeTurn(compTurnSeq);
                        if( DEBUG_ENABLED ) System.out.println("BBTA: Should start the sweep now!");
                    } 
                    
                    // In case of single target (ship), sweeping is not needed

                    // Enable vehicle controller target sweeping mode
                    vehControl.setTargetSweepingModeActive(true);

                    // Mark sweep as 'started'
                    sweepStarted = true;
                    
                    // Mark AUV as a beacon vehicle
                    isBeacon = true;
                }
            } 
            
            if( beaconsNearby && !isBeacon ) {
                
                if( DEBUG_ENABLED ) System.out.println("BBTA: Checking for beacons ...");
                
                // Loop over nearby beacons to check which to join (if at all)
                UWVehicleControl beacon;
                boolean beaconIndicatesPreferType,
                        largeTargCountAnnounced,
                        mediumTargCountAnnounced,
                        probJoiningMedCountAchieved;
                
                while( !nearbyBeacons.isEmpty() ) {
                    
                    beacon = nearbyBeacons.pop();
                    
                    beaconIndicatesPreferType = beacon.taskIndicatorLightOn(getTaskDescriptor(preferredTargType));
                    largeTargCountAnnounced = beacon.taskIndicatorLightOn(TaskDescriptor.LARGE_NUM_TARGS);
                    mediumTargCountAnnounced = beacon.taskIndicatorLightOn(TaskDescriptor.MEDIUM_NUM_TARGS);
                    probJoiningMedCountAchieved = FastMath.rand.nextFloat() > 0.25f;
                    
                    if( 
                        beaconIndicatesPreferType ||
                        ( largeTargCountAnnounced || (mediumTargCountAnnounced && probJoiningMedCountAchieved) ) 
                      ) {
                            
                        String[] sensedTargets;
                        int numTargets;

                        sensedTargets = vehControl.getSensedTargetsList();
                        numTargets = sensedTargets.length;

                        if( numTargets > 0 ) {
                            if( DEBUG_ENABLED ) System.out.println("BBTA: Targets sensed (2) ...");
                            vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                            
                            if( beaconIndicatesPreferType ) {
                                vehControl.setTypeOfTargToProcess(preferredTargType);
                                if( DEBUG_ENABLED ) System.out.println("BBTA: Preferred type selected (2) ( "+vehControl.getVehicleName()+ "):" + preferredTargType);
                            } else {
                                TargetType selType = preferredTargType;
                                for( String target : nearbyTargets ) {
                                    selType = sim.getTargetType(target);
                                    if( !selType.equals(preferredTargType) ) break;
                                }
                                vehControl.setTypeOfTargToProcess(selType);
                                if( DEBUG_ENABLED ) System.out.println("BBTA: Another type selected (2) ( "+vehControl.getVehicleName()+ "):" + selType);
                            }
                            
                            if( DEBUG_ENABLED ) System.out.println("BBTA: Exploitation solicitation (2): "+vehControl.getVehicleName());

                            joinedABeacon = true;
                            vehControl.limitBubbleChainJumps(true);
                            vehControl.changePathDebugColor(ColorRGBA.Red);
                            vehControl.changePathDebugColor(3f);
                            break;
                        } else {
                            vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                            vehControl.generateAtTargetRandTurn();
                            if( DEBUG_ENABLED ) System.out.println("BBTA: At-target random turn ...");
                            // No break here so that all other beacons may be checked if 
                            // no suitable beacon has been found so far
                        }
                            
                    }
                }
                
                // Used in case the loop is broken before all beacons are poped
                if( !nearbyBeacons.isEmpty() ) nearbyBeacons.clear();
            }
        
        } else {
            
            if( DEBUG_ENABLED ) System.out.println("BBTA: target not found ...");
            
            vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
            if( vehControl.straightTravelRequested() ) {
                if( DEBUG_ENABLED ) System.out.println("BBTA: Straight travel was rest (2) ...");
                vehControl.solicitStraightTravel(false);
            }
            //System.out.println("Returned because target was not found ... should look for another");
        }

    }

    @Override
    protected boolean taskDone() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void setMaxIterations(int maxNumIter) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    private ArrayList<TaskDescriptor> getTaskDescriptorList(ArrayList<String> sweepedTargets) {
        
        ArrayList<TaskDescriptor> discriptors = new ArrayList<TaskDescriptor>();
        
        if( USE_SHIP ) {
            discriptors.add(TaskDescriptor.SINGLE_TARGET_TASK);
            return discriptors;
        }
        
        int targCounts = sweepedTargets.size();

        HashMap<TargetType, Integer> targTypeCounts = new HashMap<TargetType, Integer>(4);

        // Initialize counts to zeros
        for( TargetType targType : TargetType.values() ) {
            targTypeCounts.put(targType, 0);
        }
        
        // Fill counts for each type
        for( String target : sweepedTargets ) {
            TargetType targType = sim.getTargetType(target);
            targTypeCounts.put(targType, targTypeCounts.get(targType) + 1);
        }
        
        // Get the one with max count
        int max = 0;
        TargetType maxCountTargType = TargetType.RED_BARREL;
        
        for( Entry<TargetType, Integer> entry : targTypeCounts.entrySet() ) {
            max = Math.max( max, entry.getValue() );
            if( max == entry.getValue() ) maxCountTargType = entry.getKey();
        }
        
        // Add task descriptors
        if( targTypeCounts.get(TargetType.RED_BARREL).equals(targTypeCounts.get(TargetType.GREEN_BARREL)) && 
            targTypeCounts.get(TargetType.GREEN_BARREL).equals(targTypeCounts.get(TargetType.BLUE_BARREL)) ) {
            discriptors.add(TaskDescriptor.UNIFORM_MIXTURE_OF_TYPES);
        } else {
            switch( maxCountTargType ) {
                case RED_BARREL:
                    discriptors.add(TaskDescriptor.MOSTLY_TYPE_I_TARGS);
                    break;
                case GREEN_BARREL:
                    discriptors.add(TaskDescriptor.MOSTLY_TYPE_II_TARGS);
                    break;
                case BLUE_BARREL:
                    discriptors.add(TaskDescriptor.MOSTLY_TYPE_III_TARGS);
            }
        }
        
        if( targCounts >= 0.666f * LARGE_TARGET_COUNT ) {
            discriptors.add(TaskDescriptor.LARGE_NUM_TARGS);
        } else if( targCounts >= 0.333f * LARGE_TARGET_COUNT && targCounts < 0.666f * LARGE_TARGET_COUNT ) {
            discriptors.add(TaskDescriptor.MEDIUM_NUM_TARGS);
        } else {
            discriptors.add(TaskDescriptor.SMALL_NUM_TARGS);    
        }
        
        return discriptors;
    }
    
    private boolean beaconVehiclesNearby() {
        
        nearbyBeacons.clear();
        
        HashMap<String, UWVehicleControl> beaconAUVsMap = sim.getTargBeaconingAUVsMap();
        
        Vector3f beaconPos, vehPos;
        
        for( Entry<String, UWVehicleControl> entry : beaconAUVsMap.entrySet() ) {
            
            beaconPos = entry.getValue().getPhysicsVehicle().getPhysicsLocation();
            vehPos = vehControl.getPhysicsVehicle().getPhysicsLocation();
            
            if( beaconPos.distance(vehPos) < SWEEP_LENGTH ) {
                nearbyBeacons.push(entry.getValue());
            }
        }
        
        return !nearbyBeacons.isEmpty();
        
    }
    
    private TaskDescriptor getTaskDescriptor(TargetType type) {
        switch(type) {
            case RED_BARREL:
                return TaskDescriptor.MOSTLY_TYPE_I_TARGS;
            case GREEN_BARREL:
                return TaskDescriptor.MOSTLY_TYPE_II_TARGS;
            case BLUE_BARREL:
                return TaskDescriptor.MOSTLY_TYPE_III_TARGS; 
            case SHIP:
                return TaskDescriptor.SINGLE_TARGET_TASK;
        }
        return null;
    }
    
    private void checkAmountOfWorkDone() {
        
        int numProcessedTargs = vehControl.getProcessedTargetsList().size(); 
        if( numProcessedTargs >= vehControl.getPresetGoalTargProcessings() ) {
            didSufficientWork = true;
        }
        
        float percentage = 100*((float)numProcessedTargs)/vehControl.getPresetGoalTargProcessings();
        
        if( DEBUG_ENABLED && percentage >= 100 ) {
            System.out.println(vehControl.getVehicleName()+": Percent of processing goal achieved: " + percentage + "%");    
        }

    }

}
