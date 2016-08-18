/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.flocking;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.DebugUtils;

/**
 *
 * @author Sherif
 */
public class ModifiedReynolds extends FlockingAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean MRF_DEBUG_ENABLED = false;
    private final float INTER_UPDATE_TIME = 10;
    private float COHESION_WEIGHT = 0.2f,//0.5f,//0.9f,
                  SEPARATION_WEIGHT = 0.1f,//0.2f,//0.5f,
                  ALIGNMENT_WEIGHT = 0.7f;//0.3f;//0.99f; //0.01f;
    
    private final ColorRGBA COHESION_DIR_COLOR = ColorRGBA.Red,
                            SEPARATION_DIR_COLOR = ColorRGBA.Cyan,
                            ALIGNMENT_DIR_COLOR = ColorRGBA.Magenta;
    
    private final boolean FLOCKING_DEBUG_ENABLED = false;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private long lastFlockingUpdate;
    private float cohVecLength, sepVecLength, alignVecLength;   
    private int flockingStartTime;
    private boolean paused = false;
    // =========================================================================   
    
    public ModifiedReynolds(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        //@TODO: Improve in the future. Currently, this is manually changed to
        // save time, make it automated and adaptive
//        if(Simulator.bulkSimActive()) {
//            
//            if( sim.getRunNumber() <= 8 ) {
//                ALIGNMENT_WEIGHT = 0.1f;
//                SEPARATION_WEIGHT = 0.1f * (float)sim.getRunNumber();
//            } else if( sim.getRunNumber() > 8 && sim.getRunNumber() <= 15 ) {
//                ALIGNMENT_WEIGHT = 0.2f;
//                SEPARATION_WEIGHT = 0.1f * (float)(sim.getRunNumber() - 8);
//            } else if( sim.getRunNumber() > 15 && sim.getRunNumber() <= 21 ) {
//                ALIGNMENT_WEIGHT = 0.3f;
//                SEPARATION_WEIGHT = 0.1f * (float)(sim.getRunNumber() - 15);
//            } else if( sim.getRunNumber() > 21 && sim.getRunNumber() <= 26 ) {
//                ALIGNMENT_WEIGHT = 0.4f;
//                SEPARATION_WEIGHT = 0.1f * (float)(sim.getRunNumber() - 21);
//            } else if( sim.getRunNumber() > 26 && sim.getRunNumber() <= 30 ) {
//                ALIGNMENT_WEIGHT = 0.5f;
//                SEPARATION_WEIGHT = 0.1f * (float)(sim.getRunNumber() - 26);
//            } else if( sim.getRunNumber() > 30 && sim.getRunNumber() <= 33 ) {
//                ALIGNMENT_WEIGHT = 0.6f;
//                SEPARATION_WEIGHT = 0.1f * (float)(sim.getRunNumber() - 30);
//            } else if( sim.getRunNumber() > 33 && sim.getRunNumber() <= 35 ) {
//                ALIGNMENT_WEIGHT = 0.7f;
//                SEPARATION_WEIGHT = 0.1f * (float)(sim.getRunNumber() - 33);
//            } else {
//                ALIGNMENT_WEIGHT = 0.8f;
//                SEPARATION_WEIGHT = 0.1f;
//            }
//            COHESION_WEIGHT = 1.0f - (ALIGNMENT_WEIGHT + SEPARATION_WEIGHT);
//            //System.out.println(sim.getNumSimRuns());
//            //float increment = 1.0f/sim.getNumSimRuns();
//            //float currWeightVal = sim.getRunNumber() * increment;
//            //COHESION_WEIGHT = currWeightVal;
//            //SEPARATION_WEIGHT = currWeightVal;
//            //ALIGNMENT_WEIGHT = currWeightVal;
//            //System.out.println("COHESION WEIGHT: "+COHESION_WEIGHT);
//            //System.out.println("ALIGNMENT WEIGHT: "+ALIGNMENT_WEIGHT);
//            //System.out.println("SEPARATION WEIGHT: "+SEPARATION_WEIGHT);
//            System.out.println("(A, S, C): " + ALIGNMENT_WEIGHT+", "+SEPARATION_WEIGHT+", "+COHESION_WEIGHT);
//        }
    }
    
    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        lastFlockingUpdate = startTime;
        started = true;
    }

    @Override
    protected void flock() {
        
        if( paused ) return;
        
        //if(System.currentTimeMillis() - lastFlockingUpdate > INTER_UPDATE_TIME) {
            
            Vector3f flockDirection = calculateFlockDirection();

            //System.out.println("Active velocity: "+vehActiveVel);
            //System.out.println("Flocking Vector: "+combinedDir);

            // Flip a coin to decide whether to follow neighbors or not
            //if(Math.random() >= 0.9) {
            if( flockDirection == null ) {
                return;
            } else {
                flockingStartTime = sim.getCurrentTick();
                vehControl.updateVehicleVelocity(flockDirection);
            }
            //}

            lastFlockingUpdate = System.currentTimeMillis();
        //}
    }
    
    public Vector3f calculateFlockDirection() {
        
        Vector3f avgPositionDir = calcAvgNeighPosDirection();
        Vector3f separationDir = calcSeparationDirection();
        Vector3f alignmentDir = calcAlignmentDirection();
        
//        System.out.println(avgPositionDir.toString());
//        System.out.println(separationDir.toString());
//        System.out.println(alignmentDir.toString());

        if(avgPositionDir == null ||
           separationDir  == null || 
           alignmentDir   == null) {
            //System.out.println("At least one is null");
            return null;
        }

        Vector3f vehActiveVel = vehControl.getCurrentVehicleVelocity();

        //Vector3f combinedDir = new Vector3f(0,0,0);
        Vector3f combinedDir = vehActiveVel.clone();
        combinedDir = combinedDir.add(avgPositionDir.clone().mult(COHESION_WEIGHT));
        combinedDir = combinedDir.add(separationDir.clone().mult(SEPARATION_WEIGHT));
        combinedDir = combinedDir.add(alignmentDir.clone().mult(ALIGNMENT_WEIGHT));
        
        return combinedDir;
        
    }
    
    private Vector3f calcAvgNeighPosDirection() {
        
        // Get vehicles in sensing range
        String[] sensedVehicles = vehControl.getSensedVehiclesList();
        
        //System.out.println("Sensed Vehicles Count: "+sensedVehicles.length);
        
        // Hold current neighbor's estimated position
        Vector3f estNeighborPos;

        // New calculated average position
        Vector3f avgPos = new Vector3f(0, 0, 0);

        if(sensedVehicles != null && sensedVehicles.length > 0) {

            // Calculate average position
            for(String neigh : sensedVehicles) {
                estNeighborPos = vehControl.simpleEstNeighborPos(neigh);
                avgPos = avgPos.add(estNeighborPos);
                //estNeighborVel = simpleEstNeighborVel(neigh);
                //System.out.print(neigh+", ");
            }
            avgPos = avgPos.mult(1/sensedVehicles.length);
            
            // To avoid the case where avg position is equal to vehicle's position,
            // resulting a choDir pointing to zero
            if(avgPos.distance(Vector3f.ZERO) < 0.01) {
                return null;
            }
            
            // Estimate vehicle position (estimatedSelfPos)
            Vector3f estimatedVehPos = vehControl.getCurrentVehiclePosition();
            
            
            Vector3f cohDir = avgPos.subtractLocal(estimatedVehPos);
            
            cohVecLength = cohDir.length();
            
            //System.out.print("Distance from average neighbor position: "+cohDir.length());
            
            if(FLOCKING_DEBUG_ENABLED) {
                DebugUtils.plotArrow( vehControl.getPhysicsVehicle().getPhysicsLocation(), cohDir, COHESION_DIR_COLOR, 1f, sim);
            }
            
            // Find the direction the vehicle should move to (to reach avg position)
            return cohDir;
            //System.out.println();
        }
        
        return null;
        
    }
    
    private Vector3f calcSeparationDirection() {

        // Get vehicles in sensing range
        String[] sensedVehicles = vehControl.getSensedVehiclesList();
        
        // Hold current neighbor's estimated position
        Vector3f estNeighborPos;

        // Will hold the separation direction
        Vector3f sepDir = new Vector3f(0, 0, 0);

        if(sensedVehicles != null && sensedVehicles.length > 0) {
            
            // Estimate vehicle position (estimatedSelfPos)
            Vector3f estimatedVehPos = vehControl.getCurrentVehiclePosition();
            
            // Build separation direction vector
            for(String neigh : sensedVehicles) {
                estNeighborPos = vehControl.simpleEstNeighborPos(neigh);
                sepDir = sepDir.add(estimatedVehPos.subtract(estNeighborPos));
                //estNeighborVel = simpleEstNeighborVel(neigh);
                //System.out.print(neigh+", ");
            }
            sepDir = sepDir.mult(1/sensedVehicles.length);
            
            sepVecLength = sepDir.length();

            if(FLOCKING_DEBUG_ENABLED) {
                DebugUtils.plotArrow( vehControl.getPhysicsVehicle().getPhysicsLocation(), sepDir, SEPARATION_DIR_COLOR, 1f, sim);
            }
            
            // Find the direction the vehicle should move to travel away from neighbors
            return sepDir;
            //System.out.println();
        }
        
        return null;        
        
    }
    
    private Vector3f calcAlignmentDirection() {

        // Get vehicles in sensing range
        String[] sensedVehicles = vehControl.getSensedVehiclesList();
        
        // Hold current neighbor's estimated velocity
        Vector3f estNeighborVel;

        // Will hold the alignment direction
        Vector3f alignDir = new Vector3f(0, 0, 0);

        if(sensedVehicles != null && sensedVehicles.length > 0) {
            
            // Build alignment direction vector
            for(String neigh : sensedVehicles) {
                estNeighborVel = vehControl.simpleEstNeighborVel(neigh);
                alignDir = alignDir.add(estNeighborVel);
                //
                //System.out.print(neigh+", ");
            }
            alignDir = alignDir.mult(1/sensedVehicles.length);  
            
            alignVecLength = alignDir.length();

            if(FLOCKING_DEBUG_ENABLED) {
                DebugUtils.plotArrow( vehControl.getPhysicsVehicle().getPhysicsLocation(), alignDir, ALIGNMENT_DIR_COLOR, 1f, sim);
            }
           
            // Find the direction the vehicle should move to travel away from neighbors
            return alignDir;
            //System.out.println();
        }
        
        return null;                
        
    }
    
    public long getLastFlockingUpdateTime() {
        return lastFlockingUpdate;
    }
    
    public long getExecutionTime() {
        if(stopped) {
            return stopTime - startTime;
        } else {
            if(MRF_DEBUG_ENABLED) { 
                System.out.println("Algorithm is still running!");
            }
            return -1;
        }
    }
    
    public float getSepVecLength() {
        return sepVecLength;
    }
    
    public float getCohVecLength() {
        return cohVecLength;
    }
    
    public float getAlignVecLength() {
        return alignVecLength;
    }
    
    public int getFlockingStartTime() {
        return flockingStartTime;
    }
    
    @Override
    protected void saveState() {
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        state.storeIntVar("flockingStartTime", flockingStartTime);
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        state.storeLongVar("lastFlockingUpdate", lastFlockingUpdate);
        
        state.storeFloatVar("COHESION_WEIGHT", COHESION_WEIGHT);
        state.storeFloatVar("SEPARATION_WEIGHT", SEPARATION_WEIGHT);
        state.storeFloatVar("ALIGNMENT_WEIGHT", ALIGNMENT_WEIGHT);
        state.storeFloatVar("cohVecLength", cohVecLength);
        state.storeFloatVar("sepVecLength", sepVecLength);
        state.storeFloatVar("alignVecLength", alignVecLength);
        
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
        flockingStartTime = state.getIntVar("flockingStartTime");
        
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        lastFlockingUpdate = state.getLongVar("lastFlockingUpdate");

        COHESION_WEIGHT = state.getFloatVar("COHESION_WEIGHT");
        SEPARATION_WEIGHT = state.getFloatVar("SEPARATION_WEIGHT");
        ALIGNMENT_WEIGHT = state.getFloatVar("ALIGNMENT_WEIGHT");
        cohVecLength = state.getFloatVar("cohVecLength");
        sepVecLength = state.getFloatVar("sepVecLength");
        alignVecLength = state.getFloatVar("alignVecLength");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
}
