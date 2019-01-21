/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.selforg;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import swim.sim.SimulationMode;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class InitialSOAlgorithm extends SelfOrgAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    // FLAGS
    private final boolean SO_DEBUG_ENABLED = true;
    
    // ALGORITHM PARAMETERS
    private float SPEED_SYNC_PERIOD_LENGTH = 150,  // was 5000 for all runs
                  CURVATURE_COEFF = 0.01f; //-1.15f;
    // =========================================================================
    // Algorithm variables
    // =========================================================================
    private boolean reorTowMagNorthStarted = false,
                    initAlignmentReached = false,
                    initReorientNeeded = true,
                    reorientEnded = false,
                    speedSyncPeriod = false,
                    flockStarted = false;
    
    private Vector3f measuredInitMagNorth;
    private long initReorientStartTime;
    
    private float maxReorientTime, timeSpentInReorient,
                  maxReorientPathLength;
    private long slowdownStartTime, speedSyncPeriodStartTime;
    float unitySpeed;
    // =========================================================================
    
    public InitialSOAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        unitySpeed = (vehControl.getMaxVehicleSpeed() + vehControl.getMinVehicleSpeed())/2;
    }

    @Override
    protected void selfOrganize() {
         
        if(SO_DEBUG_ENABLED) {
            System.out.println("SO update called ...");
        }

        
        
        // Orientation not started yet
        if(!reorTowMagNorthStarted) {
            
            if(SO_DEBUG_ENABLED) {
                System.out.println("Reorientation just started ...");
            }

            measuredInitMagNorth = vehControl.measureMagNorthDir().mult(vehControl.getVehicleSpeed());                       

            initReorientStartTime = sim.getCurrentTick();
            vehControl.updateVehicleVelocity(measuredInitMagNorth);
            reorTowMagNorthStarted = true;

            if( vehControl.advancedReorientAlgEnabled() ) {
                maxReorientPathLength = FastMath.PI * vehControl.getActiveTurnRadius();
            } else {
                maxReorientPathLength = 2* (1+FastMath.PI+((0.9999999f*FastMath.HALF_PI)/2)) * vehControl.getActiveTurnRadius();
            }

            if(!vehControl.turnNeeded() || vehControl.enhanReorAlgTookEffect()) {
                // If the vehicle is already aligned in a direction very close to the
                // measured magnetic north, it should slow down to allow other vehicles
                // with less alignment to align before starting to apply flocking. The 
                // slowdown will take place for a period equal to time spent to execute
                // the longest turn sequence at minimum vehicle speed. This is: 
                // Original VRA: 2( 1 + Pi + theta/2 )r_min/v_min. We take theta = 89.99
                // Advanced VRA: 2*Pi*r_min/v_min

                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(vehControl.getMinVehicleSpeed()));

            }

        // Started, finished, has not been marked finished yet
        } else if(reorTowMagNorthStarted && !vehControl.compositeTurnInProgress() && !reorientEnded) {

            if(SO_DEBUG_ENABLED) {
                System.out.println("Reorientation just finished ...");
            }
            
            timeSpentInReorient = sim.getCurrentTick() - initReorientStartTime;

            // Mark finished
            reorientEnded = true;

            /* Initial reoientation allowance time should be based on the time needed by a
               vehicle to perform the most complex turn sequence at minimum vehicle speed.
               This is equal to path length/vehicle minimum speed. Consider adding this later
               ... for now, the time will be selected by trial and error for a given swarm size.
               Note that without the use of that allowance, even if a vehicle fully reorients
               towards the magnetic north, it can lose that orientation if flocking is applied
               too early while neighbor vehicles are still turning to reoient and it tries to
               flock with them!
            */

//                        //First trial ...
//                        float percentOfFullSpeed = FastMath.log((timeSpentInReorient/maxReorientTime)*100) / FastMath.log(maxReorientTime);
//                        //Second trial
//                        float mu = 128;
//                        float x = timeSpentInReorient/maxReorientTime;
            
            float x = vehControl.getReorientPathLength()/maxReorientPathLength;
//                        float percentOfFullSpeed = FastMath.log(1 + (mu * x))/FastMath.log(1 + mu);

            if(SO_DEBUG_ENABLED) {
                System.out.println("======================================");
                System.out.println(vehControl.getVehicleName() + ": x: " + x);
            }

            if(x == 0) {
                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(vehControl.getMinVehicleSpeed()));
            } else {
                // Normalized tunable half sigmoid function
                // (x * k)/(1 + k - x), k = CURVATURE_COEFF
                float percentOfFullSpeed = (x * CURVATURE_COEFF)/(1 + CURVATURE_COEFF - x);
                
                //float percentOfFullSpeed = x;
                
                if(SO_DEBUG_ENABLED) {
                    System.out.println("Percent Of Full Speed: " + percentOfFullSpeed);
                }
                
                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(percentOfFullSpeed * vehControl.getMaxVehicleSpeed()));
                vehControl.limitVehicleSpeed();
                if(SO_DEBUG_ENABLED) {
                    System.out.println(vehControl.getVehicleName() + ": Percent of full speed: " + percentOfFullSpeed);
                }
            } 

            if(SO_DEBUG_ENABLED) {
                System.out.println(vehControl.getVehicleName() + ": Speed after reorientation: " + vehControl.getActiveVelocity().length());
            }

            speedSyncPeriod = true;
            speedSyncPeriodStartTime = sim.getCurrentTick();

        } else if(speedSyncPeriod) {    
            
            if(SO_DEBUG_ENABLED) {
                System.out.println("Speed sync ...");
            }
            
            // New ====================
            int northVehCount = vehControl.getSensedNorthVehiclesList().size();
            int southVehCount = vehControl.getSensedSouthVehiclesList().size();

            if( (northVehCount == 0 && southVehCount == 0) || northVehCount != 0 && southVehCount != 0) {
                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(unitySpeed));
            } else if( northVehCount == 0 ) {
                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(vehControl.getMinVehicleSpeed()));
            } else if( southVehCount == 0 ) {
                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(vehControl.getMaxVehicleSpeed()));
            }
            // New ====================
            
            System.out.println("Ticks elapsed: "+(sim.getCurrentTick() - speedSyncPeriodStartTime));
            
            if(sim.getCurrentTick() - speedSyncPeriodStartTime > SPEED_SYNC_PERIOD_LENGTH) {
                
                
                vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(unitySpeed));
                flockStarted = true;
                speedSyncPeriod = false;
            }

        } else if(flockStarted){
            
            if(SO_DEBUG_ENABLED) {
                System.out.println("Flocking ...");
            }
            
            if( vehControl.isSimModeEnabled(SimulationMode.INIT_SELF_ORG_MODE) ) {
                 vehControl.solicitFlockingAlgRuleUpdate();
            } else {
                
                // I am not sure why I didn't want flocking for the advanced version
                // of the turning algorithm!!
                if( !vehControl.advancedReorientAlgEnabled() ) {
                    vehControl.solicitFlockingAlgRuleUpdate();
                } else {
                    this.stop();    // Algorithm completed!
                    //if(SO_DEBUG_ENABLED) {
                //        System.out.println("SO Algorithm just completed.");
                    //}
                }
            }

        }
        
    }

    public long getExecutionTime() {
        if(stopped) {
            return stopTime - startTime;
        } else {
            if(SO_DEBUG_ENABLED) { 
                System.out.println("Algorithm is still running!");
            }
            return -1;
        }
    }
    
}
