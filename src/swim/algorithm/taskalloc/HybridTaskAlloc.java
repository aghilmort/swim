/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.taskalloc;

import com.jme3.math.FastMath;
import java.util.Arrays;
import swim.algorithm.common.TargetConfig;
import swim.sim.IntegrationMode;
import swim.sim.Simulator;
import swim.sim.TaskAllocMode;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class HybridTaskAlloc extends TaskAllocAlgorithm {
    
    private boolean DEBUG_ENABLED = false,
                    USING_SHIP = false,
                    USING_BARRELS = false;
    
    //private float NEIGHBORS_EFFECT = 
    
    private String[] sensedStaticVehicles,
                     sensedMovingVehicles,
                     sensedTargets;
    
    private float numStaticVehicles,
                  numMovingVehicles,
                  numTargets,
                  elapsedMissionTime,
                  missionTimeConstraint,
                  goalNumProcessedTargs,
                  actualNumProcessedTargs;
    
    private double eRTFunct, eRTFunct2, eRTFunct3, 
                   eRTFunct4, eRTFunct5, eRTFunct6, 
                   threshold;
            
    private float[] barrelCounts;
    private int barrelColorIndex;
    TargetType preferredTargType = TargetType.SHIP;
    
    
    public HybridTaskAlloc(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        TaskAllocMode taskAllocMode = sim.getStateManager().getState(TaskAllocMode.class);
        IntegrationMode integrationMode = sim.getStateManager().getState(IntegrationMode.class);
        
        if( taskAllocMode != null && taskAllocMode.isEnabled() ) {
            USING_SHIP = taskAllocMode.usingTargetConfiguration(TargetConfig.SHIP);
            USING_BARRELS = !USING_SHIP;
        } 
        
        if( integrationMode != null && integrationMode.isEnabled() ) {
            USING_SHIP = integrationMode.usingTargetConfiguration(TargetConfig.SHIP);
            USING_BARRELS = !USING_SHIP;
        }
        
        if( DEBUG_ENABLED && USING_BARRELS ) System.out.println("Barrels enabled.");
        if( DEBUG_ENABLED && USING_SHIP ) System.out.println("Ship enabled.");
    }

    @Override
    protected void takeAction(float tpf) {
        
        if( DEBUG_ENABLED ) System.out.println("Applying updates ...");
        
        // Get sensed vehicles' statuses
        sensedStaticVehicles = vehControl.getSensedStaticVehiclesList();
        sensedMovingVehicles = vehControl.getSensedMovingVehiclesList();
        numStaticVehicles = sensedStaticVehicles.length;
        numMovingVehicles = sensedMovingVehicles.length;
        
        if( DEBUG_ENABLED ) System.out.println("Static AUVs count:" + numStaticVehicles);
        if( DEBUG_ENABLED ) System.out.println("Moving AUVs count:" + numMovingVehicles);
        
        // Get sensed targets and do the classification below
        sensedTargets = vehControl.getSensedTargetsList();
        numTargets = sensedTargets.length;
        
        if( DEBUG_ENABLED ) System.out.println("Targets count:" + numTargets);
        
        // Get elapsed mission time and time constraint
        missionTimeConstraint = vehControl.getMissionTimeConstraint();
        elapsedMissionTime = vehControl.getElapsedMissionIntTime();
        
        if( DEBUG_ENABLED ) System.out.println("Elapsed mission time:" + elapsedMissionTime);
        if( DEBUG_ENABLED ) System.out.println("Mission time constraint:" + missionTimeConstraint);
        
        // Get goal and achieved number of target processings
        actualNumProcessedTargs = vehControl.getProcessedTargetsList().size(); 
        goalNumProcessedTargs = vehControl.getPresetGoalTargProcessings();
        
        if( DEBUG_ENABLED ) System.out.println("Number of processed targets:" + actualNumProcessedTargs);
        if( DEBUG_ENABLED ) System.out.println("Goal number of targets:" + goalNumProcessedTargs);
        
        // Condition to do anything is first to have targets in range
        if( numTargets > 0 ) {
            
            if( DEBUG_ENABLED ) System.out.println("Targets sensed ...");
            
            if( USING_BARRELS ) {
                
                barrelCounts = new float[]{0,0,0};  // reg,green,blue

               // Determine counts of different target types
                for( String target : sensedTargets ) {
                    switch( sim.getTargetType(target) ) {
                        case RED_BARREL:   ++barrelCounts[0]; break;
                        case GREEN_BARREL: ++barrelCounts[1]; break;
                        case BLUE_BARREL:  ++barrelCounts[2]; break;
                    }
                }
                
                // Check preferred target type to determine the barrel color 
                // index to use
                
                // Get vehicle's preferred target type
                preferredTargType = vehControl.getPreferredTargType();
                
                switch( preferredTargType ) {
                    case RED_BARREL:   barrelColorIndex = 0; break;
                    case GREEN_BARREL: barrelColorIndex = 1; break;
                    case BLUE_BARREL:  barrelColorIndex = 2; break;
                }

                int secondIndex = (barrelColorIndex + 1) % 3;
                int thirdIndex  = (barrelColorIndex + 2) % 3;

                // Now that we have enerything, apply the single neuron model
                
                // First ERT function: num static AUVs / num targets
                eRTFunct  = 1 - Math.exp(-numStaticVehicles/numTargets);
                
                // Second ERT function: num preferred target type / num targets
                eRTFunct2 = Math.exp(-barrelCounts[barrelColorIndex]/numTargets);
                
                // Third ERT function: elapsed time / time constraint
                eRTFunct3 = Math.exp(-elapsedMissionTime/missionTimeConstraint);
                
                // Fourth ERT function: num processed targets / goal number
                eRTFunct4 = 1 - Math.exp(-actualNumProcessedTargs/goalNumProcessedTargs);
                
                // Their average ... used for making the final decision
                threshold = (eRTFunct + eRTFunct2 + eRTFunct3 + eRTFunct4)/4;
                
                if( DEBUG_ENABLED ) System.out.println("Threshold value:" + threshold);
                
                // Fifth and sixth ERT are not used in making the decision, they
                // are used later to set type of target to process in case the
                // preferred one is not present
                eRTFunct5 = Math.exp(-barrelCounts[secondIndex]/numTargets);
                eRTFunct6 = Math.exp(-barrelCounts[thirdIndex]/numTargets);
                
                if(Math.random() > threshold) {  // Simulate the probability
                    
                    vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                    
                    TargetType selectedType;
                    
                    if( barrelCounts[barrelColorIndex] > 0 ) {
                        selectedType = TargetType.getType(barrelColorIndex);
                    } else {
                        if( barrelCounts[secondIndex] > 0 && barrelCounts[thirdIndex] > 0 ) {
                            
                            int selectedIndex;
                            double rand = Math.random();
                            
                            if( rand > eRTFunct5 && rand > eRTFunct6 ) {
                                if( barrelCounts[secondIndex] != barrelCounts[thirdIndex] ) {
                                    selectedIndex = ( barrelCounts[secondIndex] > barrelCounts[thirdIndex] ? secondIndex : thirdIndex );                                
                                } else {
                                    selectedIndex = ( FastMath.nextRandomFloat() > 0.5f ? secondIndex : thirdIndex );
                                }
                            } else if( rand > eRTFunct5 ) {
                                selectedIndex = secondIndex;
                            }  else {
                                selectedIndex = thirdIndex;
                            }
          
                            selectedType = TargetType.getType(selectedIndex);
                            
                        } else if( barrelCounts[secondIndex] > 0 ) {
                            selectedType = TargetType.getType(secondIndex);
                        } else {
                           selectedType = TargetType.getType(thirdIndex);
                        }
                    }
                    
                    vehControl.setTypeOfTargToProcess(selectedType);
                            
                    if( DEBUG_ENABLED ) System.out.println("Task alloc alg. requested: Exploit target. Type: "+selectedType);
                
                } else {
                    //System.out.println("ERT Function value: "+expThresFunction);
                    vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                    if( DEBUG_ENABLED ) System.out.println("Task alloc alg. requested: Look for another target");       
                }
            }
            
            if( USING_SHIP ) {
                
                if(numMovingVehicles == 0) {
                    eRTFunct = 1;
                } else {  
                    if( DEBUG_ENABLED ) System.out.println("Vehicles sensed ...");
                    eRTFunct = 1 - Math.exp(-numTargets/(numMovingVehicles+numStaticVehicles));
                }
                
                if(Math.random() > eRTFunct) {  // Simulate the probability
                    //System.out.println("ERT Function value: "+expThresFunction);
                    vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                    if( DEBUG_ENABLED ) System.out.println("Task alloc alg. requested: Look for another target");
                } else {
                    vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                    if( DEBUG_ENABLED ) System.out.println("Task alloc alg. requested: Exploit target");
                }
            }
  
        } else {
            vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
            //System.out.println("Looking for another target ...");
        }

        //System.out.println(Arrays.toString(vehControl.getSensedVehiclesList()));
        //System.out.println(Arrays.toString(vehControl.getSensedTargetsList()));

    }
    
    public long getExecutionTime() {
        if(stopped) {
            return stopTime - startTime;
        } else {
            System.out.println("Algorithm is still running!");
            return -1;
        }
    }

    @Override
    protected boolean taskDone() {
        return true;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {
    }
    
}
