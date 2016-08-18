/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.taskalloc;

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
public class ExpRespThresh extends TaskAllocAlgorithm {
    
    private boolean DEBUG_ENABLED = false,
                    USING_SHIP = false,
                    USING_BARRELS = false;
    
    private String[] sensedVehicles,
                     sensedTargets;
    
    private float numVehicles,
                  numTargets;
    
    private double expThresFunction, expThresFunction2, expThresFunction3;
            
    private float[] barrelCounts;
    private int barrelColorIndex;
    TargetType preferredTargType = TargetType.SHIP;
    
    
    public ExpRespThresh(UWVehicleControl vehControl, Simulator sim) {
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
        
        sensedVehicles = vehControl.getSensedVehiclesList();
        sensedTargets = vehControl.getSensedTargetsList();
        
        numVehicles = sensedVehicles.length;
        numTargets = sensedTargets.length;
        
        if( numTargets > 0 ) {
            
            if( DEBUG_ENABLED ) System.out.println("Targets sensed ...");
            
            if( USING_BARRELS ) {

                // Clear any previously set allowed target type for processing
                //vehControl.unsetTypeOfTargToProcess();
                
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

                if(numVehicles == 0) {
                    expThresFunction = 1;
                } else { 
                    expThresFunction  = 1 - Math.exp(-barrelCounts[barrelColorIndex]/numVehicles);
                }
                
                if(barrelCounts[barrelColorIndex] == 0) {
                    expThresFunction2 = 1;
                    expThresFunction3 = 1;
                } else { 
                    expThresFunction2 = 1 - Math.exp(-barrelCounts[secondIndex]/barrelCounts[barrelColorIndex]);
                    expThresFunction3 = 1 - Math.exp(-barrelCounts[thirdIndex]/barrelCounts[barrelColorIndex]);
                }

                double selectedThreshFunc;
                               
                if(expThresFunction == expThresFunction2 && expThresFunction2 == expThresFunction3) {
                    selectedThreshFunc = expThresFunction;
                } else {
                    // Priority selection
                    selectedThreshFunc = Math.max(expThresFunction, expThresFunction2);
                    selectedThreshFunc = Math.max(selectedThreshFunc, expThresFunction3);
                }
                
                if(Math.random() > selectedThreshFunc) {  // Simulate the probability
                    //System.out.println("ERT Function value: "+expThresFunction);
                    vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                    if( DEBUG_ENABLED ) System.out.println("Task alloc alg. requested: Look for another target");
                } else {
                    
                    vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                    
                    TargetType selectedType;
                    
                    if( selectedThreshFunc == expThresFunction ) {
                        selectedType = TargetType.getType(barrelColorIndex);
                    } else if( selectedThreshFunc == expThresFunction2 ) {
                        selectedType = TargetType.getType(secondIndex);
                    } else {
                        selectedType = TargetType.getType(thirdIndex);
                    }
                    
                    vehControl.setTypeOfTargToProcess(selectedType);
                    
                    if( DEBUG_ENABLED ) System.out.println("Task alloc alg. requested: Exploit target. Type: "+selectedType);
                }
            }
            
            if( USING_SHIP ) {
                
                if(numVehicles == 0) {
                    expThresFunction = 1;
                } else {  
                    if( DEBUG_ENABLED ) System.out.println("Vehicles sensed ...");
                    expThresFunction = 1 - Math.exp(-numTargets/numVehicles);
                }
                
                if(Math.random() > expThresFunction) {  // Simulate the probability
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
