/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.taskalloc;

import java.util.Arrays;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class PolyRespThresh extends TaskAllocAlgorithm {
    
    private String[] sensedVehicles,
                     sensedTargets;
    
    private float numVehicles,
                  numTargets;
    
    private double expThresFunction;
                     
    
    public PolyRespThresh(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    @Override
    protected void takeAction(float tpf) {
        
        sensedVehicles = vehControl.getSensedVehiclesList();
        sensedTargets = vehControl.getSensedTargetsList();
        
        numVehicles = sensedVehicles.length;
        numTargets = sensedTargets.length;
        
        if( numTargets > 0 ) {
            
            if(numVehicles == 0) {
                expThresFunction = 1;
            } else {
                expThresFunction = Math.pow(numTargets, 2)/(Math.pow(numTargets, 2) + Math.pow(numVehicles, 2));
            }
            
            if(Math.random() > expThresFunction) {  // Simulate the probability
                vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
                //System.out.println("Looking for another target ...");
            } else {
                vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                //System.out.println("Exploiting target ...");
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
