/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.taskalloc;

import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class BlindTaskAlloc extends TaskAllocAlgorithm {
    
    private String[] sensedTargets;
    private float numTargets;
    
    public BlindTaskAlloc(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }

    @Override
    protected void takeAction(float tpf) {
        
        sensedTargets = vehControl.getSensedTargetsList();
        numTargets = sensedTargets.length;
        
        if( numTargets > 0 ) {
                vehControl.updateVehicleBehavior(TaskAction.ExploitTarget);
                //System.out.println("Exploit target ...");
        } else {
            vehControl.updateVehicleBehavior(TaskAction.LookForAnother);
            //System.out.println("Looking for another target ...");
        }

        // In the case of "ship wreck", check if target has been processed and 
        // terminate execution accordingly
        if( vehControl.getProcessedTargetsList().contains("ShipWreck") ) {
            stop();
        }
        
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
