/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.surfacing;

import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class SimpleSurfacing extends SurfacingAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private boolean ALG_DEBUG = true;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private boolean surfacingStarted = false;
    // =========================================================================
    
    public SimpleSurfacing(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
    }
    
    @Override
    protected void executeSurfacingStep() {
        if(!surfacingStarted) {
            if( ALG_DEBUG ) System.out.println(vehControl.getVehicleName()+": Surfacing started.");
            vehControl.setTargetFound(false);
            vehControl.solicitInPlaceTurn();
            surfacingStarted = true;
        }
        
        vehControl.solicitUpwardsAcceleration(0.01f);
        if( ALG_DEBUG ) System.out.println(vehControl.getVehicleName()+": Going up ...");
    }

    @Override
    protected boolean surfaced() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void setMaxIterations(int maxNumIter) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
}
