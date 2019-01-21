/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class MiniResourceMonitor extends ResourceMonitor {
    
    private UWVehicleControl agentController;
    
    private float energyLevel;
    
    /**
     * @todo: Change argument to be more general, e.g. "agent controller"
     */ 
    public MiniResourceMonitor( UWVehicleControl agentController ) {
        this.agentController = agentController;
    }
    
    @Override
    public ResidualEnergyLevel getResidualEnergy() {

        energyLevel = agentController.getResidualEnergy();
        float fullBatteryEnergy = agentController.getInitBatteryEnergy();
        float percent = 100 * energyLevel/fullBatteryEnergy;
        
        if( percent >= 90 ) {
            return ResidualEnergyLevel.VERY_HIGH;
        } else if( percent >= 60 && percent < 90 ) {
            return ResidualEnergyLevel.HIGH;
        } else if( percent >= 40 && percent < 60 ) {
            return ResidualEnergyLevel.MEDIUM;
        } else if( percent >= 10 && percent < 40 ) {
            return ResidualEnergyLevel.LOW;
        } else {
            return ResidualEnergyLevel.VERY_LOW;
        }
    }
    
    @Override
    public void evaluateResources() {
        
    }
    
}
