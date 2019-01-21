package swim.algorithm.integration;

import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.sim.uwswarm.intelli.brain.MiniBrain;

/**
 *
 * @author Sherif
 */
public class AllStageIntegration extends IntegrationAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean ALG_DEBUG_ENABLED = false;
    // =========================================================================   
    // Algorithm variables
    // ========================================================================= 
    private MiniBrain brain;
    
    
    private int decisionCounter = 0;
    // =========================================================================   
   
    public AllStageIntegration(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        this.brain = vehControl.getBrain();
    }

    @Override
    protected void execute() {
        
        System.out.println("Brain asked to make a decision "+ ++decisionCounter);
        brain.makeDecision();
        
    }
    
}
