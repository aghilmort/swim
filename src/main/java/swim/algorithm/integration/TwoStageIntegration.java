/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.integration;

import java.util.Arrays;
import swim.algorithm.flocking.FlockingAlgorithm;
import swim.algorithm.flocking.ModifiedReynolds;
import swim.algorithm.search.ConstrainedSpiralFlocking;
import swim.algorithm.search.DHCP;
import swim.algorithm.search.SearchAlgorithm;
import swim.algorithm.search.SimpleSweeping;
import swim.algorithm.selforg.InitialSOAlgorithm;
import swim.algorithm.selforg.SelfOrgAlgorithm;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class TwoStageIntegration extends IntegrationAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean ALG_DEBUG_ENABLED = false;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================  
    private boolean initSOStarted = false,
                    initSOCompleted = false,
                    searchStarted = false;
    
    private SelfOrgAlgorithm selfOrgAlgorithm;
    private SearchAlgorithm searchAlgorithm;
    // =========================================================================   
   
    public TwoStageIntegration(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        
        selfOrgAlgorithm = new InitialSOAlgorithm(vehControl, sim);
        
        switch(UWVehicleControl.getActiveSearchAlgorithm().index()) {
            case 6: // DHCP
                searchAlgorithm = new DHCP(vehControl, sim);
            case 8: // SSW
                searchAlgorithm = new SimpleSweeping(vehControl, sim);
            case 1: // CSF
            default:
                searchAlgorithm = new ConstrainedSpiralFlocking(vehControl, sim);
        }        
    }

    @Override
    protected void execute() {
        
        if(!initSOStarted) {
            if(ALG_DEBUG_ENABLED) { System.out.println("Initial SO Started ..."); }
            vehControl.setActiveModes(new boolean[]{false, true, false, false, true});
            selfOrgAlgorithm.start();
            initSOStarted = true;
        }
        
        if(initSOStarted && !initSOCompleted) {
            if(ALG_DEBUG_ENABLED) { System.out.println("Initial SO updating ..."); }
            selfOrgAlgorithm.applyUpdateRules();
        }
        
        if( selfOrgAlgorithm.stopped() ) {
            if(ALG_DEBUG_ENABLED) { System.out.println("Initial SO Completed, Search Started ..."); }
            initSOCompleted = true;
            vehControl.setActiveModes(new boolean[]{false, false, true, false, true});
            searchAlgorithm.start();
            searchStarted = true;
        }
        
        if(initSOCompleted && searchStarted) {
            if(ALG_DEBUG_ENABLED) { System.out.println("Search updating ..."); }
            searchAlgorithm.applyUpdateRules();
        }
    }
    
}
