package swim.algorithm.integration;

import com.jme3.math.ColorRGBA;
import java.util.HashMap;
import swim.algorithm.search.SearchAlgorithms;
import swim.algorithm.selforg.InitSelfOrgAlgorithms;
import swim.algorithm.surfacing.SurfacingAlgorithms;
import swim.algorithm.taskalloc.TaskAllocAlgorithms;
import swim.core.MissionStage;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class BrainlessAllStageInt extends IntegrationAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean ALG_DEBUG_ENABLED = false;
    private int START_MISSION_STAGE = 1;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================  
    private HashMap<Integer, Pair<MissionStage,Enum>> missionStageActionPairs;
    private int activeMissionIndex = START_MISSION_STAGE;
    private boolean noStageSelectedBefore = true;
    private MissionStage activeMissionStage;  
    private Enum activeMissionLvlAction;
    private ColorRGBA currentColor, newColor;
    // =========================================================================

    public BrainlessAllStageInt(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        setMissionStageActionPairs();
        
        activeMissionStage = missionStageActionPairs.get(activeMissionIndex).key;
        activeMissionLvlAction = missionStageActionPairs.get(activeMissionIndex).val;
        
        vehControl.setActiveMissionStage(activeMissionStage);
        vehControl.setActiveMissionLvlAction(activeMissionLvlAction);
        
        if( UWVehicleControl.getActiveSearchAlgorithm().equals(SearchAlgorithms.SDHCP) ||
            UWVehicleControl.getActiveSearchAlgorithm().equals(SearchAlgorithms.VTS) || 
            UWVehicleControl.getActiveSearchAlgorithm().equals(SearchAlgorithms.RPSO) ) {
            activeMissionIndex = 2;
        }
        
        currentColor = vehControl.getPathDebugColor();
    }

    @Override
    protected void execute() {
        
        boolean activeMissionCompleted = 
                (activeMissionStage != null && vehControl.isMissionStageCompleted(activeMissionStage));
        
        // The very beginning
        if( noStageSelectedBefore || activeMissionCompleted ) {
            
            if( activeMissionCompleted ) {
                ++activeMissionIndex;
                if(activeMissionIndex > missionStageActionPairs.size()) return;
                
                do {
                    newColor = ColorRGBA.randomColor();
                } while ( newColor.equals(currentColor) );
                
                vehControl.changePathDebugColor(newColor);
                vehControl.changePathDebugColor(2f);
            }
            
            activeMissionStage = missionStageActionPairs.get(activeMissionIndex).key;
            activeMissionLvlAction = missionStageActionPairs.get(activeMissionIndex).val;
            
            vehControl.setActiveMissionStage(activeMissionStage);
            vehControl.setActiveMissionLvlAction(activeMissionLvlAction);
            
            vehControl.initActiveMissionAction();
            vehControl.startActiveMissionAction();
            
            //System.out.println("Mission stage: "+activeMissionStage);
            //System.out.println("Mission action: "+activeMissionLvlAction);
            
            noStageSelectedBefore = false;
            
        }
     
    }
    
    private void setMissionStageActionPairs() {
        
        SearchAlgorithms searchAlgorithm = UWVehicleControl.getActiveSearchAlgorithm();
        TaskAllocAlgorithms taskAllocAlgorithm = UWVehicleControl.getActiveTaskAllocAlgorithm();
        InitSelfOrgAlgorithms initSOAlgorithm = InitSelfOrgAlgorithms.MAG_NORTH_BASED;  // can be changed in the future
        SurfacingAlgorithms srcSearchAlgorithm = SurfacingAlgorithms.SSRF;  // can be changed in the future
        
        missionStageActionPairs = new HashMap();
        
        missionStageActionPairs.put(1, new Pair(MissionStage.INITIAL_SELF_ORGANIZATION, initSOAlgorithm));
        missionStageActionPairs.put(2, new Pair(MissionStage.TARGET_SEARCH, searchAlgorithm));
        missionStageActionPairs.put(3, new Pair(MissionStage.TASK_ALLOCATION, taskAllocAlgorithm));
        missionStageActionPairs.put(4, new Pair(MissionStage.SOURCE_SEARCH, srcSearchAlgorithm));
    }
    
}
