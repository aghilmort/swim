package swim.algorithm.surfacing;

import swim.core.MissionStage;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public abstract class SurfacingAlgorithm {
    
    protected UWVehicleControl vehControl;
    protected Simulator sim;
    
    protected long startTime, stopTime;
    protected boolean started = false, stopped = false;
    
    public SurfacingAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        this.vehControl = vehControl;
        this.sim = sim;
    }
    
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
        vehControl.setActiveMotionAlgKey(startTime);
        System.out.println("Surfacing algorithm started.");
    }
    
    protected abstract void executeSurfacingStep();
    
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
        vehControl.reportMotionAlgStopped(getKey());
        vehControl.reportMissionStageCompleted(MissionStage.SOURCE_SEARCH);
        System.out.println("Surfacing algorithm completed.");
    }
    
    protected abstract boolean surfaced();
    
    public abstract void setMaxIterations(int maxNumIter);
    
    public void applyUpdateRules() {
        if( started ) {
            executeSurfacingStep();
        } else {
            System.out.println("Surfacing algorithm must be started first!");
        }
    }
    
    public boolean started() {
        return started;
    }
    
    public boolean stopped() {
        return stopped;
    }
    
    public long getKey() {
        return startTime;
    }
    
}