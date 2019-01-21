/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.selforg;

import swim.core.MissionStage;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public abstract class SelfOrgAlgorithm {

    protected boolean started = false, stopped = false;
    protected long startTime, stopTime;
    
    protected UWVehicleControl vehControl;
    protected Simulator sim;
    
    public SelfOrgAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        this.sim = sim;
        this.vehControl = vehControl;
    }
    
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
        vehControl.setActiveMotionAlgKey(startTime);
        //System.out.println("Search Start time should be set by NOW!!!!");
    }
    
    protected abstract void selfOrganize();
    
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
        vehControl.reportMotionAlgStopped(getKey());
        vehControl.reportMissionStageCompleted(MissionStage.INITIAL_SELF_ORGANIZATION);
    }
    
    public void applyUpdateRules() {
        if( started ) {
            selfOrganize();
        } else {
            System.out.println("Self-Organization algorithm must be started first!");
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
