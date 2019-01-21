/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.taskalloc;

import swim.core.MissionStage;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public abstract class TaskAllocAlgorithm {
    
    protected UWVehicleControl vehControl;
    protected Simulator sim;
    
    protected long startTime, stopTime;
    protected boolean started = false, stopped = false;
    
    public TaskAllocAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        this.vehControl = vehControl;
        this.sim = sim;
    }
    
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
        vehControl.setActiveMotionAlgKey(startTime);
        System.out.println("Task Allocation algorithm started.");
    }
    
    protected abstract void takeAction(float tpf);
    
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
        vehControl.reportMotionAlgStopped(getKey());
        vehControl.reportMissionStageCompleted(MissionStage.TASK_ALLOCATION);
        System.out.println("Task Allocation algorithm completed.");
    }
    
    protected abstract boolean taskDone();
    public abstract void setMaxIterations(int maxNumIter);
    
    public void applyUpdateRules(float tpf) {
        if( started ) {
            takeAction(tpf);
        } else {
            System.out.println("Task allocation algorithm must be started first!");
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
