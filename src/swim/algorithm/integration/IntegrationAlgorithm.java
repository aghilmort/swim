/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.integration;

import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public abstract class IntegrationAlgorithm {
    protected long startTime, stopTime;
    protected UWVehicleControl vehControl;
    protected Simulator sim;    
    protected boolean started = false, stopped = false;
    
    public IntegrationAlgorithm(UWVehicleControl vehControl, Simulator sim) {
        this.vehControl = vehControl;
        this.sim = sim;
    }
    
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
    }
    protected abstract void execute();
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
    }
    
    public void applyUpdateRules() {
        if( started ) {
            execute();
        } else {
            System.out.println("Integration algorithm must be started first!");
        }
    }
    
    public boolean started() {
        return started;
    }
    
    public boolean stopped() {
        return stopped;
    }
    
}

