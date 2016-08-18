/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.test;

import com.jme3.system.AppSettings;
import java.util.logging.Level;
import java.util.logging.Logger;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public class SimRunner extends Thread {

    // Config - START
    private final boolean SMALL_SCREEN_RESOLUTION_ACTIVE = false;
    // Config - END
    
    private Simulator sim;
    private boolean simRunning = false,
            started = false;
    private int runNumber, numRuns;

    public SimRunner(int runNumber, int numRuns) {
        this.runNumber = runNumber;
        this.numRuns = numRuns;
        this.setName("SimRunner");
    }
    
    @Override
    public void run() {
        
        if(!simRunning) {
            sim = new Simulator(runNumber, numRuns);
            
            sim.setShowSettings(false);
            AppSettings settings = new AppSettings(true);
            if(SMALL_SCREEN_RESOLUTION_ACTIVE) {
                settings.setResolution(1280, 768);
            } else {
                settings.setResolution(1600, 900);
            }            
            settings.setBitsPerPixel(32);
            sim.setSettings(settings);
            
            sim.start();
            started = true;
            simRunning = true;
        }
        
        while(!sim.stopped) {
            try {
                Thread.sleep(10000);
                //System.out.println("In Sim Thread ...");
            } catch (InterruptedException ex) {
                Logger.getLogger(SimRunner.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        
        
        System.out.println("Simulation stopped ...");
        
    }
    
    public boolean isSimRunning() {
        return simRunning;
    }
    
    public boolean started() {
        return started;
    }
    
    public Simulator getSimulator() {
        return sim;
    }
    
}
