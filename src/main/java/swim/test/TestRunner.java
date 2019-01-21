/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.test;

import java.util.logging.Level;
import java.util.logging.Logger;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public class TestRunner {

    private int numRuns = 10, 
            numRunsRemaining = numRuns,
            runNumber = 1;
    public boolean nextTestWasRun = false;
    
    private SimRunner simRunner;
    
    private boolean simRunning = false;
    
    public void run() {

        if(Simulator.bulkSimActive()) {
            
            if( !simRunning && numRunsRemaining > 0 ) {
                simRunner = new SimRunner(runNumber, numRuns);
                simRunner.setDaemon(true);
                simRunner.start();
                runNumber++;
                numRunsRemaining--;
                simRunning = true;

                while(simRunner.isAlive()) {
                    try {
                        simRunner.join(10000);
                    } catch (InterruptedException ex) {
                        Logger.getLogger(TestRunner.class.getName()).log(Level.SEVERE, null, ex);
                    }
                }

                simRunning = false;
                run();
            }
        
        } else {
            simRunner = new SimRunner(-1, numRuns);
            simRunner.start();
        }
        
    }
    
    public static void main(String[] args) {
        
        TestRunner myTestRunner = new TestRunner();
        myTestRunner.run();
        
    }
    
}
