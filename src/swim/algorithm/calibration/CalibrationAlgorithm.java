/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.calibration;

/**
 *
 * @author Sherif
 */
public abstract class CalibrationAlgorithm {
  
    protected boolean started = false, stopped = false;
    
    public abstract void start();
    protected abstract void calibrate();
    protected abstract void stop();
    
    public void applyUpdateRules() {
        if( started ) {
            calibrate();
        } else {
            System.out.println("Calibration algorithm must be started first!");
        }
    }
    
    public boolean started() {
        return started;
    }
    
    public boolean stopped() {
        return stopped;
    }    
    
}
