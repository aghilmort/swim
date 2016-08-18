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
public class VelocityCalibration extends CalibrationAlgorithm {

    private long startTime, stopTime;
    
    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
    }

    @Override
    protected void calibrate() {
        
    }

    @Override
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
    }
    
}
