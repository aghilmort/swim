/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.algorithm.learning;

import com.jme3.math.Vector3f;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class QLearning extends LearningAlgorithm {

    private final int Q_MATRIX_SIZE = 16;   
    
    private long startTime, stopTime;
    private UWVehicleControl vehControl;
    private Simulator sim;
    
    float[][] QMatrix = new float[Q_MATRIX_SIZE][Q_MATRIX_SIZE];
    
    private int currentStateIndex, nextStateIndex, currentActionIndex;
    
    
    public QLearning(UWVehicleControl vehControl, Simulator sim) {
        this.vehControl = vehControl;
        this.sim = sim;
        
        // Initialize Q matrix
        for(int i = 0; i < Q_MATRIX_SIZE; i++){
            for(int j = 0; j < Q_MATRIX_SIZE; j++){
                QMatrix[i][j] = 0;
            }
        }
    }
    
    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
    }

    @Override
    protected void learn() {
        
        // Get the current turning state: s_t
        currentStateIndex = vehControl.getCurrentTurningState();
        
        // Get the next turning state from the vehicle controller: s_t+1
        nextStateIndex = vehControl.getNextTurningState();
        
        // Get the current turning action: a_t
        currentActionIndex = vehControl.getCurrentTurningAction();
        
        
        // Update the Q matrix: Q_t+1(s_t, a_t)
//        QMatrix[currentState][currentAction] =
//                QMatrix[currentState][currentAction] + 
        
    }
    
    @Override
    protected void applyPolicy() {
        
    }

    @Override
    protected void stop() {
        stopTime = System.currentTimeMillis();
        stopped = true;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {
    }
    
}
