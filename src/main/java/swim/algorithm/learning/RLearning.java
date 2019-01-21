package swim.algorithm.learning;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class RLearning extends LearningAlgorithm {

    private final int Q_MATRIX_SIZE = 8,
                      MAX_NUM_ITERATIONS = 100000,
                      MIN_NUM_ITERATIONS = 10000;
    private final float LEARNING_TERMINATION_THRESH = 0.25f,
                        ALPHA = 0.1f,   // learning rate or step size
                        BETA = 0.01f;   // rho update step size
    
    private long startTime, stopTime;
    private UWVehicleControl vehControl;
    private Simulator sim;
    
    private float[][] QMatrix = new float[Q_MATRIX_SIZE][Q_MATRIX_SIZE];
    private float[][] rho = new float[Q_MATRIX_SIZE][Q_MATRIX_SIZE];         // Average expected reward
    
    private int currentStateIndex, nextStateIndex, currentActionIndex;
    
    private int iterationNumber = 0;
    private float maxDiff = Float.MAX_VALUE;
    
    private boolean learningCompleted = false;
    
    private float reward;
    
    private float qMaxRewardCurrStateAction,     // Action-value of maximum-reward 
                                                 // action in the current state
                  qMaxRewardNextStateAction;     // Action-value of maximum-reward 
                                                 // action in the next state
    
    public RLearning(UWVehicleControl vehControl, Simulator sim) {
        this.vehControl = vehControl;
        this.sim = sim;
        
        // Initialize Q and rho matrices to all zeros
        for(int i = 0; i < Q_MATRIX_SIZE; i++){
            for(int j = 0; j < Q_MATRIX_SIZE; j++){
                QMatrix[i][j] = 0;
                rho[i][j] = 0;
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
        
        if(iterationNumber > MAX_NUM_ITERATIONS || (
                maxDiff < LEARNING_TERMINATION_THRESH && iterationNumber >= MIN_NUM_ITERATIONS)) {
            learningCompleted = true;
            return;
        }
        
        // Get the current turning state: s_t
        currentStateIndex = vehControl.getCurrentTurningState();
        
        // Get the next turning state from the vehicle controller: s_t+1
        nextStateIndex = vehControl.getNextTurningState();
        
        // Get the current turning action: a_t
        currentActionIndex = vehControl.getCurrentTurningAction();
        
        // Get reward received after selecting current action (a_t): r
        reward = vehControl.getTurningReward();  
        
        System.out.println("Reward: "+reward);
        
        // Retrieve the action-value of maximum-reward action in the 
        // NEXT state from Q-matrix
        qMaxRewardNextStateAction = 0;
        for(int i = 0; i < Q_MATRIX_SIZE; i++) {
            qMaxRewardNextStateAction = Math.max(QMatrix[nextStateIndex][i], qMaxRewardNextStateAction);
        }
         
        // Update the Q matrix: Q_t+1(s_t, a_t)
        QMatrix[currentStateIndex][currentActionIndex] =
                QMatrix[currentStateIndex][currentActionIndex] + 
                (ALPHA * (reward - rho[currentStateIndex][currentActionIndex] + qMaxRewardNextStateAction - 
                QMatrix[currentStateIndex][currentActionIndex]));
        
        
        // Retrieve the state-action value of maximum-reward action in the 
        // CURRENT state from Q-matrix
        qMaxRewardCurrStateAction = 0;
        for(int i = 0; i < Q_MATRIX_SIZE; i++) {
            qMaxRewardCurrStateAction = Math.max(QMatrix[currentStateIndex][i], qMaxRewardCurrStateAction);
        }
        
        // Check if it is equal to Q(s_t,a_t) ... if so, update rho
        if(QMatrix[currentStateIndex][currentActionIndex] == qMaxRewardCurrStateAction) {
            rho[currentStateIndex][currentActionIndex] = rho[currentStateIndex][currentActionIndex] + (BETA * (reward - rho[currentStateIndex][currentActionIndex] + qMaxRewardNextStateAction - qMaxRewardCurrStateAction));
        }
        
        // Update the learning stop condition
        maxDiff = 0;
        for(int i = 0; i < Q_MATRIX_SIZE; i++){
            for(int j = 0; j < Q_MATRIX_SIZE; j++){
                maxDiff = Math.max(maxDiff, QMatrix[i][j]);
            }
        }
        
        printMatrices(new boolean[]{true, false});
        
        iterationNumber++;
        
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
    
    public void setReceivedReward(float reward) {
        this.reward = reward;
    }
    
    private void printMatrices(boolean[] matricesToDisp) {
        
        if( matricesToDisp[0] ) {
            System.out.println("============================");
            System.out.println(vehControl.getVehicleName() + ": Q-Matrix:");
            System.out.println("============================");
            for(int i = 0; i < Q_MATRIX_SIZE; i++){
                for(int j = 0; j < Q_MATRIX_SIZE; j++){
                    System.out.printf("%f ", QMatrix[i][j]);
                }
                System.out.println();
            }
        }
        
        if( matricesToDisp[1] ) {
            System.out.println("============================");
            System.out.println(vehControl.getVehicleName() + ": R-Matrix:");
            System.out.println("============================");
            for(int i = 0; i < Q_MATRIX_SIZE; i++){
                for(int j = 0; j < Q_MATRIX_SIZE; j++){
                    System.out.printf("%f ", rho[i][j]);
                }
                System.out.println();
            }
        }
    }
    
}
