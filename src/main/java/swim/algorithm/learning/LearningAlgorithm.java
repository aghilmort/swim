package swim.algorithm.learning;

/**
 *
 * @author Sherif
 */
public abstract class LearningAlgorithm {
    
    protected boolean started = false, stopped = false;
    
    public abstract void start();
    protected abstract void learn();
    protected abstract void stop();
    protected abstract void applyPolicy();
    public abstract void setMaxIterations(int maxNumIter);
    
    public void applyUpdateRules() {
        if( started ) {
            learn();
        } else {
            System.out.println("Learning algorithm must be started first!");
        }
    }
    
    public boolean started() {
        return started;
    }
    
    public boolean stopped() {
        return stopped;
    }    
    
}
