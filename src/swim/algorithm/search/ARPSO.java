package swim.algorithm.search;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.AdaptivePSOControl;

/**
 *
 * @author Sherif
 */
public class ARPSO extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final int PART_BEST_WEIGHT = 1,
                      LOCAL_BEST_WEIGHT = 5;
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private AdaptivePSOControl vehControl;
    private Vector3f p_i_minus_x_i, p_l_minus_x_i,
            particleBestComp, localBestComp,
            p_l, p_i, x_i, v_i;
    private boolean paused = false;
    // =========================================================================    
    
    public ARPSO(AdaptivePSOControl vehControl, Simulator sim) {
        // Hack ... be careful!!!
        super(null, sim);
        
        this.vehControl = vehControl;
        this.sim = sim;
    }

    @Override
    protected void search() {
        
        if( paused ) return;
        
        // Check duration constraints
        checkDurationConditions();
        if( stopped ) return; 
        
        boolean broadcasted = vehControl.broadcastSelfBestPos();
        
//        if(broadcasted) {
//            System.out.println("Self best solution and pointer broadcasted ...");
//        } 
//        else {
//            System.out.println("No neighbors found.");
//        }
        
        v_i = vehControl.getCurrentVehicleVelocity();
        //System.out.println("Current vehicle velocity: "+v_i);
        
        x_i = vehControl.getCurrentVehiclePosition();
        //System.out.println("Current vehicle position: "+x_i);
        
        //p_i_minus_x_i = vehControl.getVehicleBestPosPointer();
        p_i = vehControl.getVehicleBestPos();
        
        if(p_i == null) {
            p_i_minus_x_i = new Vector3f(0,0,0);
        } else {
            p_i_minus_x_i = p_i.subtract(x_i);
            //System.out.println("Entered here ..., p_i = "+p_i);
        }
        
        //System.out.println("Self best position pointer: "+p_i_minus_x_i);
        
        particleBestComp = p_i_minus_x_i.mult(FastMath.rand.nextFloat() * PART_BEST_WEIGHT);
        
        if( particleBestComp.length() != 0 ) {
            System.out.println("(PSO) Particle best update component: "+particleBestComp);
        }
        
        p_l = vehControl.getNeighborhoodBestPosition();
        if(p_l == null) {
            p_l = vehControl.getMostRecentNeighPosPointer();
            if(p_l != null) {
                System.out.println("Most recent neighborhood best position for ("+vehControl.getVehicleName()+"): "+p_l);
            }
        }
        //System.out.println("Neighborhood best position: "+p_l);
        
        if(p_l == null) {
            p_l_minus_x_i = new Vector3f(0,0,0);
        } else {
            p_l_minus_x_i = p_l.subtract(x_i);
        }
        //System.out.println("Neighborhood best position pointer: "+p_l_minus_x_i);
        
        localBestComp = p_l_minus_x_i.mult(FastMath.rand.nextFloat() * LOCAL_BEST_WEIGHT);
        
        if( localBestComp.length() != 0 ) {
            System.out.println("(PSO) Local best update component: "+localBestComp);
            //DebugUtils.plotArrow(x_i, p_l_minus_x_i, ColorRGBA.Cyan, 2f, sim);
        }
        
        // =====================================================================
        // Construct a weight for this update to compare it to future weights
        // =====================================================================
        
        // Get particle's best solution
        Integer vehBestSol = vehControl.getVehicleBestSolution();
        
        System.out.println("(PSO) Particle best solution: "+vehBestSol);
        
        // Get local best solution  (must be called after "getNeighborhoodBestPosition()"
        // or will be null
        Integer localBestSol = vehControl.getNeighborhoodBestSol();
        
        System.out.println("(PSO) Local best solution: "+localBestSol);
        
        Integer newlyTargetedWeight = 
                (vehBestSol != null ? PART_BEST_WEIGHT * vehBestSol : 0) + 
                (localBestSol != null ? LOCAL_BEST_WEIGHT * localBestSol : 0);
        
        System.out.println("(PSO) Targetted weight: "+newlyTargetedWeight);
        
        // =====================================================================

        if(newlyTargetedWeight > vehControl.getCurrTargetedWeight()) {
            //System.out.println("Vehicle velocity: "+v_i);
            v_i = v_i.add(particleBestComp).add(localBestComp);
            
            if(vehControl.isCompTurnInProgress()) {
               vehControl.resetCompositeTurnFlags(); 
            }
            
            //System.out.println("Updated vehicle velocity: "+v_i);
        }
        
        vehControl.setCurrTargetedWeight(newlyTargetedWeight);
        
        vehControl.updateVehicleVelocity(v_i);
        //System.out.println("Position to move to from current vehicle position: "+v_i.add(vehControl.getCurrentVehiclePosition()));
        
//        if( localBestComp.length() != 0 ) {
//            DebugUtils.plotArrow(x_i, p_i_minus_x_i, ColorRGBA.Magenta, 2f, sim);
//        }
        
        //DebugUtils.plotArrow(x_i, p_i_minus_x_i, ColorRGBA.Magenta, 2f, sim);
        vehControl.setCommInitiated(false);
    }

    public long getExecutionTime() {
        if(stopped) {
            return stopTime - startTime;
        } else {
            System.out.println("Algorithm is still running!");
            return -1;
        }
    }

    @Override
    protected boolean targetFound() {
        return true;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {
    }

    @Override
    protected void saveState() {
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeVectorVar("p_i_minus_x_i", p_i_minus_x_i);
        state.storeVectorVar("p_l_minus_x_i", p_l_minus_x_i);
        state.storeVectorVar("particleBestComp", particleBestComp);
        state.storeVectorVar("localBestComp", localBestComp);
        state.storeVectorVar("p_l", p_l);
        state.storeVectorVar("p_i", p_i);
        state.storeVectorVar("x_i", x_i);
        state.storeVectorVar("v_i", v_i);
        
        savedStates.push(state);
        
        paused = true;
    }

    @Override
    protected void restoreState() {
        AlgorithmState state = savedStates.pop();
        useState(state);
    }
    
    protected void useState(AlgorithmState state) {
        
        duration = state.getDuration();
        goal = state.getGoal();
        
        presetDur = state.getIntVar("presetDur");
        durLength = state.getIntVar("durLength");
        
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        p_i_minus_x_i = state.getVectorVar("p_i_minus_x_i");
        p_l_minus_x_i = state.getVectorVar("p_l_minus_x_i");
        particleBestComp = state.getVectorVar("particleBestComp");
        localBestComp = state.getVectorVar("localBestComp");
        p_l = state.getVectorVar("p_l");
        p_i = state.getVectorVar("p_i");
        x_i = state.getVectorVar("x_i");
        v_i = state.getVectorVar("v_i");

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }
    
}
