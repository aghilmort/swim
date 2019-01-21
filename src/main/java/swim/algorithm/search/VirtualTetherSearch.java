package swim.algorithm.search;

import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.Random;
import swim.algorithm.common.AlgorithmState;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.DebugUtils;

/**
 *
 * @author Sherif
 */
public class VirtualTetherSearch extends SearchAlgorithm {

    // =========================================================================
    // Constants
    // =========================================================================
    private final boolean VTS_DEBUG_ENABLED = false,
                          USE_EXACT_LOC_CALC = false;
    
    private final ColorRGBA TETHER_COLOR = ColorRGBA.Orange; 
    
    private final float MAX_TETHER_LENGTH = 80,   // Maximum allowable radius for the search area
                  MAX_ANGLE_WITH_TETHER = FastMath.PI,
                  MAX_LEN_ANGLE_PRODUCT = MAX_TETHER_LENGTH * MAX_ANGLE_WITH_TETHER,
                  // k = -1.2 (see: https://dinodini.wordpress.com/2010/04/05/normalized-tunable-sigmoid-functions/ )
                  NORM_TUN_HALF_SIG_CURV_COEFF = -1.2f,
                  NORMAL_DIST_STAND_DEV = 0.8f;      
    
    // =========================================================================   
    // Algorithm variables
    // =========================================================================
    private Vector3f tether, currVehPos, prevVehPos, 
            prevPosPointer, currTurnDir, selTurnDir, velUpdateDir;
    private boolean initTetherDirNotSet = true;
    private float theta, tetherLength, 
            tetherLenAngleProduct, normTetherLenAngProd, 
            selTetherLenAngProd, normTunHalfSigFn, currTurnDirAngle;
    
    private Quaternion rotQuaternion;
    
    private int traveledSteps = 0;
    private boolean paused = false, 
                    maxRadiusDrawn = false;
    
    private SearchAlgSpan searchSpan = SearchAlgSpan.GLOBAL;
    
    private float maxDistToTravel, distTraveledSoFar = 0;
    private Vector3f travelStartLoc;
    
    private boolean justBrokeATravel = false;
    // =========================================================================   
    
    public VirtualTetherSearch(UWVehicleControl vehControl, Simulator sim) {
        super(vehControl, sim);
        rotQuaternion = new Quaternion();
    }
    
    public VirtualTetherSearch(UWVehicleControl vehControl, Simulator sim, SearchAlgSpan span) {
        this(vehControl, sim);
        this.searchSpan = span; 
    }

    @Override
    protected void search() {

        if( vehControl.isTargetFound() ) {
            stop();
            return;
        }
        
        // Used for debugging: draw the maximum search radius
        if(!maxRadiusDrawn && VTS_DEBUG_ENABLED && vehControl.getVehicleName().equals("V_1")) {
            Vector3f startLoc = UWVehicleControl.getDropOffLocation().setY(3);
            float boxSideLength = (float) java.lang.Math.cbrt(sim.getVehiclesContainerVolume());
            Vector3f endLoc = UWVehicleControl.getDropOffLocation().setY(3).add(MAX_TETHER_LENGTH +(0.5f*boxSideLength), 0, 0);
            DebugUtils.plotArrow(startLoc, endLoc.subtract(startLoc), ColorRGBA.Black, 2, sim);
            maxRadiusDrawn = true;
        }
        
        if( paused ) return;
        
        if(vehControl.compositeTurnInProgress()) {
            return;
        }
        
        // Check duration constraints (stops the algorithm based on different
        // preset duration constraints)
        checkDurationConditions();
        if( stopped ) return; 
        
        if(initTetherDirNotSet) {
            
            if( USE_EXACT_LOC_CALC ) {
                prevVehPos = vehControl.getInContainerVehPos().setY(10); // start depth
                currVehPos = vehControl.getPhysicsVehicle().getPhysicsLocation();
            } else {
                prevVehPos = vehControl.getEstimatedInitPos();
                currVehPos = vehControl.estimatePos();
            }

            prevPosPointer = prevVehPos.subtract(currVehPos);
            tether = prevPosPointer.clone();
            if(VTS_DEBUG_ENABLED) { DebugUtils.plotArrow(currVehPos, tether, TETHER_COLOR, 2f, sim); }
            prevVehPos = currVehPos;
            initTetherDirNotSet = false;           
        }

        // First re-estimate tether direction and length
        reestimateTetherVector();
        
        // Find the angle "theta" between the vehicle's orientation and thether 
        // direction
        theta = vehControl.getActiveVelocity().normalize().angleBetween(tether.normalize());
        
        if(VTS_DEBUG_ENABLED) System.out.println("Theta: " + theta*FastMath.RAD_TO_DEG);
        
        // Calculate tether length
        tetherLength = tether.length();
        
        // Find maximum distance that can be traveled withough exceeding the
        // max. tether length 
        float a, b, c, root_1, root_2;
        a = 1;
        b = -2*tetherLength*FastMath.cos(theta);  
        c = FastMath.pow(tetherLength, 2) - FastMath.pow(MAX_TETHER_LENGTH, 2);
        root_1 = (-b + FastMath.sqrt(FastMath.pow(b, 2) - 4*a*c))/(2*a);
        root_2 = (-b - FastMath.sqrt(FastMath.pow(b, 2) - 4*a*c))/(2*a);
        maxDistToTravel = Math.max(root_1, root_2);
        
        travelStartLoc = vehControl.getPhysicsVehicle().getPhysicsLocation();
        
        if(VTS_DEBUG_ENABLED) {
            System.out.println("Roots: ("+root_1+", "+root_2+")");
            System.out.println("Max. distance to travel: "+maxDistToTravel);
        }
        
        // With a very low probability break the travel and trigger a thether 
        // recalculation and a turn
        if( Math.random() > 0.99f && !justBrokeATravel ) {
            distTraveledSoFar = maxDistToTravel;
            justBrokeATravel = true;
            return;
        }
        
        justBrokeATravel = false;
        
        if( distTraveledSoFar < maxDistToTravel ) {
            vehControl.setActiveVelocity(vehControl.getActiveVelocity().normalize().mult(vehControl.getVehicleSpeed()));
            distTraveledSoFar = vehControl.getPhysicsVehicle().getPhysicsLocation().subtract(travelStartLoc).length();
            return;
        } else {
            distTraveledSoFar = 0;
        }
        
        if(VTS_DEBUG_ENABLED) {
            System.out.println("Angle with tether: "+theta);
            System.out.println("Tether Length: "+tetherLength);
        }
        
        // Find tether length-vehicle orientation angle product and normalize
        // it to use with normalized tunable half sigmoid function
        tetherLenAngleProduct = tetherLength * theta;
        normTetherLenAngProd = tetherLenAngleProduct / MAX_LEN_ANGLE_PRODUCT;
        
        // Find a value around it with a probability that is normally distributed
        // with its mean is "normTetherLenAngProd" and standard deviation 0.25
        Random rnd = new Random();
        selTetherLenAngProd = -1;
        while(selTetherLenAngProd < 0 || selTetherLenAngProd > 1) {
            selTetherLenAngProd = (float)(normTetherLenAngProd + (NORMAL_DIST_STAND_DEV * rnd.nextGaussian()));
        }
        
        if(VTS_DEBUG_ENABLED) System.out.println("Selected tether length-angle product: " + selTetherLenAngProd);
        
        // Use that value to find the next angle using the
        // normalized half sigmoid function with k = -1.2. The result is in the 
        // range [0,1]. We multiply that by Pi to get the angle
        normTunHalfSigFn = NORM_TUN_HALF_SIG_CURV_COEFF * selTetherLenAngProd / 
                (NORM_TUN_HALF_SIG_CURV_COEFF - selTetherLenAngProd + 1);
        
        if(VTS_DEBUG_ENABLED) System.out.println("Normalized half sigmoid: " + normTunHalfSigFn);
        
        currTurnDirAngle = normTunHalfSigFn * FastMath.PI;
        rotQuaternion.fromAngleAxis(currTurnDirAngle, Vector3f.UNIT_Y);
        currTurnDir = rotQuaternion.mult(vehControl.getActiveVelocity().normalize());

        // Set the selected direction
        selTurnDir = currTurnDir.clone();
        
        // Give the selected turn direction the magnitude of active velocity
        velUpdateDir = selTurnDir.mult(vehControl.getActiveVelocity().length());
        
        // Turn the vehicle in that direction
        vehControl.updateVehicleVelocity(velUpdateDir);
        
    }
    
    private void reestimateTetherVector() {
        
        if( USE_EXACT_LOC_CALC ) {
            currVehPos = vehControl.getPhysicsVehicle().getPhysicsLocation();
        } else {
            currVehPos = vehControl.estimatePos();
        }
        
        prevPosPointer = prevVehPos.subtract(currVehPos);
        tether = prevPosPointer.add(tether);
        if(VTS_DEBUG_ENABLED) { DebugUtils.plotArrow(currVehPos, tether, TETHER_COLOR, 2f, sim); }
        prevVehPos = currVehPos;
    }

    @Override
    protected boolean targetFound() {
        return false;
    }

    @Override
    public void setMaxIterations(int maxNumIter) {}
    
        @Override
    protected void saveState() {
        
        // ==================================================
        // IMPORTANT!!
        // ==================================================
        // Go through them later to discard unneeded ones
        // ==================================================
        
        AlgorithmState state = new AlgorithmState();
        
        state.storeDuration(duration);
        state.storeGoal(goal);
        
        state.storeIntVar("presetDur", presetDur);
        state.storeIntVar("durLength", durLength);
        state.storeIntVar("traveledSteps", traveledSteps);
        
        state.storeFlagVar("interrupted", interrupted);
        state.storeFlagVar("durLengthSet", durLengthSet);
        state.storeFlagVar("started", started);
        state.storeFlagVar("stopped", stopped);
        state.storeFlagVar("initTetherDirNotSet", initTetherDirNotSet);
        
        state.storeFloatVar("theta", theta);
        state.storeFloatVar("tetherLength", tetherLength);
        state.storeFloatVar("tetherLenAngleProduct", tetherLenAngleProduct);
        state.storeFloatVar("normTetherLenAngProd", normTetherLenAngProd);
        state.storeFloatVar("selTetherLenAngProd", selTetherLenAngProd);
        state.storeFloatVar("normTunHalfSigFn", normTunHalfSigFn);
        state.storeFloatVar("currTurnDirAngle", currTurnDirAngle);
        
        state.storeLongVar("startTime", startTime);
        state.storeLongVar("stopTime", stopTime);
        
        state.storeVectorVar("tether", tether);
        state.storeVectorVar("currVehPos", currVehPos);
        state.storeVectorVar("prevVehPos", prevVehPos);
        state.storeVectorVar("prevPosPointer", prevPosPointer);
        state.storeVectorVar("currTurnDir", currTurnDir);
        state.storeVectorVar("selTurnDir", selTurnDir);
        state.storeVectorVar("velUpdateDir", velUpdateDir);        

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
        traveledSteps = state.getIntVar("traveledSteps");
        
        interrupted = state.getFlagVar("interrupted");
        durLengthSet = state.getFlagVar("durLengthSet");
        started = state.getFlagVar("started");
        stopped = state.getFlagVar("stopped");
        initTetherDirNotSet = state.getFlagVar("initTetherDirNotSet");
        
        theta = state.getFloatVar("theta");
        tetherLength = state.getFloatVar("tetherLength");
        tetherLenAngleProduct = state.getFloatVar("tetherLenAngleProduct");
        normTetherLenAngProd = state.getFloatVar("normTetherLenAngProd");
        selTetherLenAngProd = state.getFloatVar("selTetherLenAngProd");
        normTunHalfSigFn = state.getFloatVar("normTunHalfSigFn");
        currTurnDirAngle = state.getFloatVar("currTurnDirAngle");
        
        startTime = state.getLongVar("startTime");
        stopTime = state.getLongVar("stopTime");
        
        tether = state.getVectorVar("tether");
        currVehPos = state.getVectorVar("currVehPos");
        prevVehPos = state.getVectorVar("prevVehPos");
        prevPosPointer = state.getVectorVar("prevPosPointer");
        currTurnDir = state.getVectorVar("currTurnDir");
        selTurnDir = state.getVectorVar("selTurnDir");
        velUpdateDir = state.getVectorVar("velUpdateDir");        

        paused = false; 
    }
    
    public void continueWithState(AlgorithmState state) {
        useState(state);
    }

    public SearchAlgSpan getSearchSpan() {
        return searchSpan;
    }

    public void setSearchSpan(SearchAlgSpan searchSpan) {
        this.searchSpan = searchSpan;
    }
    
}
