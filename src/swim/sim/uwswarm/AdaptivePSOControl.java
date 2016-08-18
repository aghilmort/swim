
package swim.sim.uwswarm;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.scene.shape.Line;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.terrain.geomipmap.TerrainQuad;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.Vector;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.vecmath.Point3f;
import swim.algorithm.search.ARPSO;
import swim.algorithm.search.SearchAlgorithm;
import swim.core.AgentControl;
import swim.core.CommChannel;
import swim.core.CommState;
import swim.core.ContainerType;
import swim.core.Message;
import swim.core.MsgTransmitType;
import swim.core.SensorName;
import swim.core.SimpleMessage;
import swim.core.State;
import swim.core.network.Assignment;
import swim.core.network.Demand;
import swim.core.network.FlowModel;
import swim.core.network.ODPair;
import swim.core.network.PolynomialFlowModel;
import swim.core.network.PowerModel;
import swim.core.network.UWSensorNetwork;
import swim.core.network.algorithm.Router;
import swim.core.network.algorithm.ShortestPathRouter;
import swim.core.network.deployment.PlanarRandomNG;
import swim.core.network.err.InvalidLinkId;
import swim.core.network.err.InvalidNodeId;
import swim.core.network.misc.DoublePropertyMap;
import swim.sim.Simulator;
import swim.util.DebugUtils;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class AdaptivePSOControl extends AgentControl {
    
    // =========================================================================
    // Config
    // =========================================================================
    
    // Vehicle physical constraints
    private final float VEHICLE_SPEED = 10;
    
    private final float MIN_TURN_RADIUS = 0.5f,
                        MAX_TURN_ANGLE = FastMath.TWO_PI / 30,
                        MTA_ARC_LEN = MIN_TURN_RADIUS * MAX_TURN_ANGLE,
                        MTA_CHORD_LEN = 2*MIN_TURN_RADIUS*FastMath.cos(MAX_TURN_ANGLE/2),
                        VEH_TURNING_SPEED = 0.5f * MTA_CHORD_LEN,
                        MIN_VEH_SPEED = VEH_TURNING_SPEED,
                        MAX_VEH_SPEED = 10;
    
    
    private static int HALF_PLANE;   
    // Minimum permissible height
    private static Point3f MIN_PT;
    private static Point3f MAX_PT;
    private final float TURN_RADIUS = 0.1f;        // 0.01f
    private final float TURN_RESOLUTION = 25;
    
    private final ColorRGBA COMM_RANGE_COLOR = ColorRGBA.Green;
    private final ColorRGBA FRONT_SENS_RANGE_COLOR = ColorRGBA.Magenta;
    private final ColorRGBA BACK_SENS_RANGE_COLOR = ColorRGBA.Blue;
    private final ColorRGBA RIGHT_SENS_RANGE_COLOR = ColorRGBA.Orange;
    private final ColorRGBA LEFT_SENS_RANGE_COLOR = ColorRGBA.Cyan;
    private final ColorRGBA BOTTOM_SENS_RANGE_COLOR = ColorRGBA.Red;
    
    private final ColorRGBA SEND_COMM_LINK_COLOR = ColorRGBA.Blue,
                            RECEIVE_COMM_LINK_COLOR = ColorRGBA.Red,
                            IDLE_COMM_LINK_COLOR = ColorRGBA.Yellow,
                            DEBUD_PATH_COLOR = ColorRGBA.Black;
    private final float COMM_LINK_WIDTH = 2f;
    
    private final boolean 
            SENS_DEBUG = false,
            COMM_DEBUG = false,
            NET_DEBUG = true,
            VEHICLE_PATH_DEBUG = true,
            LOC_EST_DEBUG = false,
            VEL_EST_DEBUG = false,
            DEBUG_SELF_POS_HIST = false,
            DEBUG_SHOW_VEHICLE_NAME = false,
            DEBUG_SELF_BEST_ANCHOR = false;
    
    private int DEBUG_PATH_REPLOT = 1;
    
    
    private final Integer INTER_MSG_INTERVAL = 5000; // in milliseconds
    private final float RECEIVE_RATE = 1.0f/100.0f;  // in milliseconds

    private final int SENSOR_RESOLUTION = 30;
    private final int COMM_RESOLUTION = 200; //45
    
    private final int MAX_TERR_COLL_AVOID_TIME = 4000;
    private final int RAMP_DOWN_TIME = 2000;
    
    private Integer VEHICLE_ID;
    private String VEHICLE_NAME;
    
    // Wait time
    private final double MAX_WAIT_TIME = 10000;
    
    private final float MOTION_START_DEPTH = 20;
    
    private final int POSITION_HISTORY_lEN = 60;    // multiple of POS_RECORDS_TO_AVERAGE
    private final float POS_RECORDS_TO_AVERAGE = 5;
    private final int POS_HIST_BUFF_POLL_TIMES = (int)(POSITION_HISTORY_lEN / POS_RECORDS_TO_AVERAGE),
                      NUM_AVG_POSITIONS = POS_HIST_BUFF_POLL_TIMES;
    
    private final float SELF_POS_EST_ERROR = 0.05f;
    
    private final float TURN_TRIGGER_THRESHOLD = 2f;
    
    private final int ACCELERATION_LENGTH = 50;
    private final float MAX_ACCELERATION_PITCH = 0.9f,
                        DEFAULT_DECELERATION_PITCH = 0.08f;
    
    private final int NUM_SENSORS = 5;
    
    private final boolean FRONT_SENS_ENABLED = true,
            BACK_SENS_ENABLED = true,
            RIGHT_SENS_ENABLED = true,
            LEFT_SENS_ENABLED = true,
            BOTTOM_SENS_ENABLED = true;
    
    private final float SENSING_RANGE = 8f;
    private final float COMM_RANGE = 16f;
    
    private final boolean APPLY_UPWARDS_VELOCITY = false;
    private final float DEFAULT_UPWARDS_VELOCITY = 0.025f;
    
    private final float UPWARD_ACCELERATION = 0.01f;
    
    private final long INTER_SEARCH_TIME = 7000;
    private final long GRACE_PERIOD = 0;
    // =========================================================================
    // Config
    // =========================================================================
    
    // Other Controls used by UWVehicleControl
    private RigidBodyControl physicsVehicle;
    
    // FSM
    private State state;
    private boolean forward;
    private Spatial target;
    private float distToTarget;
    
    // Sensing
    private float sensFieldAngle = FastMath.QUARTER_PI;
    private Geometry[][] sightLines = new Geometry[NUM_SENSORS][SENSOR_RESOLUTION];
    
    private List<Spatial> auxSearchTargets = new ArrayList<Spatial>();
    private List<Spatial> ultimateSearchTargets = new ArrayList<Spatial>();
    private List<Spatial> vehicleTargets = new ArrayList<Spatial>();
    
    private float[][] sensorXAngles = new float[NUM_SENSORS][SENSOR_RESOLUTION];
    private float[][] sensorYAngles = new float[NUM_SENSORS][SENSOR_RESOLUTION];
    
    private boolean targetFound;
    private SensorName sensorName;
    
    private Integer iterPartCount = 0;
    private ArrayBlockingQueue<Integer> partCountHist;
    
    // Searching
    private Spatial particle;
    
    // Following
    private float followTurnRadius;
    private Vector3f neighborEstVel;
    private float neighborDistEst;
    private Vector3f estNeighborLoc;
    
    // Communication
    private float commFieldAngle = FastMath.TWO_PI;
    private Geometry[] commLines = new Geometry[COMM_RESOLUTION];
    private List<Spatial> commPartners = new ArrayList<Spatial>();
    private boolean vehiclesInRange;
    private boolean commInitiated = false;
    
    private float[] commXAngles = new float[COMM_RESOLUTION];
    private float[] commYAngles = new float[COMM_RESOLUTION];
    
    private boolean commSend = true;
    
    // Network
    private float[] linkLengths;
    private HashMap<String, Geometry> commLinkLines = new HashMap<String, Geometry>();
    // Wait times
    private double timeBeforeReBroadcast;
    private double timeBeforeReListening;
    private long lastBroadcastTime = 0, lastReceiveTime = 0;
    private CommChannel commChannel;
    
    // Networking
    private ArrayList<String> neighborList;
    private Long networkID;
    private CommState commState; 
    
    Vector<Integer> neighborNodes;
    Integer centerNodeID;
    
    private PhysicsSpace physicsSpace;    
    private int terrainSideLen;    
    private UWVehicle vehicle;    
    public Vector3f initialPosition;
    private Vector3f initialVelocity;    
    private Vector3f currVel; 
    private Boolean headingSet = false;
    
    // Moving
    private Vector3f normVelVec;
    private Quaternion yawXAngle, pitchYAngle, vehRot;
    private float angleWithZ, angleWithY;
    private Quaternion turnQuat;
    private float turnStepAngle;
    private Vector3f activeVelocity;
    private float activeSpeed;
    private boolean firstTimeForce = true;
    
    // Turning
    private boolean turningStarted = false;
    private int compositeTurnStepsDone = 0,
                compTurnStepInd = 0,
                turnStepsDone = 0;
    private float chordLen = 0;
    private float traveledDist, distToTravel;
    private Vector3f turnIncStartLoc, travelStartLoc;
    private boolean compositeTurnInProgress = false,
                    turnInProgress = false,
                    travelInProgress = false,
                    turnCompleted = false;
    
    private float vehSpeedSnapshot;
    
    private float turnAngle;
    private String turnDirection;
    
    private ArrayList<Pair<String, Object>> compositeTurnSeq;
    
    // Acceleration
    private boolean accelerateUpwards = false,
                    decelerate = false;
    private int accelStep = 0,
                decelStep = 0,
                decelerationLength = 0;
    private float activeAcceleration;

    // Multi-threading
    //The future that is used to check the execution status:
    private Future future = null;
    
    // Height changes
    private boolean ascending = false;
    private boolean descending = false;
    private long terrainCollAvoidTime, currRampDownTime;
    
    // App
    private Simulator sim;
    private AssetManager assetManager;
    
    // Position tracking
    private Vector3f initialSubmergePos;
    private ArrayBlockingQueue<Vector3f> positionHistory;
    private ArrayBlockingQueue<String> posMarkerHist;
    private Vector3f prevSelfPosition, estimatedSelfPos;
    private float distFromLastPosRecord = 0;
    private long lastPosRecordTime;
    private Vector3f prevActVelocity;
    
    // Optimization
    private Vector3f selfBestPointer;
    private Vector3f selfBestPos;
    private Integer selfBestSol;
    private Integer bestNeighborSol = 0;
    private Integer targetedWeight = 0;
    
    // =========================================================================
    // Search algorithm
    // =========================================================================
    SearchAlgorithm searchAlgorithm;
    private float searchTurnRadius, searchTurnAngle;
    private String searchTurnDir;
    private Vector3f searchUpdatePos, searchUpdateVelocity;
    private boolean turnNotNeeded = false;
    
    private long timeSinceLastSearchStep = 0,
                 gracePeriodStartTime = 0;
    
    private boolean inGracePeriod = false;
    
    
    // Debugging =======================================================
    private boolean sentOnce = false;
    private boolean receivedOnce = false;
    private boolean broadcast = true;
    private float totalTurnAngleSoFar = 0;
    private int vehPathCounter = DEBUG_PATH_REPLOT;
    // Debugging =======================================================
   
    
    // Improvement to the current process
    private Vector3f mostRecentNeighBestPos;
    
    private Vector3f inContainerVehPos;
    
        
    public AdaptivePSOControl(UWVehicle vehicle, BulletAppState bulletAppState, Simulator sim) {
        
        super();

        this.vehicle = vehicle;
        this.sim = sim;
        
        assetManager = sim.getAssetManager();

        physicsSpace = bulletAppState.getPhysicsSpace();

        terrainSideLen = (int)Math.floor((physicsSpace.getWorldMax().x - physicsSpace.getWorldMin().x));

        // =====================================================================        
        // Store terrain bounds 
        // =====================================================================
        
        MIN_PT = new Point3f(physicsSpace.getWorldMin().x, physicsSpace.getWorldMin().y, physicsSpace.getWorldMin().z);
        MAX_PT = new Point3f(physicsSpace.getWorldMax().x, physicsSpace.getWorldMax().y, physicsSpace.getWorldMax().z);
        HALF_PLANE = (int)(terrainSideLen/2);
        
        // =====================================================================      
        // Initialize other controls
        // =====================================================================
        
        physicsVehicle = new RigidBodyControl(vehicle.getMass());

        this.vehicle.getModel().addControl(physicsVehicle);
        bulletAppState.getPhysicsSpace().add(physicsVehicle);
        
        physicsVehicle.setKinematic(false);
        
        setInitLocation(ContainerType.box, new Vector3f(-50, 50, 30));
        
        float upwardsVel = (APPLY_UPWARDS_VELOCITY ? DEFAULT_UPWARDS_VELOCITY : 0);
        Vector3f tempVelVec = new Vector3f(0, upwardsVel * VEHICLE_SPEED, 1);
        Quaternion velRotQuat = new Quaternion();
        velRotQuat.fromAngleAxis(FastMath.rand.nextFloat() * FastMath.TWO_PI, Vector3f.UNIT_Y);
        Vector3f velVec = velRotQuat.mult(tempVelVec);
        velVec = velVec.mult(VEHICLE_SPEED);
        initialVelocity = velVec;
        
        this.vehicle.setVel(initialVelocity);
        activeVelocity = initialVelocity;
        
        // =====================================================================
        // Self position tracking
        // =====================================================================

        positionHistory = new ArrayBlockingQueue<Vector3f>(POSITION_HISTORY_lEN);
        posMarkerHist = new ArrayBlockingQueue<String>(POSITION_HISTORY_lEN);
        estimatedSelfPos = new Vector3f();
        estimateSelfPosition();
        prevSelfPosition = estimatedSelfPos;
        
        // =====================================================================
        // Auxiliary-targets count history
        // =====================================================================
        partCountHist = new ArrayBlockingQueue<Integer>(POSITION_HISTORY_lEN);
        
        // =====================================================================
        // Turning
        // =====================================================================
        compositeTurnSeq = new ArrayList<Pair<String, Object>>();
        // =====================================================================
        
        // =====================================================================
        // Sensor initialization
        // =====================================================================
        
        float angleWithActVel, angleAroundActVel;
        for(int i = 0; i < NUM_SENSORS; i++) {
            for(int j = 0; j < SENSOR_RESOLUTION; j++) {
                
                angleWithActVel  = FastMath.acos((float)( (Math.random() * (1 - FastMath.cos(sensFieldAngle/2))) + FastMath.cos(sensFieldAngle/2) ));
                angleWithActVel = (Math.random() > 0.5 ? angleWithActVel : -angleWithActVel);
                
                angleAroundActVel = 2 * FastMath.PI * (float)Math.random();
                angleAroundActVel = (Math.random() > 0.5 ? angleAroundActVel : -angleAroundActVel);
                
                sensorXAngles[i][j] = angleWithActVel;
                sensorYAngles[i][j] = angleAroundActVel;

            } 
        }
        
        // =====================================================================
        // Communication range initialization
        // =====================================================================
        
        float angleWithActVel2, angleAroundActVel2;
        for(int j = 0; j < COMM_RESOLUTION; j++) {

            angleWithActVel2  = FastMath.acos((float)( (Math.random() * (1 - FastMath.cos(commFieldAngle/2))) + FastMath.cos(commFieldAngle/2) ));                   
            angleWithActVel2 = (Math.random() > 0.5 ? angleWithActVel2 : -angleWithActVel2);
            
            angleAroundActVel2 = 2 * FastMath.PI * (float)Math.random();
            angleAroundActVel2 = (Math.random() > 0.5 ? angleAroundActVel2 : -angleAroundActVel2);

            commXAngles[j] = angleWithActVel2;
            commYAngles[j] = angleAroundActVel2;

        } 
        
        searchAlgorithm = new ARPSO(this, sim); 
    }
    
    @Override
    protected void controlUpdate(float tpf) {
        
        // Vehicle's state machine
        switch(state) {
            case Drop:
                //System.out.println("DROP state ...");
                if(physicsVehicle.getPhysicsLocation().y < MOTION_START_DEPTH) { 
                    move(tpf);
                    sense();
                    processData();
                    state = State.Search;
                }
                break;
            case Search:
                //System.out.println("SEARCH state ...");
                // Initialization
                estimateSelfPosition();
                selfBestPointer = null;
                selfBestPos = null;//estimatedSelfPos;
                selfBestSol = 0;
                searchAlgorithm.start();
                state = State.Update;
                break;  
            case Update:
                //System.out.println("Update State ...");
                searchAlgorithm.applyUpdateRules();
                
                if(turnNotNeeded) {
                    //System.out.println("Update entered 'turnNotNeeded'...");
                    state = State.Move;
                    turnNotNeeded = false;
                } else {
                    //System.out.println("Update triggered a turn ...");
                    state = State.Turn;
                }
                break;
            case Move:
                move(tpf);
                sense();
                processData();
                communicate();      // NEW
                state = State.Update;
                //timeSinceLastSearchStep = System.currentTimeMillis();
                break;
            case Turn:
                //System.out.println("TURN state ...");
                
//                if(System.currentTimeMillis() - timeSinceLastSearchStep > INTER_SEARCH_TIME) {
//                    compositeTurnInProgress = false;
//                }
                
                if(compositeTurnInProgress) {
                    
                    //System.out.println("Composite turn started ...");
                    
                    if( !turnInProgress && !travelInProgress ) {
                        
                        if( compositeTurnStepsDone < compositeTurnSeq.size() ) {
                            
    //                        System.out.println("Started turn sequence checking ...");

                            Pair<String,Object> currCompTurnStep = 
                                    (Pair<String,Object>)compositeTurnSeq.get(compTurnStepInd);

                            if(currCompTurnStep.key.equals("turn_direction")) {

    //                            System.out.println("Turn direction entered ...");

                                turnDirection = (String)currCompTurnStep.val;
                                compositeTurnStepsDone++;
                                currCompTurnStep = (Pair<String,Object>)compositeTurnSeq.get(++compTurnStepInd);

                                if(currCompTurnStep.key.equals("turn_angle")) {
    //                               System.out.println("Turn angle entered ...");
                                   turnAngle = (Float)currCompTurnStep.val; 
                                   turnInProgress = true;
                                } else {
                                    System.out.println("No turn angle specified for the current turn step!");
                                    System.exit(-1);
                                }

                            } else if(currCompTurnStep.key.equals("travel")) {
    //                            System.out.println("Travel entered ...");
                                distToTravel = (Float)currCompTurnStep.val;
                                travelStartLoc = physicsVehicle.getPhysicsLocation();
                                traveledDist = 0;   // shared between turn and travel
                                travelInProgress = true;
                            } else {
                                System.out.println("Unsupported turn-sequence step!");
                                System.exit(-1);
                            }

                            compositeTurnStepsDone++;
                            compTurnStepInd++;
                    
                        } else {
                            compositeTurnInProgress = false;
                        }
                    
                    }

                    //System.out.println("Turn steps done: "+turnStepsDone);
                    if(turnInProgress) {
//                        System.out.println("Turning in progress ...");
                        if(!turningStarted) {
                            turningStarted = true;
                            
                            //System.out.println("Going to start turning ...");
                            turn(turnDirection, turnAngle, tpf);
                        }
                        traveledDist = turnIncStartLoc.distance(physicsVehicle.getPhysicsLocation());
                        //System.out.println("Traveled Dist: "+traveledDist);
                        if(traveledDist < chordLen) {
                            move(tpf);
                            sense();
                            processData();
                            communicate();      // NEW
                        } else if(turnCompleted){
                            //System.out.println("turn completed ... ("+vehicle.getName()+")");
                            move(tpf);
                            sense();
                            processData();
                            communicate();      // NEW
                            resetTurnFlags();
                            
                            break;
                        } else {
                            turnStepsDone++;
                            turn(turnDirection, turnAngle, tpf);  
                            move(tpf);
                            sense();
                            processData();
                            communicate();      // NEW
                        }
                        
                    } else if(travelInProgress) {
                        
                        traveledDist = travelStartLoc.distance(physicsVehicle.getPhysicsLocation());
//                        System.out.println("Traveled Dist: "+traveledDist+", Distance to travel: "+distToTravel);
                        if(traveledDist < distToTravel) {
                            move(tpf);
                            sense();
                            processData();
                            communicate();      // NEW
                        } else {
                            //System.out.println("travel completed ... ("+vehicle.getName()+")");
                            move(tpf);
                            sense();
                            processData();
                            communicate();      // NEW
                            resetTravelFlags();
                        }
                        
                    } else {
                        move(tpf);
                        sense();
                        processData();
                        communicate();      // NEW
                    }
                    
                    state = State.Update;   // NEW 2
                    
                } else {
                    move(tpf);
                    sense();
                    processData();
                    communicate();      // NEW
                    resetCompositeTurnFlags();
                    
                    // Update speed
                    activeSpeed = searchUpdateVelocity.length();
                    activeVelocity = activeVelocity.normalize().mult(activeSpeed);
                    limitVehicleSpeed();
                    
                    state = State.Update;
                }
                break;
            case Follow:
                follow(followTurnRadius, neighborEstVel, tpf);
                break;
            case Idle:
                break;
        }
        
    }
    
    private void communicate() {
        if(commSend) {
           broadcastSelfBestPos();
        } else {
            mostRecentNeighBestPos = getNeighborhoodBestPosition(); 
        }
        commSend = !commSend;
        commInitiated = false;
    }
    
    public void setCommInitiated(boolean val) {
        commInitiated = val;
    }
    
    private void resetTurnFlags() {
        turnInProgress = false;
        turnStepsDone = 0;
        turningStarted = false;
        totalTurnAngleSoFar = 0;
        turnCompleted = false;
        traveledDist = 0;   // shared between turn and travel
    }
    
    private void resetTravelFlags() {
        travelInProgress = false;
        distToTravel = 0;
        traveledDist = 0;   // shared between turn and travel
    }
    
    public void resetCompositeTurnFlags() {
        compositeTurnInProgress = false;
        compositeTurnStepsDone = 0;
        compTurnStepInd = 0;
        compositeTurnSeq.clear();
        turnNotNeeded = false;
        resetTurnFlags();
        resetTravelFlags();
        inGracePeriod = true;
        gracePeriodStartTime = System.currentTimeMillis();
    }
    
    public boolean isCompTurnInProgress() {
        return compositeTurnInProgress;
    }
    
    private void resetGracePeriodVars() {
        inGracePeriod = false;
        gracePeriodStartTime = 0;
    }
    
    private void limitVehicleSpeed() {
        
        activeSpeed = activeVelocity.length();
        
        if( activeSpeed < MIN_VEH_SPEED) {
            activeSpeed = MIN_VEH_SPEED;
        } else if( activeSpeed > MAX_VEH_SPEED) {
            activeSpeed = MAX_VEH_SPEED;
        }
        
        activeVelocity = activeVelocity.normalize().mult(activeSpeed);
    }
    
    @Override
    public void move(float tpf) {

        limitVehicleSpeed();
        
        if(accelerateUpwards) {
            accelerateUpwards(UPWARD_ACCELERATION);
        }
        
        if(decelerate) {
            decelerate();
        }
        
        normVelVec = activeVelocity.normalize();
        if(VEHICLE_PATH_DEBUG && vehPathCounter == DEBUG_PATH_REPLOT) {
            DebugUtils.plotArrow( physicsVehicle.getPhysicsLocation(), normVelVec, DEBUD_PATH_COLOR, 1f, sim);
            vehPathCounter = 0;
        }
        vehPathCounter++;
        
        yawXAngle = new Quaternion();
        pitchYAngle = new Quaternion();
        vehRot = new Quaternion();
        
        angleWithZ = normVelVec.angleBetween(Vector3f.UNIT_Z);
        angleWithY = normVelVec.angleBetween(Vector3f.UNIT_Y);

        Vector3f vehPitchAxis = Vector3f.UNIT_Y.cross(normVelVec);
        
        if( (normVelVec.x < 0 && normVelVec.z < 0) || 
                (normVelVec.x < 0 && normVelVec.z > 0)  ) {
            angleWithZ = - angleWithZ;
        }
        
        yawXAngle.fromAngleAxis(angleWithZ, Vector3f.UNIT_Y);

        pitchYAngle.fromAngleAxis(angleWithY - FastMath.HALF_PI, vehPitchAxis);

        vehRot.set(pitchYAngle);
        vehRot.multLocal(yawXAngle);
            
        physicsVehicle.setPhysicsRotation(vehRot);
        
        physicsVehicle.setLinearVelocity(activeVelocity);
        keepInBounds();  
        
        // Fake-it approach
        estimateSelfPosition();
        
//        if( DEBUG_SELF_BEST_ANCHOR && selfBestPointer != null ) {
//            DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), selfBestPointer.subtract(physicsVehicle.getPhysicsLocation()), ColorRGBA.Red, 2, sim);
//        }
    }
    
    
    public void processData() {
        
        // =====================================================================
        // Self-best "anchor" construction - START
        // =====================================================================
        
        /* Check whether position buffer is full. It was chosen to have a length
           of "X" so that we can divide it into "Y = X/5" 5-point segments. The average
           of particle counts and positions of each of these 5 points are used to 
           get "Y" particle count and position averages. These new "Y" points are then used
           to contruct "Y-1" vectors (differences between each successive pair of points).
           The "anchor" pointing back to the self best solution (used in PSO) can be of 
           any length ranging from 0-(Y-1) based on where the best particle count, among the 
           "Y" points, is located back in the sequence.
        
        Default: X = 15, Y = 3
        */
        if( positionHistory.size() == POSITION_HISTORY_lEN && 
                partCountHist.size() == POSITION_HISTORY_lEN) {
            
            // Will hold the new "Y" points
            ArrayList<Pair<Vector3f, Integer>> Solutions = new ArrayList<Pair<Vector3f, Integer>>(NUM_AVG_POSITIONS);
            
            // Keeps track of the max. count so far and the position of the current max.
            int oldMaxCount = 0, maxCount = 0, maxCountPos = 0;
            boolean betterMax = false;
            
            // Poll the buffer "Y" times to empty it
            for(int k = 0; k < POS_HIST_BUFF_POLL_TIMES; k++) {
                
                // extract every 5 successive points and find their average position
                // and average particle count
                int i = 0;
                Vector3f averagePos = new Vector3f(0,0,0);
                double averagePartCount = 0;
                int intAvgPartCount;
                
                while(i < POS_RECORDS_TO_AVERAGE) {
                    averagePos = averagePos.add(positionHistory.poll());
                    averagePartCount += partCountHist.poll();
                    i++;
                }
                averagePos = averagePos.divide(POS_RECORDS_TO_AVERAGE);
                averagePartCount /= POS_RECORDS_TO_AVERAGE;
                intAvgPartCount = (int)Math.round(averagePartCount);
                
//                System.out.println(spatial.getName()+"'s average position ("+(k+1)+"): "+averagePos.toString());
//                System.out.println(spatial.getName()+"'s average particle count ("+(k+1)+"): "+intAvgPartCount);
                
                // Update current max. and max. count position               
                betterMax = ( intAvgPartCount > oldMaxCount );  
                if(betterMax) { oldMaxCount = intAvgPartCount; }
                if( betterMax || intAvgPartCount == oldMaxCount ) {
                    maxCountPos = k;
                }
                
                // If a better max, prune all previous points and add the new one
                if( (betterMax || intAvgPartCount == oldMaxCount) && Solutions.size() > 0) {
                    while(Solutions.size() > 0) {
                       Solutions.remove(0);
                    }
                }
                
                // Store this solution in the list of solutions
                if(averagePartCount > 0) {
                    //System.out.println("Average particle count: " + averagePartCount);
                    Pair sol = new Pair(averagePos, intAvgPartCount);
                    Solutions.add(sol);
                }

            }
            
            //System.out.println(spatial.getName()+"'s Solution size: "+Solutions.size());
            
            if(Solutions.size() > 0) {
                // Self-best pointer construction
                Vector3f origin = Solutions.get(Solutions.size() - 1).key;
                Vector3f dest = (Solutions.get(0).key).subtract(origin); 
                selfBestPointer = ( (int)Solutions.get(0).val == 0 ? null : Solutions.get(0).key );
                selfBestPos = ( (int)Solutions.get(0).val == 0 ? null : Solutions.get(0).key );
                selfBestSol = ( (int)Solutions.get(0).val == 0 ? null : (int)Solutions.get(0).val );

                if(DEBUG_SELF_BEST_ANCHOR) {
                    // Construct self-best "anchor"
                    DebugUtils.plotArrow(origin, dest, ColorRGBA.Black, 2, sim);
                }
            }
            
        }
            
        // =====================================================================
        // Self-best "anchor" construction - END
        // =====================================================================
        
    }
    
    private void estimateSelfPosition() {
        
        Vector3f actualLoc = physicsVehicle.getPhysicsLocation();
        
        //System.out.println("Vehicle's Physics Location: " + physicsVehicle.getPhysicsLocation().toString());

        float xError = (float) Math.random() * actualLoc.x * SELF_POS_EST_ERROR;
        float yError = (float) Math.random() * actualLoc.y * SELF_POS_EST_ERROR;
        float zError = (float) Math.random() * actualLoc.z * SELF_POS_EST_ERROR;
        
        estimatedSelfPos.setX(actualLoc.x + (Math.random() > 0.5 ? xError : -xError));
        estimatedSelfPos.setY(actualLoc.y + (Math.random() > 0.5 ? yError : -yError));
        estimatedSelfPos.setZ(actualLoc.z + (Math.random() > 0.5 ? zError : -zError));
        
        String oldCrossHairID = "", newCrossHairID = "";
        
        if(DEBUG_SELF_POS_HIST) {
            
            //System.out.println(posMarkerHist.toString());
            if( posMarkerHist.size() == POSITION_HISTORY_lEN ) {
                oldCrossHairID = (String)posMarkerHist.poll();
                DebugUtils.removeCrossHair(oldCrossHairID, sim);
            } 
            newCrossHairID = DebugUtils.plotCrossHair(estimatedSelfPos.clone(), 2, 0.5f, ColorRGBA.Red, sim);
            posMarkerHist.offer(newCrossHairID);
                      
        }
        
        if( positionHistory.size() == POSITION_HISTORY_lEN ) {
            positionHistory.poll();
        }

        // Push it to into history
        positionHistory.offer(estimatedSelfPos.clone());
        
        //System.out.println("Position History of vehicle (): " + positionHistory.toString());
        
    }
    
    private void estimatePervPosParams() {
        Vector3f dirToPrevPosEst = estimatedSelfPos.addLocal(prevSelfPosition).normalize();
    }
    
    private boolean sense() {
        
        int northParticlesCount = 0,
            southParticlesCount = 0,
            eastParticlesCount = 0,
            westParticlesCount = 0;
        ArrayList<String> northPartNames = new ArrayList<String>(),
                          southPartNames = new ArrayList<String>(),
                          eastPartNames = new ArrayList<String>(),
                          westPartNames = new ArrayList<String>();
        iterPartCount = 0;
        
        // Create four sensors
        for(int i = 0; i < NUM_SENSORS; i++) {
            
            boolean skipSensor = false;
            if( !FRONT_SENS_ENABLED  && i == 0 ||
                !BACK_SENS_ENABLED   && i == 1 ||
                !RIGHT_SENS_ENABLED  && i == 2 ||
                !LEFT_SENS_ENABLED   && i == 3 ||
                !BOTTOM_SENS_ENABLED && i == 4 ) {
                skipSensor = true;
            }
            
            if(skipSensor) { continue; }
            
            // Shared vars
            Vector3f rayOrigin = new Vector3f();
            Quaternion aimDirection = new Quaternion();
            Vector3f rayDirection = new Vector3f();
            ColorRGBA lineColor = ColorRGBA.Yellow;
            
            Vector3f auxVec = new Vector3f();
            Quaternion auxQuat = new Quaternion();

            int j = 0;
            targetFound = false;

            float angleWithActVel, angleAroundActVel;
            
            for(int r = 0; r < SENSOR_RESOLUTION; r++) { 
                
                // Calculate ray origin
                switch(i) {
                    case 0:
                        rayOrigin = spatial.getWorldTranslation().add(activeVelocity.normalize().mult(0.8f));
                        rayDirection.set(activeVelocity.normalize());
                        lineColor = FRONT_SENS_RANGE_COLOR;
                        sensorName = SensorName.NorthSensor;
                        break;
                    case 1:
                        rayOrigin = spatial.getWorldTranslation().add(activeVelocity.normalize().negate().mult(0.8f));
                        rayDirection.set(activeVelocity.normalize().negate());
                        lineColor = BACK_SENS_RANGE_COLOR;
                        sensorName = SensorName.SouthSensor;
                        break;
                    case 2:
                        rayOrigin = spatial.getWorldTranslation().add(activeVelocity.normalize().cross(Vector3f.UNIT_Y).mult(0.2f));
                        rayDirection.set(activeVelocity.normalize().cross(Vector3f.UNIT_Y));
                        lineColor = RIGHT_SENS_RANGE_COLOR;
                        sensorName = SensorName.EastSensor;
                        break;
                    case 3:
                        rayOrigin = spatial.getWorldTranslation().add(Vector3f.UNIT_Y.cross(activeVelocity.normalize()).mult(0.2f));
                        rayDirection.set(Vector3f.UNIT_Y.cross(activeVelocity.normalize()));
                        lineColor = LEFT_SENS_RANGE_COLOR;
                        sensorName = SensorName.WestSensor;
                        break;     
                    case 4:
                        rayOrigin = spatial.getWorldTranslation().add(Vector3f.UNIT_Y.negate().mult(0.2f));

                        Vector3f normalVec = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
                        float angWithY = activeVelocity.normalize().angleBetween(Vector3f.UNIT_Y);
                        float pitchAngle = FastMath.abs(FastMath.HALF_PI - angWithY);
                        Quaternion rayRotQuat = new Quaternion();
                        rayRotQuat.fromAngleAxis(pitchAngle, normalVec);
                        rayDirection.set(Vector3f.UNIT_Y.negate());
                        rayDirection = rayRotQuat.mult(rayDirection);
                        
                        lineColor = BOTTOM_SENS_RANGE_COLOR;
                        sensorName = SensorName.BottomSensor;
                        break;  
                }    
                
                auxVec = rayDirection.clone();
                
                if(SENS_DEBUG && sightLines[i][j] != null){
                    ((Node)getSpatial().getParent()).detachChild(sightLines[i][j]);
                }

                // =============================================================
                
                angleWithActVel = sensorXAngles[i][r];
                angleAroundActVel = sensorYAngles[i][r];
                
                if(sensorName.equals(SensorName.NorthSensor) || 
                   sensorName.equals(SensorName.SouthSensor)) {
                    
                    Vector3f perpenVec = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
                    Quaternion rotateFromActVel = new Quaternion();
                    rotateFromActVel.fromAngleAxis(angleWithActVel, perpenVec.normalize());

                    Quaternion rotateAroundActVel = new Quaternion();
                    rotateAroundActVel.fromAngleAxis(angleAroundActVel , activeVelocity.normalize());

                    aimDirection = rotateAroundActVel.mult(rotateFromActVel);
                    
                } else if(sensorName.equals(SensorName.EastSensor) || 
                          sensorName.equals(SensorName.WestSensor)) {
                    
                   Vector3f sensDirVec = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
                   
                   Vector3f rotVec = activeVelocity.normalize().negate();
                   Quaternion rotateFromSensDir = new Quaternion();
                   rotateFromSensDir.fromAngleAxis(angleWithActVel, rotVec);
                    
                   Quaternion rotateAroundSensDir = new Quaternion();
                   rotateAroundSensDir.fromAngleAxis(angleAroundActVel , sensDirVec.normalize());
                    
                   aimDirection = rotateAroundSensDir.mult(rotateFromSensDir);
                    
                } else if(sensorName.equals(SensorName.BottomSensor)) {
                    
                   Vector3f sensDirVec = Vector3f.UNIT_Y.negate();
                   
                   Vector3f normalVec = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
                   float angWithY = activeVelocity.normalize().angleBetween(Vector3f.UNIT_Y);
                   float pitchAngle = FastMath.abs(FastMath.HALF_PI - angWithY);
                   Quaternion rayRotQuat = new Quaternion();
                   rayRotQuat.fromAngleAxis(pitchAngle, normalVec);
                   sensDirVec = rayRotQuat.mult(sensDirVec);
                   
                   Quaternion rotateFromSensDir = new Quaternion();
                   rotateFromSensDir.fromAngleAxis(angleWithActVel, normalVec);
                    
                   Quaternion rotateAroundSensDir = new Quaternion();
                   rotateAroundSensDir.fromAngleAxis(angleAroundActVel , sensDirVec.normalize());
                    
                   aimDirection = rotateAroundSensDir.mult(rotateFromSensDir);
                    
                }

                // =============================================================
                
                aimDirection.multLocal(rayDirection);

                Ray ray = new Ray(rayOrigin, rayDirection);
                ray.setLimit(SENSING_RANGE);

                // Collision with particles
                CollisionResults partColRes = new CollisionResults();
                for(Spatial s: auxSearchTargets){
                    s.collideWith(ray, partColRes);
                }

                if(partColRes.size() > 0){
                    
                    Iterator colls = partColRes.iterator();
                    CollisionResult coll;
                    String pName;
                    while( colls.hasNext() ) {
                        coll = (CollisionResult)colls.next();
                        particle = coll.getGeometry();
                        
                        //System.out.println("P_NAME: "+particle.getParent().getParent().getName());
                        pName = particle.getParent().getParent().getName();
                        
                         if( pName.substring(0, 9).equals("particle_") ) {  
                            
                            switch(i) {
                                case 0:
                                    if(!northPartNames.contains(pName)) {
                                        northPartNames.add(pName);
                                        northParticlesCount++;
//                                        System.out.println("Particle count for "+spatial.getName()+"'s NORTH sensor is: "+northParticlesCount);
                                    }
                                    break;
                                case 1:
                                    if(!southPartNames.contains(pName)) {
                                        southPartNames.add(pName);
                                        southParticlesCount++;
//                                        System.out.println("Particle count for "+spatial.getName()+"'s SOUTH sensor is: "+southParticlesCount);
                                    }
                                    break;
                                case 2:
                                    if(!eastPartNames.contains(pName)) {
                                        eastPartNames.add(pName);
                                        eastParticlesCount++;
//                                        System.out.println("Particle count for "+spatial.getName()+"'s EAST sensor is: "+eastParticlesCount);
                                    }
                                    break;
                                case 3:
                                    if(!westPartNames.contains(pName)) {
                                        westPartNames.add(pName);
                                        westParticlesCount++;
//                                        System.out.println("Particle count for "+spatial.getName()+"'s WEST sensor is: "+westParticlesCount);
                                    }
                                    break;
                            }
                         }
                    }
                }
                
                // Collisions with terrain
                if(sensorName.equals(SensorName.BottomSensor)) {
                    
                    CollisionResults terrColRes = new CollisionResults();

                    List<Spatial> terrainQuads = ((TerrainQuad)((Node)sim.getTerrain()).getChild("terrain")).getChildren();

                    for(Spatial s : terrainQuads) {
                        //System.out.println( ((TerrainQuad)s).toString() );
                        ((TerrainQuad)s).collideWith(ray, terrColRes);
                    }     

                    if(terrColRes.size() > 0){

                        CollisionResult hit = terrColRes.getClosestCollision();

                        Vector3f closestTerrainPoint = hit.getContactPoint();

                        if( physicsVehicle.getPhysicsLocation().distance(closestTerrainPoint) < SENSING_RANGE ) {
                            accelerateUpwards = true;
                        }

                    }
                }
                
                // Collisions with one of the ultimate targets
                CollisionResults ultimateTargetColRes = new CollisionResults();
                
                for(Spatial s: ultimateSearchTargets){
                    s.collideWith(ray, ultimateTargetColRes);
                }

                if(ultimateTargetColRes.size() > 0){
                    
                    CollisionResult hit = ultimateTargetColRes.getClosestCollision();
                    Vector3f closestTarget = hit.getContactPoint();
                    
                    if( physicsVehicle.getPhysicsLocation().distance(closestTarget) < SENSING_RANGE ) {
                        activeVelocity = new Vector3f(0,0,0);
                        sim.notifyTargetFound();
                    }
                    break;
                }
                
                if(SENS_DEBUG){
                    Geometry line = makeDebugLine(ray, SENSING_RANGE, lineColor);
                    sightLines[i][j++] = line;
                    ((Node)getSpatial().getParent()).attachChild(line);
                }
            }            
        }
        
        iterPartCount = northParticlesCount + southParticlesCount + eastParticlesCount + westParticlesCount;
        partCountHist.offer(iterPartCount);

        return targetFound;
        
    }
    
    public void ascend() {
        ascending = true;
    }
    
    public void descend() {
        
    }

    private void rampUp() {
        activeVelocity = activeVelocity.setY(activeVelocity.getY() + 0.00001f);
        terrainCollAvoidTime = System.currentTimeMillis();
    }
    
    private void rampDown() {
        activeVelocity = activeVelocity.setY(activeVelocity.getY() - 0.00001f);
        terrainCollAvoidTime = System.currentTimeMillis();
    }
    
    private void accelerateUpwards(float acceleration) {
        if(acceleration > MAX_ACCELERATION_PITCH) {
            acceleration = MAX_ACCELERATION_PITCH;
        }
        if(accelStep < ACCELERATION_LENGTH) {
            activeVelocity = activeVelocity.setY(activeVelocity.getY() + acceleration * activeVelocity.length());
            accelStep++;
        } else {           
            resetAccelerationFlags();
            decelerate = true;
            decelStep = 0;
            
            activeAcceleration = acceleration;
            
            float currYVel = activeVelocity.getY();
            
            if(APPLY_UPWARDS_VELOCITY) {
                decelerationLength = (int)FastMath.ceil((currYVel - DEFAULT_UPWARDS_VELOCITY) / DEFAULT_DECELERATION_PITCH);
            } else {
                decelerationLength = (int)FastMath.ceil(currYVel / DEFAULT_DECELERATION_PITCH);
            }
           
            decelerate();
        }
    }
    
    private void decelerate() {
        
        if(decelerationLength < 0) {
            decelerate = false;
            return;
        }
        
        if(decelStep < decelerationLength || (APPLY_UPWARDS_VELOCITY && activeVelocity.getY() > DEFAULT_UPWARDS_VELOCITY) || activeVelocity.getY() > 0) {
            activeVelocity = activeVelocity.setY(activeVelocity.getY() - activeAcceleration);
            decelStep++;
        } else {
            if(activeVelocity.getY() < 0) {
                activeVelocity.setY(0);
            }
            resetDeceleration();
        }
    }
    
    private void resetAccelerationFlags() {
        accelStep = 0;
        accelerateUpwards = false;
    }
    
    private void resetDeceleration() {
        decelStep = 0;
        decelerate = false;
        decelerationLength = 0;
    }
    
    public void setCurrTargetedWeight(Integer targettedWeight) {
        this.targetedWeight = targetedWeight;
    }
    
    public Integer getCurrTargetedWeight() {
        return targetedWeight;
    }
    
    public void reorient(Vector3f newOrientationDir) {
        
        // Normalize the direction vector
        Vector3f normDirVec = newOrientationDir.normalize();
        
        // Find a vector perpendicular to activeVelocity and Vector3f.UNIT_Y
        Vector3f perpenVec = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
        
        // Find the target direction's component in the direction of activeVelocity
        float actVelComp = activeVelocity.normalize().dot(normDirVec);

        // Find target direction's component in the direction perpendiular to activeVelocity
        float actVelPerpenComp = perpenVec.dot(normDirVec);
        
        //System.out.println("Active Velocity Component: "+actVelComp);
        //System.out.println("Active Velocity Perpen Component: "+actVelPerpenComp);
        
        // Determine the quadrant containing the vector or the axis it is 
        // aligned with based on the two components
        String quadrant = "", axis = "";
        float compProduct = actVelComp * actVelPerpenComp;
        if(compProduct == 0) {
            if(actVelComp == 0 && actVelPerpenComp == 0) {
                return; // nothing needs to be done (length of target orientation vector is 0)
            } else if(actVelComp == 0) {
                axis = "+vePerpenAxis";
                if(actVelPerpenComp < 0) { axis = "-vePerpenAxis"; }
            }  else if(actVelPerpenComp == 0) {
                axis = "+veActVelAxis";
                if(actVelComp < 0) { axis = "-veActVelAxis"; }
            } 
        }
        quadrant = (compProduct > 0 ? "first" : "second");
        if(quadrant.equals("first") && actVelComp < 0) { quadrant = "third"; }
        if(quadrant.equals("second") && actVelComp < 0) { quadrant = "fourth";}
        
        //System.out.println("Axis: "+axis);
        //System.out.println("Quadrant: "+quadrant);
        
        // Make the re-orientation turn based on the quadrant/axis the orientation
        // vector lies in/on
        if(axis.equals("+veActVelAxis")) {
            return; // no turn needed; just keep going straight
        } else if(axis.equals("-veActVelAxis")) {   // re-orient towards backward direction
            
            compositeTurnSeq.add(new Pair<String,Object>("travel", 2 * MIN_TURN_RADIUS));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", "right"));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", 1.5f * FastMath.PI));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", "left"));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", 0.5f * FastMath.PI));
            
            return; 

        } else if(axis.equals("+vePerpenAxis")) {   // re-orient towards vehicle's east direction
            
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", "left"));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", 1.5f * FastMath.PI));
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS));
            
            return; 
            
        } else if(axis.equals("-vePerpenAxis")) {   // re-orient towards vehicle's west direction
            
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", "right"));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", 1.5f * FastMath.PI));
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS));  
            
            return; 
        }
        
        // If the new orientation is not along any of the axes, start checking quadrants:
        
        if(quadrant.equals("first") || quadrant.equals("second")) {
            
            String firstDir = "left", secDir = "right";
            if(quadrant.equals("second")) {
                firstDir = "right";
                secDir = "left";
            }
            
            // Find the angle between the orientation vector and active velocity
            float theta = normDirVec.angleBetween(activeVelocity.normalize());
            
            // Calculate the distance the vehicle needs to travel before turning
            float delta =  MIN_TURN_RADIUS * ( 1 - (FastMath.cos(theta) / (1 + FastMath.sin(theta))));
            
            //System.out.println("Theta: "+(theta*FastMath.RAD_TO_DEG));
            //System.out.println("Delta: "+delta);
            
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", firstDir));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.HALF_PI));
            compositeTurnSeq.add(new Pair<String,Object>("travel", delta));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", firstDir));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.HALF_PI));
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", firstDir));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", FastMath.PI));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", secDir));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", theta));
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS - delta));
            
        } else if(quadrant.equals("third") || quadrant.equals("fourth")) {
            
            String firstDir = "right", secDir = "left";
            Vector3f perpenVecToUse = perpenVec.negate();
            if(quadrant.equals("fourth")) {
                firstDir = "left";
                secDir = "right";
                perpenVecToUse = perpenVec;
            }
            
            // Find the angle between the orientation vector and the perpendicular vector
            // to active velocity
            float theta = normDirVec.angleBetween(perpenVecToUse);
            
            // Calculate the distance the vehicle needs to travel before turning
            float delta = MIN_TURN_RADIUS * ( 1 - (FastMath.cos(theta) / (1 + FastMath.sin(theta))));
            
            //System.out.println("Theta: "+(theta*FastMath.RAD_TO_DEG));
            //System.out.println("Delta: "+delta);
            
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS + delta));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", firstDir));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", 1.5f * FastMath.PI));
            compositeTurnSeq.add(new Pair<String,Object>("turn_direction", secDir));
            compositeTurnSeq.add(new Pair<String,Object>("turn_angle", theta));       
            compositeTurnSeq.add(new Pair<String,Object>("travel", MIN_TURN_RADIUS - delta));  
            
        }  
    }
    
    /**
     * Takes a target relative direction (to the vehicle) and orients
     * the vehicle onto the that direction through a shortest path turn
     * based on the minimum turn radius
     * @param turnDirection
     * @param targetTurnAngle
     * @param tpf 
     */
    public void turn(String turnDirection, float targetTurnAngle, float tpf) {

//        System.out.println("Passed target turn angle: "+(targetTurnAngle*FastMath.RAD_TO_DEG));
//        System.out.println("Passed turn direction: "+turnDirection);

        turnStepAngle = MAX_TURN_ANGLE;
        chordLen = MTA_CHORD_LEN;
        
        if(totalTurnAngleSoFar + turnStepAngle > targetTurnAngle) {
            turnStepAngle -= (totalTurnAngleSoFar + turnStepAngle - targetTurnAngle);
            chordLen = 2 * MIN_TURN_RADIUS * FastMath.cos(turnStepAngle/2);
            //chordLen = MIN_TURN_RADIUS * turnStepAngle;
            turnCompleted = true;
        }
        
        totalTurnAngleSoFar += turnStepAngle;
        
//        System.out.println("Total turn angle so far: "+totalTurnAngleSoFar*FastMath.RAD_TO_DEG);   
//        System.out.println("Turn step angle: "+(turnStepAngle*FastMath.RAD_TO_DEG));
//        System.out.println("Chord length: "+chordLen);      
//        System.out.println("Turn angle: "+(turnAngle*FastMath.RAD_TO_DEG));
//        System.out.println("Chord length: "+chordLen);
        
        if(!turnDirection.equals("left")) {
            turnStepAngle = - turnStepAngle;
        }
        
        turnQuat = new Quaternion();  
        turnQuat.lookAt(activeVelocity, Vector3f.UNIT_Y);
        turnQuat.fromAngleAxis(turnStepAngle, Vector3f.UNIT_Y);

        //System.out.println("Active Velocity (BEFORE): "+activeVelocity);

        activeVelocity = turnQuat.mult(activeVelocity);
        turnIncStartLoc = physicsVehicle.getPhysicsLocation();

        //System.out.println("Active Velocity (AFTER): "+activeVelocity);

        traveledDist = 0;
    }
    
    public void estimateNeighborPosition(String neighborName, Vector3f minRay, Vector3f maxRay) {
        
        //System.out.println("Neighbor Name: "+neighborName);
        
        Vector3f actNeighborLoc = sim.getUWVehiclesMap().get(neighborName).getPhysicsVehicle().getPhysicsLocation();
        float actNeighborDist = physicsVehicle.getPhysicsLocation().distance(actNeighborLoc);
        
        float distError = (float) Math.random() * 0.2f * actNeighborDist; // If actual is 50, estimated dist is 50-60
        distError = (Math.random() > 0.5 ? distError : -distError);       // add it or subtract it: 40-60
        
        neighborDistEst = actNeighborDist + distError;
        
        // Pick a ray lying between min and max rays ( a random linear combination of the two)
        float minRayWeight = (float)Math.random(); // a fraction less than 1
        Vector3f posEstRay = minRay.mult(minRayWeight).add(maxRay.mult(1-minRayWeight));
        
        // Find estimated neighbor position
        Vector3f relativeNeighPos = posEstRay.mult(neighborDistEst);
        
        estNeighborLoc = physicsVehicle.getPhysicsLocation().add(relativeNeighPos);
        
        if(LOC_EST_DEBUG) {
            DebugUtils.plotCrossHair(estNeighborLoc, 2, 0.5f, ColorRGBA.Green, sim);
        }

    }
    
    public Vector3f estimateNeighborVelocity(String neighborName) {
        
        Vector3f nieghborVel = sim.getUWVehiclesMap().get(neighborName).getPhysicsVehicle().getLinearVelocity();
        
        float velMag = nieghborVel.length();
        Vector3f velDir = nieghborVel.normalize();
        
        float errorX = (float) Math.random() * 0.2f * velMag;     // If actual is 30, estimated is 30-36
        errorX = (Math.random() > 0.5 ? errorX : -errorX);        // add it or subtract it: 24-36
        
        float errorZ = (float) Math.random() * 0.2f * velMag;
        errorZ = (Math.random() > 0.5 ? errorZ : -errorZ);
        
        float xComp = velDir.x + errorX;
        float zComp = velDir.z + errorZ;
        
        Vector3f estimatedVel = new Vector3f(xComp, velDir.y, zComp);
        
        if (VEL_EST_DEBUG) {
            DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), estimatedVel, ColorRGBA.White, 2, sim);
        }
        
        return estimatedVel;
    }
    
    public float estimateTurnRadius(Vector3f targetPos) {
        
        // Default turn radius
        float turnRadius;
        
        // Create a vector pointing from vehicle to target position
        Vector3f vecToTargetPos = targetPos.subtract(physicsVehicle.getPhysicsLocation());
        
        // Find distance to target position
        float distToTargetPos = vecToTargetPos.length();
        
        // Find a vector perpendicular to the vehicle's velocity direction and the 
        // Y-axis ( lies in the x-z plane )
        Vector3f perpenVector = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
        perpenVector = perpenVector.normalize();
        
        // Find the angle between the vector pointing to target position and the,
        // just found, perpendicular vector
        float angleWithPerpen = perpenVector.angleBetween(vecToTargetPos.normalize());
        
        // Find the perpendicular distance between the vehicle's velocity direction
        // and the parallel vector located at the target position
        float perpenDist = 0.5f * distToTargetPos * FastMath.cos(angleWithPerpen);
        
        
        // Make sure the turn radius is not smaller than the minimum allowed turn radius  
        if(FastMath.abs(perpenDist) < MIN_TURN_RADIUS) {
            System.out.println("Original turn radius: "+perpenDist);
            turnRadius = FastMath.sign(perpenDist) * MIN_TURN_RADIUS;
        } else {
            turnRadius = perpenDist;
        }
        
        return turnRadius;
    }
    
    public void follow(float turnRadius, Vector3f neighborEstVel, float tpf) {
        
        float followAngle = activeVelocity.normalize().angleBetween(neighborEstVel.normalize());

        if(followAngle < FastMath.PI) {
            this.turn("right", followAngle, tpf);
        } else {
            this.turn("left", followAngle, tpf);
        }
        
    }
    
    private void changeVelocityTo(Vector3f neighborEstVel, float tpf) {
//        estimateTurnRadius(float neighborDistEst)
//        follow(MIN_TURN_RADIUS, Vector3f neighborEstVel, float tpf);
    }
      
    public void initCommunication() {
        
        // Shared vars
        Quaternion aimDirection = new Quaternion();
        Vector3f rayDirection = new Vector3f();
        neighborList = new ArrayList<String>();
        Vector3f rayOrigin = new Vector3f();
        ColorRGBA lineColor;
        
        
        int i = 0;
        vehiclesInRange = false;

        //System.out.println("Comm. partners: " + commPartners);
          
        float angleWithActVel, angleAroundActVel;
            
        for(int r = 0; r < COMM_RESOLUTION; r++) {
            
            rayOrigin = spatial.getWorldTranslation().add(activeVelocity.normalize().mult(0.01f));
            rayDirection.set(activeVelocity.normalize());
            lineColor = COMM_RANGE_COLOR;
        
            if(COMM_DEBUG && commLines[i] != null){
                //System.out.println("In detatch: "+(Node)getSpatial().getParent());
                ((Node)getSpatial().getParent()).detachChild(commLines[i]);
            }
            
            angleWithActVel = commXAngles[r];
            angleAroundActVel = commYAngles[r];
                  
            Vector3f perpenVec = activeVelocity.normalize().cross(Vector3f.UNIT_Y);
            Quaternion rotateFromActVel = new Quaternion();
            rotateFromActVel.fromAngleAxis(angleWithActVel, perpenVec.normalize());

            Quaternion rotateAroundActVel = new Quaternion();
            rotateAroundActVel.fromAngleAxis(angleAroundActVel , activeVelocity.normalize());

            aimDirection = rotateAroundActVel.mult(rotateFromActVel);
            aimDirection.multLocal(rayDirection);
            
            //Ray ray = new Ray(spatial.getWorldTranslation().add(activeVelocity.normalize().mult(0.8f)), rayDirection);
            Ray ray = new Ray(rayOrigin, rayDirection);
            ray.setLimit(COMM_RANGE);
            
            CollisionResults colRes = new CollisionResults();
            for(Spatial s: commPartners){
                s.collideWith(ray, colRes);
            }
            
            if(colRes.size() > 0){

                // Get a list of neighbors to use in building the dynamic network
                for(CollisionResult res : colRes) {
                    String vehID = res.getGeometry().getParent().getParent().getName();
                    
                    //System.out.println("Collided with: "+vehID);
                    //System.out.println("Self name: "+spatial.getName());
                    //System.out.println("Substring: "+vehID.substring(0, 2));
                    
                    if( vehID.substring(0, 2).equals("V_") && !neighborList.contains(vehID) &&  !vehID.equals(spatial.getName())) {
//                        System.out.println("Taken: "+vehID);
                        neighborList.add(vehID);
                    } 
                }
                
                //System.out.println(vehicle.getModel().getName() + " --> " + neighborList);
                
                if(!neighborList.isEmpty()) {
                    /* Find distances to other vehicles (in an ideal case, distances
                     * should not be calculted in an exact manner like this, they 
                     * should be estimated based on sensing range; this will be done 
                     * later):
                     */
                    HashMap<String, Float> neighborDistances = new HashMap<String, Float>(neighborList.size());
                    float distance;

                    //System.out.println(neighborList.toString());

                    for(String neighbor : neighborList) {
                        distance = physicsVehicle.getPhysicsLocation().distance(sim.getUWVehiclesMap().get(neighbor).getPhysicsVehicle().getPhysicsLocation());
                        neighborDistances.put(neighbor, distance);
                    }

                    VEHICLE_NAME = spatial.getName(); //vehicle.getModel().getName();

                    buildDynamicNetwork(VEHICLE_NAME, neighborList, neighborDistances);

                    vehiclesInRange = true;
                    break;
                }
            } 
            
            if(COMM_DEBUG){
                Geometry line = makeDebugLine(ray, COMM_RANGE, lineColor);
                commLines[i++] = line;
                //System.out.println("In attach: "+(Node)getSpatial().getParent());
                ((Node)getSpatial().getParent()).attachChild(line);
            }
        }
        
        if( NET_DEBUG && !commLinkLines.isEmpty() && neighborList.isEmpty() ){
                
            Set<String> keysSet = (Set<String>) commLinkLines.keySet();
            Iterator keySetItr = keysSet.iterator();  
            String currentKey;

            while( keySetItr.hasNext() ) {
                currentKey = (String)keySetItr.next();
                if(((Node)getSpatial().getParent()).hasChild(commLinkLines.get(currentKey))) {
                    ((Node)getSpatial().getParent()).detachChild(commLinkLines.get(currentKey));
                }
            }
            commLinkLines.clear();
        }
        commInitiated = true;    
    }
    
    private void buildDynamicNetwork(String vehID, List<String> neighborList, HashMap<String, Float> neighborDistances) {
        
        try {
            // Build an UWSN out of the vehicle and its neighbors

            int nodeCount, linkCount, zoneCount;
            linkCount = 2 * neighborList.size();
            nodeCount = zoneCount = (linkCount/2) + 1;
            
            // Construct network
            UWSensorNetwork network = new UWSensorNetwork(nodeCount, zoneCount, linkCount);
            
            Random rand = new Random();
            networkID = Math.abs(rand.nextLong());

            // Link flow model
            FlowModel flowModel = new PolynomialFlowModel(network, 1,4);
            
            UWSensorNetwork.ACOUSTIC_LINK_PEAK_UTILIZATION = 0.18;
            
            // Specify nods' power levels
            PowerModel.PowerLevel lowPowerLevel = new PowerModel.PowerLevel(1, 120, 200);
            //PowerLevel highPowerLevel = new PowerLevel(10, 500, 700);

            Vector<PowerModel.PowerLevel> powerLevels = new Vector<PowerModel.PowerLevel>();

            powerLevels.add(lowPowerLevel);
            //powerLevels.add(highPowerLevel);

            Random rnd = new Random(13103);

            int[] Seeds = {rnd.nextInt(), rnd.nextInt(), rnd.nextInt()};

            /* 
             * Idea: generate random locations for neighbors within a square of size 
             * 400x400, start neighbor ID's from "1", find average x and y locations 
             * of neighbors, create an additional node to represent the center (current) 
             * node and give it ID = "numNeighbors + 1" and location (avgX, avgY), and 
             * finally, connect current node to each neighbor (star topology).
             * All neighbors are sources (for now) and the center node is the sink
             * (for now as well). PlanarRandomNG generator was used because topology can 
             * be assumed flat for the purposes of this simulation (this, of course
             * neglects interference effects, but will be considered later). The only
             * important thing for now is to set link costs based on actual distances
             * between the center vehicle and its neighbors
             */
            
            // Generate two underwater nodes
            PlanarRandomNG randNG = new PlanarRandomNG(
                    network,        // Network
                    powerLevels,    // Power levels
                    0.0,            // Deployment cost
                    0.0,            // Consumed energy rate during movement
                    2,              // Node type: 1 -> Is sink, 2 -> Regular node, 3 -> Processing node
                    0.01,           // Idle listening power
                    0.1,            // Receiving power
                    1e4,            // Initial energy
                    1,              // First node ID
                    0.0, 400.0,     // Base x, x extent
                    0.0, 400.0,     // Base y, y extent
                    (nodeCount - 1),      // Node count
                    0,              // Base z
                    1,              // gBase
                    0,              // gRange
                    Seeds);         // Seed array
            neighborNodes = randNG.generate();

            double avgX = 0, avgY = 0;

            Iterator neighborNameIter = neighborList.iterator();
            
            for(int node : neighborNodes){
                    network.setAverageChannelAccessDelay(node, 0.5);
                    network.setResidualEnergy(node, 1e4);
                    network.setDataGenerationRate(node, 1);
                    network.getPowerModel().setIdleListeningPower(node, 0.01);
                    network.getPowerModel().setTransportEnergyRate(node, 0.0);
                    network.setDeploymentCost(node, 0.0);
                    network.setName(node, (String)neighborNameIter.next());
                    
                    avgX += network.getX(node);
                    avgY += network.getY(node);
            }
            
            avgX /= neighborNodes.size();
            avgY /= neighborNodes.size();
            
            if(neighborNodes.size() == 1) {
                avgX += 100;
            }
            
            centerNodeID = network.addNode(
                    new LinkedList<Integer>(),  // In-links
                    new LinkedList<Integer>(),  // Out-links
                            avgX, avgY, 0,      // x, y, z           
                            1,                  // gBase
                            powerLevels,        // Power levels
                            0,                  // Deployment cost 
                            0,                  // Consumed energy rate during movement
                            1,                  // Node type: 1 -> Is sink
                            0.01,               // Idle listening power
                            0.1,                // Receiving power
                            1e4);               // Initial energy
            
            VEHICLE_ID = centerNodeID;
            
            network.setAverageChannelAccessDelay(centerNodeID, 0.5);
            network.setResidualEnergy(centerNodeID, 1e4);
            network.setDataGenerationRate(centerNodeID, 1);
            network.getPowerModel().setIdleListeningPower(centerNodeID, 0.01);
            network.getPowerModel().setTransportEnergyRate(centerNodeID, 0.0);
            network.setDeploymentCost(centerNodeID, 0.0);
            network.setName(centerNodeID, vehID);

            double distance;
            
            for(Integer nbr : neighborNodes) {
                //add links
                distance = neighborDistances.get(network.getName(nbr));
                
                //System.out.println("Distance from center node "+centerNodeID+" (" + network.getName(centerNodeID) + ") to node " + nbr + " (" + network.getName(nbr) + ") is: " + distance);
                //System.out.println("NetID: "+networkID+": "+ network.getName(nbr)+" ("+nbr+") --> " + network.getName(centerNodeID)+" ("+centerNodeID+")");
                //System.out.println("NetID: "+networkID+": "+ network.getName(centerNodeID)+" ("+centerNodeID+") --> " + network.getName(nbr)+" ("+nbr+")");
                // neighbor --> center node
                
                network.addLink(
                        nbr,                    // From
                        centerNodeID,           // To                        
                        100000,                 // Link Capacity
                        distance,               // Link length
                        distance/1500,          // Link propagation delay
                        1500,                   // Acoustic signal speed
                        1,                      // Link type (one type for now)
                        flowModel               // Link flow model
                        );
                
                // center node --> neighbor
                
                network.addLink(
                        centerNodeID,           // From
                        nbr,                    // To                        
                        100000,                 // Link Capacity
                        distance,               // Link length
                        distance/1500,          // Link propagation delay
                        1500,                   // Acoustic signal speed
                        1,                      // Link type (one type for now)
                        flowModel               // Link flow model
                        );
            }
            
            network.getCostModel().calculateCosts();

            //System.out.println("Link Costs: " + network.getLinkCost().toString());
            
            Demand demand  = new Demand(network);   // Data that needs to be sent
            Assignment assignment;    // Router's assignment of the given demand
            long runtime;             // The total runtime in milliseconds
            
            // Add demands from all neighbors to center node
            for(Integer nbr : neighborNodes) {
                demand.add(new ODPair(nbr,centerNodeID), 300000);
                demand.add(new ODPair(centerNodeID,nbr), 300000);
            }
            
            Router router;                       // A router to route traffic
            router = new ShortestPathRouter(network, network.getLinkCost(), null);
            
            runtime = startSimulationTimer();
	
            assignment = router.route(demand);
            DoublePropertyMap flow = assignment.getFlow();
            network.setFlow(flow);

            runtime = stopSimulationTimer(runtime);

            //printDefaultLinkProperties(network);
            //printDefaultNodeProperties(network);
            //printDemand(demand);

            double totalDelay = 0.0;
            for(int i = 1; i <= network.getLinkCount(); i++){
                    totalDelay += network.getTravelTime(i);
            }
//            System.out.print("\n\n");
//            System.out.print("Total Delay: ");
//            System.out.print(totalDelay);
//            System.out.print("\n\n");

//            printRuntime(runtime);

            sim.getCommMedium().addNetwork(network, networkID);
            
            // Visual representation
            Vector3f rayDirection = new Vector3f();
            Vector3f vehicleLoc = spatial.getWorldTranslation();
            Vector3f neighborLoc;
          
            for(String nbr : neighborList) {

                if( NET_DEBUG && !commLinkLines.isEmpty() ){

                    Set<String> keysSet = (Set<String>) commLinkLines.keySet();
                    Iterator keySetItr = keysSet.iterator();  
                    String currentKey;

                    while( keySetItr.hasNext() ) {
                        currentKey = (String)keySetItr.next();
                        if(((Node)getSpatial().getParent()).hasChild(commLinkLines.get(currentKey))) {
                            ((Node)getSpatial().getParent()).detachChild(commLinkLines.get(currentKey));
                        }
                    }
                    commLinkLines.clear();
                }                

                neighborLoc = sim.getUWVehiclesMap().get(nbr).getPhysicsVehicle().getPhysicsLocation();    
                rayDirection.set( neighborLoc.subtract(vehicleLoc).normalize() );
                
                Ray ray = new Ray(vehicleLoc, rayDirection);
                ray.setLimit(neighborDistances.get(nbr));

                if(NET_DEBUG){
                    ColorRGBA linkColor;
                    if(commState == CommState.send) {
                       linkColor = SEND_COMM_LINK_COLOR;
                    } else if(commState == CommState.receive) {
                        linkColor = RECEIVE_COMM_LINK_COLOR;
                    } else {
                        linkColor = IDLE_COMM_LINK_COLOR;
                    }
                    
                    Geometry line = makeDebugLine(ray, neighborLoc.subtract(vehicleLoc).length(), linkColor);
                    commLinkLines.put(vehID, line);
                    ((Node)getSpatial().getParent()).attachChild(line);
                }
            }
            
        } catch (InvalidNodeId ex) {
            Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
        } catch (InvalidLinkId ex) {
            Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    /**
     * Broadcast message to neighbors
     * @param message message to broadcast
     */
    private boolean broadcast(Message message) {

        if(!commInitiated) {
            initCommunication();
        }

        HashMap<String, Object> headers = new HashMap<String, Object>();
        headers.put("From", centerNodeID);
        headers.put("To", neighborNodes);
        headers.put("Type", MsgTransmitType.broadcast);
        headers.put("NetworkID", networkID);

        message.addHeaders(headers);

        commState = CommState.send;

        if( !neighborList.isEmpty() ) {     // extra cautiousness
            System.out.println("Broadcasting to neighbors ...");
            //System.out.println("Neighbors: "+neighborList.toString());
            sim.getCommMedium().conveyMessage(message);
            return true;
        } else {
            return false;
        }

//            for(Integer neigh : neighborNodes) {
//               System.out.println("Broadcasted Message: " + sim.getCommMedium().getMessage(centerNodeID, neigh).getData("text")); 
//            }
    }
    
    private List<Message> listenToNeighbors() {
     
        if(!commInitiated) {
           initCommunication(); 
        }
        
        commState = CommState.receive;
        
        ArrayList<Message> neighborMessages = new ArrayList<Message>();
        SimpleMessage msg;
        
        // Listen to neighbors
        if( !neighborList.isEmpty() ) {
            System.out.println("Listening to neighbors ...");
            for(String neigh : neighborList) {
                while((msg = (SimpleMessage)sim.getCommMedium().getMessage(neigh, VEHICLE_NAME)) != null) {
                    System.out.println(VEHICLE_NAME+" received a message from "+neigh+". Content: "+ msg.getData("SELF_BEST_POINTER"));
                    neighborMessages.add(msg);
    //                if(msg.getData("SELF_BEST_POINTER") != null) {
    //                    System.out.println(VEHICLE_NAME+" received a message from "+neigh+". Content: "+ msg.getData("SELF_BEST_POINTER").toString());
    //                }
                    sim.receivedMsgCount++;
                }
            } 
        }

        return neighborMessages;
    }
    
    /**
     * Send message to a specific neighbor
     * @param toID  neighbor id
     * @param messageContent content of the message
     */
    public void sendMessageTo(int toID, HashMap<String, Float> messageContent) {
        
    }
    
    /**
     * Poll to neighbors (get their messages)
     * @return neighbors and their messages
     */
    public HashMap<String, Message> pollNeighbors() {
        
        
        return null;
    }
    
    public String getVehicleName() {
        return spatial.getName();
    }
    
    public void printDefaultLinkProperties(UWSensorNetwork network){
        // Print out default network edge properties.
        System.out.print(network.toString("links", new String[]{"Cpcty", "Flow", "Length", "FFTTime", "TTime", "LinkCost"}));
        System.out.print("\n");
    }

    public void printDefaultNodeProperties(UWSensorNetwork network){
        // Print out default network node properties.
        System.out.print(network.toString("nodes", new String[]{"Coordinates", "NodeType", "IdleListeningPower", "ReceivingPower"}));
        System.out.print("\n");
    }

    public void printLinkProperties(UWSensorNetwork network, String[] propertiesToPrint){
        // Print out specified network edges properties.
        System.out.print(network.toString("links", propertiesToPrint));
        System.out.print("\n");
    }

    public void printNodeProperties(UWSensorNetwork network, String[] propertiesToPrint){
        // Print out specified network nodes properties.
        System.out.print(network.toString("nodes", propertiesToPrint));
        System.out.print("\n");
    }

    public void printDemand(Demand demand){
        System.out.print(demand);
        System.out.print("\n");
    }
    
    public long startSimulationTimer(){
        // Set start time
        return System.currentTimeMillis();
    }

    public long stopSimulationTimer(long runtime){
        return System.currentTimeMillis() - runtime;
    }

    public void printRuntime(long runtime){
        System.out.printf("Runtime =%d ms\n", runtime);
    }
        
    private Geometry makeDebugLine(Ray r, float length, ColorRGBA color){
        Line line = new Line(r.getOrigin(), r.getOrigin().add(r.getDirection().mult(length)));
        if(color.equals(SEND_COMM_LINK_COLOR) || 
                color.equals(RECEIVE_COMM_LINK_COLOR) || 
                color.equals(IDLE_COMM_LINK_COLOR)) {
            line.setLineWidth(COMM_LINK_WIDTH);
        }
        Geometry lineGeom = new Geometry("", line);
        Material material = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        material.setColor("Color", color);
        lineGeom.setMaterial(material);
        return lineGeom;
    }
    
    public void setAuxSearchTargets(List<Spatial> auxSearchTargets) {
        this.auxSearchTargets = auxSearchTargets;
    }

    public void setUltimateSearchTargets(List<Spatial> ultimateSearchTargets) {
        this.ultimateSearchTargets = ultimateSearchTargets;
    }

    public void setVehicleTargets(List<Spatial> vehicleTargets) {
        this.vehicleTargets = vehicleTargets;
    }
    
    public void setCommPartners(List<Spatial> vehicles) {
        this.commPartners = vehicles;
    }
    
    public State getState() {
        return state;
    }
    
    public void setState(State state) {
        this.state = state;
    }
    
    public RigidBodyControl getPhysicsVehicle() {
        return physicsVehicle;
    }
     
    private float randPos() {
        return (float)( Math.random() * (Math.random() < 0.5 ? -HALF_PLANE : HALF_PLANE) );
    }

    private float randVel() {
        return (float)( Math.random() * (Math.random() < 0.5 ? -vehicle.getMaxSpeed() : vehicle.getMaxSpeed()) );
    }

    private void setInitLocation(ContainerType containerType, Vector3f containerLocation) {
        
        switch(containerType) {
            case spread: 
                break;
            case barrel:
                break;
            case disc:  
                break;
            case box:
            default:
                float boxSideLength = (float) java.lang.Math.cbrt(sim.getVehiclesContainerVolume());
                boxSideLength += 0.2 * boxSideLength;

                do {
                    inContainerVehPos = generateInBoxVehPos(boxSideLength);
                    inContainerVehPos = containerLocation.add(inContainerVehPos);
                } while(sim.getInContainerVehPosList().contains(inContainerVehPos));
        }
        
        physicsVehicle.setPhysicsLocation(inContainerVehPos);

    }
    
    private Vector3f generateInBoxVehPos(float boxSideLen) {
        float xPos = (float)( Math.random() * (Math.random() < 0.5 ? -boxSideLen : boxSideLen) );
        float yPos = (float)( Math.random() * (Math.random() < 0.5 ? -boxSideLen : boxSideLen) );
        float zPos = (float)( Math.random() * (Math.random() < 0.5 ? -boxSideLen : boxSideLen) );
        
        return new Vector3f(xPos, yPos, zPos);
    }
    
    public Vector3f getInContainerVehPos() {
        return inContainerVehPos;
    }
    
    public void keepInBounds() {
        
        Vector3f boundedLoc = physicsVehicle.getPhysicsLocation();
    
        // If the vehicle reaches a boundary reverse its velocity to keep it inside
        if( physicsVehicle.getPhysicsLocation().x > MAX_PT.x ) {
            boundedLoc.setX(MAX_PT.x);
            vehicle.setVel(Vector3f.ZERO);
        }
        if( physicsVehicle.getPhysicsLocation().y > MAX_PT.y ) {
            boundedLoc.setY(MAX_PT.y);
            vehicle.setVel(Vector3f.ZERO);
        }
        if( physicsVehicle.getPhysicsLocation().z > MAX_PT.z ) {
            boundedLoc.setZ(MAX_PT.z);
            vehicle.setVel(Vector3f.ZERO);
        }

        if( physicsVehicle.getPhysicsLocation().x < MIN_PT.x ) {
            boundedLoc.setX(MIN_PT.x);
            vehicle.setVel(Vector3f.ZERO);
        }
        if( physicsVehicle.getPhysicsLocation().y < MIN_PT.y ) {
            boundedLoc.setY(MIN_PT.y);
            vehicle.setVel(Vector3f.ZERO);
        }
        if( physicsVehicle.getPhysicsLocation().z < MIN_PT.z ) {
            boundedLoc.setZ(MIN_PT.z);
            vehicle.setVel(Vector3f.ZERO);
        }

        physicsVehicle.setPhysicsLocation(boundedLoc);
    }
    
    protected void setVelocity(Vector3f velocity) {
        activeVelocity = velocity;
    }
    
    @Override
    protected Vector3f calcNewVel() {

            // Empty the velChanges ArrayList
            velChanges.clear();

            // Set the initial value of the new velocity to the current velocity
            Vector3f newVelocity = vehicle.getVel();

            // Combine all velocity changes with the previous velocity
            for (Vector3f velChange : velChanges) {
                newVelocity.add((Vector3f) velChange);
            }

            // Make sure the new velocity is within speed limits;
            // limit its value using a scale factor 
            newVelocity.mult( limitMaxSpeed() );

            return newVelocity;
    }  
    
    @Override
    protected float limitMaxSpeed() {

        if( vehicle.getVel().length() > vehicle.getMaxSpeed() ) {
                return (vehicle.getMaxSpeed() / vehicle.getVel().length());
        } 
        return 1.0f;
    }
    
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {}
    
    private class CommAPI {
        
        public CommAPI() {
            
        }
        
        public boolean performHandshake() {
            return true;
        }
        
        public boolean sendHello() {
            return true;
        }
        
        public boolean sendData() {
            return true;
        }
        
        public boolean endSession() {
            return true;
        }
        
    }

    // =========================================================================
    // Vehicle Control API - START
    // =========================================================================
    public Vector3f getCurrentVehicleVelocity() {
        return activeVelocity.clone();
    }
    
    public Vector3f getCurrentVehiclePosition() {  
        return estimatedSelfPos.clone();
    }
    
    public Vector3f getVehicleBestPosPointer() {
        return selfBestPointer.clone();
    }
    
    public Vector3f getVehicleBestPos() {
        return (selfBestPos == null ? null : selfBestPos.clone());
    }
    
    public Integer getVehicleBestSolution() {
        return (selfBestSol == null ? null : selfBestSol);
    }

    public Vector3f getNeighborhoodBestPosition() {
        
        // Reset
        bestNeighborSol = 0;
        
        ArrayList<Message> neighborMessages = (ArrayList)listenToNeighbors();
        
        if(neighborMessages.size() > 0) {
            System.out.println("Number of neighbor messages: "+neighborMessages.size());
        }

        Integer sol;
        Vector3f bestNeighborSolPointer = null;
        
        for(Message msg : neighborMessages) {
            //System.out.println("Processing neighbor message ...");
            sol = (Integer)msg.getData("SELF_BEST_SOLUTION");
            System.out.println("Neighbor solution: "+sol);
            System.out.println("Best Neighbor solution: "+bestNeighborSol);
            if( sol > bestNeighborSol ) {
                bestNeighborSol = sol;
                bestNeighborSolPointer = (Vector3f)msg.getData("SELF_BEST_POINTER");
            }
        }
        
        return bestNeighborSolPointer;
    }
    
    public Integer getNeighborhoodBestSol() {
        return bestNeighborSol;
    }
    
    public void updateVehicleVelocity(Vector3f newVelocity) {
        //System.out.println("Entered position update ...");
        
        searchUpdateVelocity = newVelocity;

        //System.out.println("Active velocity: "+activeVelocity);
        //System.out.println("New velocity: "+searchUpdateVelocity);
        //System.out.println("Distance between velocities: "+searchUpdateVelocity.distance(activeVelocity));
        
        //@TODO: you may need to remove this check
        if( searchUpdateVelocity.distance(activeVelocity) < TURN_TRIGGER_THRESHOLD) {
            turnNotNeeded = true;
        } else {
            
            //DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), searchUpdateVelocity, ColorRGBA.Magenta, 2f, sim);
        
            if(System.currentTimeMillis() - gracePeriodStartTime >= GRACE_PERIOD) {
                resetGracePeriodVars();
            }
            
            if(!compositeTurnInProgress && !inGracePeriod) {

                //System.out.println("Entered turning ...");
                
                // Re-orient vehicle using the current (constant) vehicle velocity
                float actVelMagnitude = activeVelocity.length();
                reorient(searchUpdateVelocity.normalize().mult(actVelMagnitude));

                // Block turning updates and hand control to the state machine. When
                // it finishes the turn, it will notify the search algorithm to provide
                // turn updates.
                compositeTurnInProgress = true;

            }
        
        }
    }
    
    public void updateVehiclePosition(Vector3f newPosition) {
        
    }
    
    public boolean broadcastSelfBestPos() {
        
        if( selfBestPointer != null && selfBestSol != null && selfBestPointer.length() != 0 && selfBestSol != 0 ) {
            
//            System.out.println("================================================");
//            System.out.println("Self Best Pointer to broadcast: "+selfBestPointer);
//            System.out.println("Self Best Solution to broadcast: "+selfBestSol);
//            System.out.println("================================================");
            
            Message msg = new SimpleMessage();
            //msg.addData("text", "Hi! Neighbor");
            msg.addData("SELF_BEST_POINTER", selfBestPointer);
            msg.addData("SELF_BEST_SOLUTION", selfBestSol);
            
            return (boolean)broadcast(msg);  
            
        } else {
            return false;
        }
        
    }
    
    public Vector3f getMostRecentNeighPosPointer() {
        return mostRecentNeighBestPos;
    }
    // =========================================================================
    // Vehicle Control API - END
    // =========================================================================
    
}
