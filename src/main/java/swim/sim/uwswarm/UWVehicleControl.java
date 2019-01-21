package swim.sim.uwswarm;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
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
import java.util.Arrays;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;
import java.util.Vector;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.vecmath.Point3f;
import swim.algorithm.common.TargetConfig;
import swim.algorithm.flocking.FlockingAlgorithm;
import swim.algorithm.flocking.ModifiedReynolds;
import swim.algorithm.integration.AllStageIntegration;
import swim.algorithm.integration.BrainlessAllStageInt;
import swim.algorithm.integration.IntegrationAlgorithm;
import swim.algorithm.integration.IntegrationAlgorithms;
import swim.algorithm.integration.TwoStageIntegration;
import swim.algorithm.learning.LearningAlgorithm;
import swim.algorithm.learning.RLearning;
import swim.algorithm.motion.NeighborsAverage;
import swim.algorithm.motion.TargetLingering;
import swim.algorithm.search.ConstrainedSpiralFlocking;
import swim.algorithm.search.DHCP;
import swim.algorithm.search.FlockedSRW;
import swim.algorithm.search.RPSO;
import swim.algorithm.search.SearchAlgorithm;
import swim.algorithm.search.SearchAlgorithms;
import swim.algorithm.search.BFM;
import swim.algorithm.search.RDPSO;
import swim.algorithm.search.SDHCP;
import swim.algorithm.search.SSDHCP;
import swim.algorithm.search.SimpleRandomWalk;
import swim.algorithm.search.SimpleSweeping;
import swim.algorithm.search.VirtualTetherSearch;
import swim.algorithm.selforg.InitSelfOrgAlgorithms;
import swim.algorithm.selforg.InitialSOAlgorithm;
import swim.algorithm.selforg.SelfOrgAlgorithm;
import swim.algorithm.surfacing.SimpleSurfacing;
import swim.algorithm.surfacing.SurfacingAlgorithm;
import swim.algorithm.surfacing.SurfacingAlgorithms;
import swim.algorithm.taskalloc.BeaconBasedTaskAlloc;
import swim.algorithm.taskalloc.ExpRespThresh;
import swim.algorithm.taskalloc.HybridTaskAlloc;
import swim.algorithm.taskalloc.BlindTaskAlloc;
import swim.algorithm.taskalloc.PolyRespThresh;
import swim.algorithm.taskalloc.TargetType;
import swim.algorithm.taskalloc.TaskAction;
import swim.algorithm.taskalloc.TaskAllocAlgorithm;
import swim.algorithm.taskalloc.TaskAllocAlgorithms;
import swim.algorithm.taskalloc.TaskDescriptor;
import swim.core.AgentControl;
import swim.core.CommChannel;
import swim.core.CommState;
import swim.core.ContainerType;
import swim.core.Message;
import swim.core.MissionStage;
import swim.core.MsgTransmitType;
import swim.core.SensorName;
import swim.core.SimpleMessage;
import swim.core.State;
import swim.core.StateChangeReason;
import swim.core.motion.ActiveMotion;
import swim.core.motion.CompositeTurnSequence;
import swim.core.motion.MotionGoal;
import swim.core.motion.OrientMode;
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
import swim.sim.GeneralTestsMode;
import swim.sim.InitSelfOrgMode;
import swim.sim.IntegrationMode;
import swim.sim.SearchMode;
import swim.sim.SimulationMode;
import swim.sim.Simulator;
import swim.sim.TaskAllocMode;
import swim.sim.uwswarm.intelli.brain.MiniBrain;
import swim.sim.uwswarm.intelli.brain.struct.artif.CompositeSequence;
import swim.sim.uwswarm.intelli.brain.struct.artif.Duration;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotionAlgorithm;
import swim.sim.uwswarm.intelli.brain.struct.artif.MotionType;
import swim.sim.uwswarm.intelli.brain.struct.artif.TriggeringEvent;
import swim.sim.uwswarm.intelli.resource.ElapsedTime;
import swim.util.DebugUtils;
import swim.util.Pair;

/**
 *
 * @author Sherif
 */
public class UWVehicleControl extends AgentControl implements PhysicsTickListener {
    
    // =========================================================================
    // Config
    // =========================================================================
    
    // In ticks (make it < MAX_SIMULATION_TICK_COUNT in Simulator)
    private int MISSION_TIME_CONSTRAINT; 
    
    private final boolean BRAIN_MODE_ENABLED = false;
    
    private final boolean DIFFERENTIATE_PROCESSING_TIMES = true;
    
    // Controller execution mode
    private boolean ORIG_REORIENTATION_ALG = false,
                    ENHANCED_REORIENTATION_ALG = false,
                    ADVANCED_REORIENTATION_ALG = true,
                    OPTIMIZED_REORIENTATION_ALG = false,
            
                    DEBUG_OVERRIDE_TARGET_FOUND = false,
            
                    LEARNING_ENABLED = false,       // was true
                    LEARNING_PHASE_ACTIVE = false,   // was true
                    POLICY_APPLICATION_ACTIVE = false;
    
    //--------------------------------------------
    //############################################
    //--------------------------------------------
    private boolean STATE_TRACKER_ENABLED = false;
    //--------------------------------------------
    //############################################
    //--------------------------------------------
    
    private static boolean USE_RPSO_SEARCH = true,
                    USE_CSF_SEARCH = false,
                    USE_VTS_SEARCH = false,
                    USE_RDPSO_SEARCH = false,
                    USE_BFM_SEARCH = false,
                    USE_BDFSO_SEARCH = false,
                    USE_DHCP_SEARCH = false,
                    USE_SDHCP_SEARCH = false,
                    USE_SSDHCP_SEARCH = false,
                    USE_DFHCP_SEARCH = false,
                    USE_SRW_SEARCH = false,
                    USE_SSW_SEARCH = false,
                    USE_FSRW_SEARCH = false,
            
                    USE_BUBBLE_CHAIN_RW = false,
                    USE_TETHERED_RW = true,
                    USE_RETRACTED_SEQ_RW = false,
            
                    USE_ERT_TASK_ALLOC = false,
                    USE_PRT_TASK_ALLOC = false,
                    USE_MBB_TASK_ALLOC = false,
                    USE_BB_TASK_ALLOC = false,
                    USE_HYB_TASK_ALLOC = false,
                    USE_BL_TASK_ALLOC = true,
            
                    USE_TWO_STAGE_INTEG = false,
                    USE_ALL_STAGE_INTEG = false,
                    USE_BRAINLESS_ALL_STAGE_INTEG = true;
    
    private static boolean SENS_DEBUG = false,
                    COMM_DEBUG = false,
                    NET_DEBUG = false,

                    //--------------------------------------------
                    //############################################
                    //--------------------------------------------
                    VEHICLE_PATH_DEBUG = true,
                    //--------------------------------------------
                    //############################################
                    //--------------------------------------------
            
                    LOC_EST_DEBUG = false,
                    VEL_EST_DEBUG = false,
                    DEBUG_SELF_POS_HIST = false,
                    DEBUG_SHOW_VEHICLE_NAME = false,

                    DEBUG_SELF_BEST_ANCHOR = false,
                    DEBUG_PREV_SELF_BEST_ANCHOR = false,

                    SEND_REC_DEBUG_ENABLED = false,
                    TURN_DEBUG_ENABLED = false,
                    PSO_DEBUG_ENABLED = false,
                    CSF_DEBUG_ENABLED = false,

                    DEBUG_TURN_DIR_ENABLED = false,

                    DEBUG_MISSION_TIME_ENABLED = false,

                    DEBUG_OPT_REORIENT_ALG = false,
                    DEBUG_REORIENTATION_ALGS = false,

                    DEBUG_TASK_ALLOC = false,
                    DEBUG_SEQ_RETRACTION = false,
                    DEBUG_EXPLOITATION = false,
                    DEBUG_TARGET_SWEEPING = false,
            
                    DEBUG_TETHERED_RW = false,

                    FRONT_SENS_ENABLED = true,
                    BACK_SENS_ENABLED = true,
                    RIGHT_SENS_ENABLED = true,
                    LEFT_SENS_ENABLED = true,
                    BOTTOM_SENS_ENABLED = true;

    private int SRW_BIG_JUMP_TICKS = 10,
                SRW_SMALL_JUMP_TICKS = 1;
   
    private static boolean ENABLE_VEH_VEH_COLL_DETECTION = false;
    private boolean DEBUG_VEH_VEH_COLLISION = false;
    
    private int MIN_ACCEPT_NUM_NEIGH = 3;
    
    private boolean LIMITED_ENERGY_VEHICLE = false,
                    DEBUG_ENERGY_LEVEL_ENABLED = false;
    
    private static Vector3f CONTAINER_LOCATION = new Vector3f(-30, 90, 0);    // Used in all tests!   
    //private static final Vector3f CONTAINER_LOCATION = new Vector3f(0, 90, 0);      // New for testing task allocation!
    //private static Vector3f CONTAINER_LOCATION = new Vector3f(0, 90, -80);      // New for testing self organization!
    
    
    //private final Vector3f CONTAINER_LOCATION = new Vector3f(45, 50, 30);
    //private final Vector3f CONTAINER_LOCATION = new Vector3f(62, 90, 65); 
    //private static final Vector3f CONTAINER_LOCATION = new Vector3f(0, 90, -50);
    
    // Vehicle physical constraints
    
    private static final float MIN_TURN_RADIUS = 1f;     // was 1
    private final float CENTRAL_ANGLE = FastMath.TWO_PI / 30,
                        MAX_TURN_ANGLE = 0.5f * CENTRAL_ANGLE,
                        MTA_CHORD_LEN = 2*MIN_TURN_RADIUS*FastMath.sin(MAX_TURN_ANGLE),
                        MIN_VEH_SPEED = 5,
                        MAX_VEH_SPEED = 10,
                        AT_TASK_VEH_SPEED = 5f;
    
    private final float VEHICLE_SPEED = MAX_VEH_SPEED;
    
    
    private static int HALF_PLANE;   
    // Minimum permissible height
    private static Point3f MIN_PT;
    private static Point3f MAX_PT;
    private final float TURN_RADIUS = 0.1f;        // 0.01f
    private final float TURN_RESOLUTION = 25;
    
    private final ColorRGBA COMM_RANGE_COLOR = ColorRGBA.Green,
            FRONT_SENS_RANGE_COLOR = ColorRGBA.Magenta,
            BACK_SENS_RANGE_COLOR = ColorRGBA.Blue,
            RIGHT_SENS_RANGE_COLOR = ColorRGBA.Orange,
            LEFT_SENS_RANGE_COLOR = ColorRGBA.Cyan,
            BOTTOM_SENS_RANGE_COLOR = ColorRGBA.Red,
            TURN_DIRECTION_ARROW_COLOR = ColorRGBA.Brown,
            TETHER_COLOR = ColorRGBA.Red; 
    
    private final ColorRGBA SEND_COMM_LINK_COLOR = ColorRGBA.Blue,
                            RECEIVE_COMM_LINK_COLOR = ColorRGBA.Red,
                            IDLE_COMM_LINK_COLOR = ColorRGBA.Yellow;
    
    private ColorRGBA DEBUD_PATH_COLOR = ColorRGBA.Black;
    private float DEBUD_PATH_WIDTH = 1f;
    
    private final float COMM_LINK_WIDTH = 2f;
    
    private int DEBUG_PATH_REPLOT = 5;  // was 5 for all runs
    
    
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
    
    private final float MOTION_START_DEPTH = 10;
    
    private final int POSITION_HISTORY_lEN = 100;//10;//20;    // multiple of POS_RECORDS_TO_AVERAGE
    
    // Number of times an attempt is made to pull the position history before
    // a decision to resort to previous best position is made
    private final int MAX_POS_HIST_PULL_TRIALS = 2* POSITION_HISTORY_lEN;
    
    private final float POS_RECORDS_TO_AVERAGE = 50;//5;
    private final int POS_HIST_BUFF_POLL_TIMES = (int)(POSITION_HISTORY_lEN / POS_RECORDS_TO_AVERAGE),
                      NUM_AVG_POSITIONS = POS_HIST_BUFF_POLL_TIMES;
    
    private final float SELF_POS_EST_ERROR = 0.05f,
                        NEIGH_MAX_POS_EST_ERROR = 0.05f, //0.2f,
                        NEIGH_MAX_VEL_EST_ERROR = 0.05f; //0.2f;
            
    private final float TURN_TRIGGER_THRESHOLD = 0.000005f; // was 2
    
    private final int ACCELERATION_LENGTH = 50;
    private final float MAX_ACCELERATION_PITCH = 0.9f,
                        DEFAULT_DECELERATION_PITCH = 0.08f;
    
    private final int NUM_SENSORS = 5;
    private final float DIRECTION_STATE_COUNT = 8;
    
    private final float SENSING_RANGE = 8f;
    private final float COMM_RANGE = 16f;
    
    private final boolean APPLY_UPWARDS_VELOCITY = false;
    private final float DEFAULT_UPWARDS_VELOCITY = 0.025f;
    
    private float UPWARD_ACCELERATION = 0.00001f; // was 0.001 until 7/17/16 (changed after the turn speed was increased for nonr MTR
    
    private final long INTER_SEARCH_TIME = 7000;
    private final long GRACE_PERIOD = 0;
    
    private final long EXPLOITATION_TIME = 30000,
                       OTHER_TYPE_EXPLOITATION_TIME = 50000;
    
    private final float MAX_RAND_WALK_TURN_ANGLE = FastMath.HALF_PI,
                        RARE_TURN_ANGLE = 3 * FastMath.QUARTER_PI,
                        MAX_RAND_TRAVEL_DIST = 1;
    
    private final float MAX_MAG_NORTH_EST_ERROR = 0.1f;
    
    private final float INIT_REOR_ALLOWANCE = 6000;
    
    // Ocean water, AUV, motion, and energy consumption parameters
    private final float WATER_DENSITY = 1035, // k/m^3 (source: http://web.mit.edu/12.000/www/m2005/a2/finalwebsite/equipment/robotics/superman.shtml)
                        DRAG_COEFFICIENT = 0.15f, // same source
                        PROPULSION_EFFICIENCY = 0.35f, // (Source: https://www.mbari.org/wp-content/uploads/2016/01/Willcox_Bellingham_Zhang_Baggeroer_IEEEJOE_2001.pdf)
                        HOTEL_LOAD = 40.0f, // Watts (power consumed by electronics, sensor systems, etc) (first source above)
                        INITIAL_BATTERY_ENERGY = 1.8e7f, //J (for a 5kWh battery) (first source above)
                        FAILURE_ENERGY = 50.0f,
                        WETTED_VEHICLE_AREA = 1.42f; //m^2 (Source: http://ac.els-cdn.com/S002980181500668X/1-s2.0-S002980181500668X-main.pdf?_tid=c47a3ed6-366a-11e6-9011-00000aab0f02&acdnat=1466374467_a59aef0557c321d5ddf6bc9499517d54) 
    
    private final float VEHICLE_FAILURE_SPEED_SCALE = 0.1f;
    
    private int VEH_VEH_COLLISION_TEST_INTERVAL = 50; // Ticks
    
    private int PRESET_NUM_TARG_PROCESSINGS;
    
    // Tethered-sequence RW constants
    private int MAX_SEQUENCES_WITHOUT_TARG_THRESH = 10;
    
    // =========================================================================
    // Instance variables
    // =========================================================================
    
    // Orientation modes
    private OrientMode prevOrientMode;
    
    // Simulation Modes
    private HashMap<SimulationMode, Boolean> simulationModes =
            new HashMap<SimulationMode, Boolean>(5);
    
    // Target sweeping
    private boolean targetSweepingModeActive = false,
                    sequenceBreakSolicited = false,
                    straightTravelRequested = false;
    private ArrayList<String> sweepedTargets;
    private ArrayList<String> currSweepTargs;
    private int sweepNumber = 0;
    private HashMap<Integer, ArrayList<String>> perSweepTargs;
    
    // Mission stages
    private MissionStage activeMissionStage;
    private Enum activeMissionLvlAction;
    private ActiveMotion activeMotion;
    private boolean activeMissionActInitialized = false;
    HashMap<MissionStage, Boolean> missionStageComplStatuses;
    
    // Motion state saving (sequences)
    private Deque<CompositeTurnSequence> savedSequenceStates;
    private boolean motionInterrupted = false;
    private MotionType interruptingMotion;
    private MotionAlgorithm previousMotionAlgorithm;
    
    // Brain
    private MiniBrain brain;
    private long activeMotionAlgKey, completedMotionAlgKey;
    private MotionType activeMotionType = MotionType.ALGORITHM;
    
    // Timers
    private int speedTimer;
    private boolean speedTimerEnabled = false,
                    speedTimerTimedOut = false;
    private int elapsedTimeOfActiveMotion = 0,
                presetDurOfActiveMotion = -1;
    
    // Tethered random walk variables
    private Vector3f seqStartPos, seqEndPos;
    private boolean startPosStored = false;
    private float countSeqExecsWithoutTarg = 0;
    
    // Tasks
    private boolean largNumTargsDetected = false,
                    mediumNumTargsDetected = false,
                    smallNumTargsDetected = false,
                    uniformMixTargTypesDetected = false,
                    dominantType_I_Targs = false,
                    dominantType_II_Targs = false,
                    dominantType_III_Targs = false,
                    singleTargetTask = false,
                    closestIsPreferred = false;
    
    // State
    private int stateSavingTime;
    private boolean fsmStateChangeRequested = false,
                    stateChangeStarted = false,
                    compTurnSeqMotionEnded = false;
    
    private StateChangeReason fsmStateChangeReason;
    
    // Motion algorithm and sequence queueing
    private ArrayBlockingQueue<Pair<MotionAlgorithm, Duration>> motionAlgQueue;
    private ArrayBlockingQueue<CompositeTurnSequence> compTurnSeqQueue;
    private MotionAlgorithm switchToMotionAlgorithm;
    private swim.algorithm.motion.MotionAlgorithm motionAlgorithm;
    private CompositeSequence motionSequence;
    private CompositeTurnSequence switchToMotionSequence;
    
    // AUV speed
    private float formerVehicleSpeed;
    
    // Other Controls used by UWVehicleControl
    private RigidBodyControl physicsVehicle;
    
    // Resources
    private float residualEnergy = INITIAL_BATTERY_ENERGY;
    
    // Interval start times 
    private int timeOfLastNeighEncounter,
                timeNeighDropBelowThres,
                sequenceStartTime,
                timeOfTrigger,
                chasingStartTime,
                searchStartTime,
                selfOrgStartTime,
                missionStartTime,
                timeOfFirstTargEncounter;
    
    private boolean seqStartTimeCaptured = false;
    
    // Time tracking variables
    private boolean dropBelowAcceptNeighCaptured = false,
                    dropBelowAcceptNeighEnded = false,
                    AcceptNumNeighReached = false;
    
    private float elapsedVehVehCollTestTime = 0.0f;
    
    // Collision tracking
    private int numCollisionsInPeriod = 0;
    
    // Target related
    private boolean targetEverEncountered = false,
                    firstTargetEncounter = true;
    
    // FSM
    private State state, prevFSMState = State.Drop;
    private boolean forward;
    private Spatial target;
    private float distToTarget;
    private boolean firstFSMUpdate = true;
    
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
    
    private int northParticlesCount,
                southParticlesCount,
                eastParticlesCount,
                westParticlesCount;
    private ArrayList<String> northPartNames,
                              southPartNames,
                              eastPartNames,
                              westPartNames;
    
    private Integer iterVehCount = 0;
    private int northVehiclesCount,
                southVehiclesCount,
                eastVehiclesCount,
                westVehiclesCount,
                bottomVehiclesCount;
    
    private ArrayList<String> northVehNames,
                              southVehNames,
                              eastVehNames,
                              westVehNames,
                              bottomVehNames,
            
                              staticVehNames,
                              movingVehNames;
    
    private Integer iterTargCount = 0;
    private int northTargetsCount,
                southTargetsCount,
                eastTargetsCount,
                westTargetsCount,
                bottomTargetsCount;
    private ArrayList<String> northTargNames,
                              southTargNames,
                              eastTargNames,
                              westTargNames,
                              bottomTargNames;
    
    // Searching
    private Spatial particle;
    private int posHistPullTrials = 0;
    private boolean initPosStored = false;
    
    //Task allocation
    private Spatial targ;

    
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
    private ArrayList<String> neighborList,
                              ignoreList;
    private Long networkID;
    private CommState commState; 
    
    Vector<Integer> neighborNodes;
    Integer centerNodeID;
    private int sentMsgsCount = 0, receivedMsgsCount = 0;
    
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
    
    float distanceTraveled = 0f;
    float overallMissionDistTraveled = 0;
    
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
                    turnCompleted = false,
                    compositeTurnJustEnded = false;
    
    private float activeTurnRadius, activeChordLen;
    
    private float vehSpeedSnapshot;
    
    private float turnAngle;
    private String turnDirection;
    
    private CompositeTurnSequence compositeTurnSeq;
    
    private float solicitedTurnRadius;
    private String solicitedTurnDirection;
    
    // Acceleration
    private boolean accelerateUpwards = false,
                    decelerate = false;
    private int accelStep = 0,
                decelStep = 0,
                decelerationLength = 0;
    private float activeAcceleration,
                solicitedUpwardsAccel;

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
    private Vector3f estimatedInitPos, prevSelfPosition, estimatedSelfPos;
    private float distFromLastPosRecord = 0;
    private long lastPosRecordTime;
    private Vector3f prevActVelocity;
    
    // Optimization
    private Vector3f selfBestPointer, prevSelfBestPointer;
    private Vector3f selfBestPos, prevSelfBestPos;
    private Integer selfBestSol, prevSelfBestSol;
    
//    private HashMap<String, Integer> posToPartCountMap;
    
    // Task allocation
    private boolean exploitTarget = false,
                    moveOnToNextTarget = true,
                    exploitingTarget = false;
    
    private long timeExploitationStarted;
    
    ArrayList<String> processedTargetsList;
    String[] targsToConsiderArr;
    
    // =========================================================================
    // Search algorithm
    // =========================================================================
    
    SearchAlgorithm searchAlgorithm;
    private float searchTurnRadius, searchTurnAngle;
    private String searchTurnDir;
    private Vector3f searchUpdatePos, searchUpdateVelocity;
    private boolean turnNotNeeded = false;
    
    private long timeSinceLastSearchStep = 0;
    private int elapsedGracePeriodTicks = 0;
    
    private boolean inGracePeriod = false;
    
    // =========================================================================
    // Task allocation algorithm
    // =========================================================================
    TaskAllocAlgorithm taskAllocAlgorithm;
    
    private TargetType preferredTargetType = TargetType.SHIP;
    
    // =========================================================================
    // Flocking algorithm
    // =========================================================================
    FlockingAlgorithm flockAlgorithm;
    
    // =========================================================================
    // Integration algorithm
    // =========================================================================
    IntegrationAlgorithm taskIntegAlgorithm;
    
    // =========================================================================
    // SO algorithm
    // =========================================================================
    SelfOrgAlgorithm selfOrgAlgorithm;
    private float reorientPathLength;
    private boolean enhanReorAlgTookEffect = false;
    
    // =========================================================================
    // Source-seeking/surfacing algorithm
    // =========================================================================
    private SurfacingAlgorithm surfacingAlgorithm;
    
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

    // ==================================================================
    // Reynold's Cohesion, Separation, and Alignment variables
    // ==================================================================
    private Vector3f avgNeighborPos;

    // ==================================================================
    // Clocks and timers - START
    // ==================================================================
    private float vehiclePhysClock = 0,
                   currPhysTime, prevPhysTime = 0;
    
    private int physTickCount = 0;
    
    //Timer timer1 = new  NanoTimer();
    //private float interFrameCounter = 0;
    
    // ==================================================================
    // Clocks and timers - END
    // ==================================================================
    
    // Testing Physics
    private Vector3f currPhysLoc, prevPhysLoc;
    
    // ==================================================================
    // Learning - START
    // ==================================================================
    private int rewardBucket;
    private boolean rewardAccumStarted = false;
    private Vector3f[] directionStates;
    private LearningAlgorithm learningAlgorithm;
    private int currentStateIndex, nextStateIndex, currentActionIndex;
    // ==================================================================
    // Learning - END
    // ==================================================================
    
    private Vector3f prevPhysLoc_;
    private long prevTime_, timeDiff_;
    private boolean firstUpdate = true;
    private float speed_, dist_;
    
    // Common variables (used by multiple algorithms)
    private boolean inPlaceTurnActivated = false,
                    inPlaceTurnStarted = false;
    
    // Random moves control
    private Deque<CompositeTurnSequence> seqRetractionBuffer;
    private OrientMode savedOrientMode;
    private boolean orientModeSaved = false;
    private boolean limitBubbleChainJumps = false;
    
    
    // Target related
    private String closestTargetName = "";
    private ArrayList<String> temporarilySkippedTargets;
    private TargetType typeOfTargToProcess;
    
    
    // =========================================================================
    // =========================================================================
    // =========================================================================
    
    
    public UWVehicleControl(UWVehicle vehicle, BulletAppState bulletAppState, Simulator sim) {
        
        // =====================================================================
        // Initialization
        // =====================================================================
        
        super();
        
        simulationModes.put(SimulationMode.SEARCH_MODE,         false);
        simulationModes.put(SimulationMode.TASK_ALLOC_MODE,     false);
        simulationModes.put(SimulationMode.INIT_SELF_ORG_MODE,  false);
        simulationModes.put(SimulationMode.INTEGRATION_MODE,    false);
        simulationModes.put(SimulationMode.GENERAL_TESTS_MODE,  false);
        
        this.vehicle = vehicle;
        this.sim = sim;
        
        activeTurnRadius = MIN_TURN_RADIUS;
        activeChordLen = MTA_CHORD_LEN;
        
        MISSION_TIME_CONSTRAINT = (int)FastMath.ceil(0.95f * sim.getMaxSimTickCount());
        
        // Set goal from 3 to 6 target processings
        do{
            PRESET_NUM_TARG_PROCESSINGS = FastMath.rand.nextInt(6);
        } while( PRESET_NUM_TARG_PROCESSINGS < 3 );

        missionStageComplStatuses = new HashMap<MissionStage, Boolean>(4);
        for( MissionStage stage : MissionStage.values() ) {
            missionStageComplStatuses.put(stage, false);
        }
        
        // =====================================================================
        // BRAIN
        // =====================================================================
        
        if( BRAIN_MODE_ENABLED ) {
            brain = new MiniBrain(this);
            
            savedSequenceStates = new LinkedList<CompositeTurnSequence>();
            motionAlgQueue = new ArrayBlockingQueue<Pair<MotionAlgorithm, Duration>>(5);
            compTurnSeqQueue = new ArrayBlockingQueue<CompositeTurnSequence>(5);
            
            activeMissionStage = brain.getHigherMentalFunc().getStartMissionStage();
        }
        
        // =====================================================================
        // Simulation Mode
        // =====================================================================

        detectActiveAppState();
        
        // =====================================================================
        // Initialization - cont'd
        // =====================================================================
        
        assetManager = sim.getAssetManager();
        physicsSpace = bulletAppState.getPhysicsSpace();
        terrainSideLen = (int)Math.floor((physicsSpace.getWorldMax().x - physicsSpace.getWorldMin().x));

        
        if( enabled(SimulationMode.INTEGRATION_MODE) || enabled(SimulationMode.TASK_ALLOC_MODE) ) {
            
            // Used for sequence retraction when the AUV has diverged from targets
            // for 3 composite turn sequence executions. 
            seqRetractionBuffer = new LinkedList<CompositeTurnSequence>();
            
            temporarilySkippedTargets = new ArrayList<String>();
            
        }
        
        if( enabled(SimulationMode.INIT_SELF_ORG_MODE) ) {
            CONTAINER_LOCATION = new Vector3f(0, 90, -80);
        } else if ( enabled(SimulationMode.TASK_ALLOC_MODE) ) {
            CONTAINER_LOCATION = new Vector3f(0, 90, 0);
        } else if( enabled(SimulationMode.INTEGRATION_MODE) ) {
            CONTAINER_LOCATION = new Vector3f(0, 90, 0);
        }
         
        
        // =====================================================================
        // Learning
        // =====================================================================
        
        if( (
                enabled(SimulationMode.SEARCH_MODE) || 
               (enabled(SimulationMode.INTEGRATION_MODE) && USE_BRAINLESS_ALL_STAGE_INTEG)
            ) 
            && USE_RPSO_SEARCH && LEARNING_ENABLED) {
            
            directionStates = new Vector3f[(int)DIRECTION_STATE_COUNT];
            float currAngle = 0, increment = FastMath.TWO_PI/DIRECTION_STATE_COUNT;
            for(int i = 0; i < DIRECTION_STATE_COUNT; i++) {            
               directionStates[i] = (new Vector3f(FastMath.cos(currAngle),0,FastMath.sin(currAngle))).normalize(); 
               currAngle += increment;
            }
        }
        
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
        
        // Prevents the vehicle's angular momentum from being affected by external forces (i.e. tip over)
        physicsVehicle.setAngularFactor(0);
        
        //setInitLocation(ContainerType.box, new Vector3f(-50, 50, 30));
        setInitLocation(ContainerType.box, CONTAINER_LOCATION);

        // Original
        // Set the initial velocity of the vehicle
        //initialVelocity = new Vector3f(randVel(), 0, randVel());
        
        // New
        float upwardsVel = (APPLY_UPWARDS_VELOCITY ? DEFAULT_UPWARDS_VELOCITY : 0);
        Vector3f tempVelVec = new Vector3f(0, upwardsVel, 1);
        Quaternion velRotQuat = new Quaternion();
        velRotQuat.fromAngleAxis(FastMath.rand.nextFloat() * FastMath.TWO_PI, Vector3f.UNIT_Y);
        Vector3f velVec = velRotQuat.mult(tempVelVec);
        velVec = velVec.mult(VEHICLE_SPEED);
        initialVelocity = velVec;
        // New

        this.vehicle.setVel(initialVelocity);
        activeVelocity = initialVelocity;
        
        limitVehicleSpeed();
        //System.out.println("Vel:" + activeVelocity);
        
        // =====================================================================
        // Self position tracking
        // =====================================================================

//        initialSubmergePos = physicsVehicle.getPhysicsLocation().clone().setY(MOTION_START_DEPTH);
        positionHistory = new ArrayBlockingQueue<Vector3f>(POSITION_HISTORY_lEN);
        posMarkerHist = new ArrayBlockingQueue<String>(POSITION_HISTORY_lEN);
//        posToPartCountMap = new HashMap<String, Integer>(POSITION_HISTORY_lEN);
//        positionHistory.offer(initialSubmergePos);
//        prevSelfPosition = initialSubmergePos;
//        prevActVelocity = activeVelocity;
//        lastPosRecordTime = System.currentTimeMillis();
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
        compositeTurnSeq = new CompositeTurnSequence();
        
//        turnAngle = FastMath.HALF_PI - ((FastMath.PI - (FastMath.HALF_PI / TURN_RESOLUTION))/2);
//        //System.out.println("Turn Angle: "+turnAngle);
//        //System.out.println("Turn Angle (Degrees): "+ turnAngle * FastMath.RAD_TO_DEG );
//        
//        chordLen = 2 * TURN_RADIUS * ((FastMath.PI - (FastMath.HALF_PI / TURN_RESOLUTION))/2);
        //System.out.println("Chord Length: "+chordLen);
        
        // =====================================================================

        //bulletAppState.getPhysicsSpace().addTickListener(this);  
        
        //Quaternion rotateToSensLimits =new Quaternion();
        //rotateToSensLimits.fromAngleAxis(sensFieldAngle/2, activeVelocity.normalize());
        
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
                
                //System.out.println(sensorXAngles[i][j] * FastMath.RAD_TO_DEG);
            } 
        }
        
        // =====================================================================
        // Communication range initialization
        // =====================================================================
        
        float angleWithActVel2, angleAroundActVel2;
        for(int j = 0; j < COMM_RESOLUTION; j++) {

//            if( commFieldAngle == FastMath.TWO_PI ) {
//                angleWithActVel2 = FastMath.acos(2 * (float)Math.random() - 1);
//            } else {
                angleWithActVel2  = FastMath.acos((float)( (Math.random() * (1 - FastMath.cos(commFieldAngle/2))) + FastMath.cos(commFieldAngle/2) ));                   
                angleWithActVel2 = (Math.random() > 0.5 ? angleWithActVel2 : -angleWithActVel2);
//            }
            //System.out.println("The angle is: "+ angleWithActVel2 * FastMath.RAD_TO_DEG);
            
            angleAroundActVel2 = 2 * FastMath.PI * (float)Math.random();
//            if( commFieldAngle != FastMath.TWO_PI ) {
                angleAroundActVel2 = (Math.random() > 0.5 ? angleAroundActVel2 : -angleAroundActVel2);
//            }
            //System.out.println("The OTHER angle is: "+ angleAroundActVel2 * FastMath.RAD_TO_DEG);

            commXAngles[j] = angleWithActVel2;
            commYAngles[j] = angleAroundActVel2;

            //System.out.println(sensorXAngles[i][j] * FastMath.RAD_TO_DEG);
        } 
        
        // Here
        //commState = CommState.send;
        
        // Networking Initialization
        ignoreList = new ArrayList<String>();
        
        // Testing Physics
        physicsSpace.addTickListener(this);

    }
    
    private void detectActiveAppState() {
        
        if(sim.getStateManager().getState(TaskAllocMode.class) != null &&
                sim.getStateManager().getState(TaskAllocMode.class).isEnabled()) {
            
            if( USE_ERT_TASK_ALLOC ) {
                taskAllocAlgorithm = new ExpRespThresh(this, sim);
            } else if( USE_PRT_TASK_ALLOC ) {
                taskAllocAlgorithm = new PolyRespThresh(this, sim);
            } else if( USE_MBB_TASK_ALLOC ) {
                // Implement ...
            } else if( USE_BB_TASK_ALLOC ) {
                taskAllocAlgorithm = new BeaconBasedTaskAlloc(this, sim);
            } else if( USE_HYB_TASK_ALLOC ) {
                taskAllocAlgorithm = new HybridTaskAlloc(this, sim);
            } else if( USE_BL_TASK_ALLOC ) {
                taskAllocAlgorithm = new BlindTaskAlloc(this, sim);
            }

            processedTargetsList = new ArrayList<String>();
            DEBUG_OVERRIDE_TARGET_FOUND = true;
            //UPWARD_ACCELERATION = 0.001f;
            disableOtherModes(SimulationMode.TASK_ALLOC_MODE);
        
        } else if(sim.getStateManager().getState(SearchMode.class) != null &&
                sim.getStateManager().getState(SearchMode.class).isEnabled()) {
            
            if( USE_RPSO_SEARCH ) {
                searchAlgorithm = new RPSO(this, sim); 
            } else if(USE_CSF_SEARCH) {
                searchAlgorithm = new ConstrainedSpiralFlocking(this, sim);
            } else if(USE_VTS_SEARCH) {
                searchAlgorithm = new VirtualTetherSearch(this, sim);
            } else if(USE_DHCP_SEARCH) {
                searchAlgorithm = new DHCP(this, sim);
            } else if(USE_SDHCP_SEARCH) {
                searchAlgorithm = new SDHCP(this, sim);
            } else if(USE_SSDHCP_SEARCH) {
                searchAlgorithm = new SSDHCP(this, sim);
            } else if(USE_SRW_SEARCH) {
                searchAlgorithm = new SimpleRandomWalk(this, sim);
            } else if(USE_SSW_SEARCH) {
                searchAlgorithm = new SimpleSweeping(this, sim);
            } else if(USE_FSRW_SEARCH) {
                searchAlgorithm = new FlockedSRW(this, sim);
            }
            
            if( USE_RPSO_SEARCH && LEARNING_ENABLED ) {
                learningAlgorithm = new RLearning(this, sim);
            }
            
            processedTargetsList = new ArrayList<String>();
            
            disableOtherModes(SimulationMode.SEARCH_MODE);
            
        } else if(sim.getStateManager().getState(InitSelfOrgMode.class) != null &&
                sim.getStateManager().getState(InitSelfOrgMode.class).isEnabled()) {
            
            selfOrgAlgorithm = new InitialSOAlgorithm(this, sim);
            flockAlgorithm = new ModifiedReynolds(this, sim);
            
            disableOtherModes(SimulationMode.INIT_SELF_ORG_MODE);
            
        } else if(sim.getStateManager().getState(GeneralTestsMode.class) != null &&
                sim.getStateManager().getState(GeneralTestsMode.class).isEnabled()) {
            
            //flockAlgorithm = new ModifiedReynolds(this, sim);
            disableOtherModes(SimulationMode.GENERAL_TESTS_MODE);
            
        } else if(sim.getStateManager().getState(IntegrationMode.class) != null &&
                sim.getStateManager().getState(IntegrationMode.class).isEnabled()) {
            
            if(USE_BRAINLESS_ALL_STAGE_INTEG) {
                taskIntegAlgorithm = new BrainlessAllStageInt(this, sim);
            } else if(USE_TWO_STAGE_INTEG) {
                taskIntegAlgorithm = new TwoStageIntegration(this, sim);
            } else if(USE_ALL_STAGE_INTEG) {
                taskIntegAlgorithm = new AllStageIntegration(this, sim); 
            }
            
            processedTargetsList = new ArrayList<String>();
            
            disableOtherModes(SimulationMode.INTEGRATION_MODE);

        }
        
        // Orientation
        prevOrientMode = getActiveOrientMode();
        
    }
    
    @Override
    protected void controlUpdate(float tpf) {
 
        elapsedVehVehCollTestTime++;
        
        if( speedTimerEnabled ) {
            if( speedTimer > 0 ) {
                speedTimer--;
            } else {
                updateVehicleSpeed(formerVehicleSpeed); // restore old speed
                disableSpeedTimer();
            }  
        }
        
        if( BRAIN_MODE_ENABLED ) {
            
            // =================================================================
            // IMPORTANT: add a mechanism to determine which among the below to 
            // be executed or given higher priority (if the two coexist).
            // =================================================================
            
            if( !motionAlgQueue.isEmpty() ) {
                try {
                    interruptingMotion = MotionType.ALGORITHM;
                    executeMotionAlgorithm(motionAlgQueue.take());
                } catch (InterruptedException ex) {
                    Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
                }
            } 
            
            if( !compTurnSeqQueue.isEmpty() ) {
                try {
                    interruptingMotion = MotionType.SEQUENCE;
                    executeCompositeTurnSeq(compTurnSeqQueue.take());
                } catch (InterruptedException ex) {
                    Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
                }
            }
            
            // FSM state change was requested
            if( fsmStateChangeRequested ) {
                
                if( fsmStateChangeReason.equals(StateChangeReason.RUN_MOTION_ALG) ) {
                    
                    activeMotionType = MotionType.ALGORITHM;
                    
                    switch( switchToMotionAlgorithm ) {
                        case LOG_SPIRAL:
                        case RANDOM_WALK_BIG_JUMPS:
                        case RANDOM_WALK_SMALL_JUMPS:
                            searchAlgorithm.start();
                            presetDurOfActiveMotion = searchAlgorithm.getPresetDuration();
                            break;
                        case FLOCKING:
                            flockAlgorithm.start();
                            presetDurOfActiveMotion = flockAlgorithm.getPresetDuration();
                            break;
                        case SAME_AS_BEFORE:
                        case PREDEFINED_MISSION_STAGE_ALG:
                            startActiveMissionAction();
                            // Add presetDurOf ... like above?!
                            break;
                        default:
                            motionAlgorithm.start();
                            presetDurOfActiveMotion = motionAlgorithm.getPresetDuration();
                    }   
                    //state = State.Update;
                } else if( fsmStateChangeReason.equals(StateChangeReason.RUN_COMP_TURN_SEQUENCE) ) {
                    activeMotionType = MotionType.SEQUENCE;
                }
                
                elapsedTimeOfActiveMotion = 0;
                fsmStateChangeRequested = false;
                stateChangeStarted = true;
                compTurnSeqMotionEnded = false;
            }
        }
        
//        if(firstUpdate) {
//            firstUpdate = false;
//        } else {
//            dist_ = physicsVehicle.getPhysicsLocation().distance(prevPhysLoc_);
//            timeDiff_ = System.currentTimeMillis() - prevTime_;
//            System.out.println("Distance: " + dist_);
//            System.out.println("Time: " + timeDiff_);
//            System.out.println("Speed: " + (dist_/timeDiff_));
//        }
//        prevPhysLoc_ = physicsVehicle.getPhysicsLocation();
//        prevTime_ = System.currentTimeMillis();
        
        
        
        
        
        //interFrameCounter += tpf;
        
//        timer1.update();
//        
//        System.out.println("============================================");
//        System.out.println("Time (seconds): " + timer1.getTimeInSeconds());
//        System.out.println("Time (ticks): " + timer1.getTime());
//        System.out.println("Frame rate: " + timer1.getFrameRate());
        //System.out.println("Resolution: " + timer1.getResolution());
         
        
//        vehiclePhysClock += tpf;
//        System.out.println(vehicle.getName()+": Physics time: "+vehiclePhysClock);
//        
//        currPhysTime = vehiclePhysClock;
//        
        
//        currPhysTime = timer1.getTimeInSeconds() - interFrameCounter;
//        currPhysTime = System.currentTimeMillis() - interFrameCounter;
        
//        System.out.println("Physics tick count: " + physTickCount);
//        
//        currPhysTime = physTickCount;
//        
//        currPhysLoc = this.physicsVehicle.getPhysicsLocation();
//        if(prevPhysLoc != null && prevPhysTime != 0) {
//            float dist = currPhysLoc.distance(prevPhysLoc);
//            System.out.println("===============================================");
//            System.out.println(vehicle.getName()+": Distance traveled since last update: " + dist);
//            System.out.println(vehicle.getName()+": Calculated vehicle speed: "+(dist/((currPhysTime - prevPhysTime))));
//            System.out.println(vehicle.getName()+": Actual vehicle speed: "+activeVelocity.length());
//        }
//        prevPhysLoc = currPhysLoc;
//        prevPhysTime = currPhysTime;
        
        // =====================================================================
        // Vehicle's state machine
        // =====================================================================
        switch(state) {
            
            //========\\
            case Drop:
            //========//
                
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": DROP state ..."); }

                if(physicsVehicle.getPhysicsLocation().y < MOTION_START_DEPTH) { 
                    
                    sim.reportReleasedIntoWater(getVehicleName());
                    
                    if(DEBUG_OVERRIDE_TARGET_FOUND) {
                        // Reduce vehicle speed
                        activeVelocity = activeVelocity.normalize().mult(AT_TASK_VEH_SPEED);
                        //DEBUG_PATH_REPLOT = 10;
                        state = State.DoTask;
                        break;
                    }
                    
                    runExploreSubCycle(tpf, false);
                    //System.out.println("Move (in drop state) ...");
                    
                    if( enabled(SimulationMode.GENERAL_TESTS_MODE) ) {
                        state = State.Update;
                    }
                    
                    if( enabled(SimulationMode.INTEGRATION_MODE) ) {
                        estimateSelfPosition();
                        taskIntegAlgorithm.start();
                        state = State.Update;
                    }
                    
                    if( (
                            enabled(SimulationMode.INIT_SELF_ORG_MODE) /*|| 
                           (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))*/ 
                        ) && !flockAlgorithm.started() ) {
                        // Initialization
                        estimateSelfPosition();
                        selfOrgAlgorithm.start();
                        flockAlgorithm.start();
                        state = State.Update;
                    }
                    
                    if( 
                        (enabled(SimulationMode.SEARCH_MODE) /*||                
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))*/)
                      ) {
                        if(USE_VTS_SEARCH) { 
                            estimateSelfPosition(); 
                            estimatedInitPos = estimatedSelfPos.clone();
                        }
                        state = State.Search;
                    }
                    
                    if(targetFound) {
                        if( (
                                enabled(SimulationMode.SEARCH_MODE) ||
                               (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))
                            ) ) {
                            state = State.Update;
                            break;
                        }
                        state = State.DoTask; 
                    }
                }
                break;
                
            //========\\
            case Search:
            //========//
                
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": SEARCH state ..."); }

                if( USE_RPSO_SEARCH ) {
                    // Initialization
                    estimateSelfPosition();
                    selfBestPointer = null;
                    selfBestPos = null;//estimatedSelfPos;
                    selfBestSol = 0;
                } else if( USE_CSF_SEARCH ) {
                    // Initialization if any
                } else if( USE_VTS_SEARCH ) {
                    // Initialization if any
                } else if( USE_DHCP_SEARCH ) {
                    // Initialization if any
                } else if( USE_SDHCP_SEARCH ) {
                    // Initialization if any
                } else if( USE_SSDHCP_SEARCH ) {
                    // Initialization if any
                } else if( USE_DFHCP_SEARCH ) {
                    // Initialization if any
                } else if( USE_SRW_SEARCH ) {
                    // Initialization if any
                } else if( USE_SSW_SEARCH ) {
                    // Initialization if any
                } else if( USE_FSRW_SEARCH ) {
                    // Initialization if any
                }
                
                searchAlgorithm.start();
                
                if( USE_RPSO_SEARCH && LEARNING_ENABLED ) {
                    learningAlgorithm.start();
                }
                
                state = State.Update;
                
                if(targetFound) { 
                    if( (
                            enabled(SimulationMode.SEARCH_MODE) ||
                           (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))
                        ) ) { break; }
                    state = State.DoTask; 
                }
                break;  
            
            //========\\ 
            case Update:
            //========//
                
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": UPDATE state ..."); }
                
                if( missionStartTime == 0 ) {
                    missionStartTime = getCurrentTick();
                }
                
                if( DEBUG_MISSION_TIME_ENABLED ) {
                    System.out.println("Elapsed mission time: " + (getCurrentTick() - missionStartTime));
                }
                
                if( vehExceededBoundary() ) { 
                    sim.getRootNode().detachChild(spatial);
                    //sim.removeVehicleRecords(spatial.getName());
                    break;
                }

                if( stateChangeStarted ) {

                    if( fsmStateChangeReason.equals(StateChangeReason.RUN_MOTION_ALG) ) {
                        switch( switchToMotionAlgorithm ) {
                            case LOG_SPIRAL:
                            case RANDOM_WALK_BIG_JUMPS:
                            case RANDOM_WALK_SMALL_JUMPS:
                                searchAlgorithm.applyUpdateRules();
                                break;
                            case FLOCKING:
                                flockAlgorithm.applyUpdateRules();
                                break;
                            case SAME_AS_BEFORE:
                            case PREDEFINED_MISSION_STAGE_ALG:
                                updateActiveMissionAction(tpf);
                                break;
                            case SURFACING:
                                surfacingAlgorithm.applyUpdateRules();
                            default:
                                motionAlgorithm.applyUpdateRules();
                        }
                    } else if( fsmStateChangeReason.equals(StateChangeReason.RUN_COMP_TURN_SEQUENCE) ) {
                        // Nothing to be done ...
                    }
                    
                    ++elapsedTimeOfActiveMotion;
                    
                } else {

                    //System.out.println("Came here (a) ...");
                    
                    if(enabled(SimulationMode.INTEGRATION_MODE)) {
                        
                        if(USE_VTS_SEARCH && !initPosStored) { 
                            estimateSelfPosition(); 
                            estimatedInitPos = estimatedSelfPos.clone();
                            initPosStored = true;
                        }
                        
                        taskIntegAlgorithm.applyUpdateRules(); 
                        //System.out.println("Came here (b) ...");
                    }
                    
                    if(enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.SOURCE_SEARCH)) { 
                        surfacingAlgorithm.applyUpdateRules(); 
                        if( physicsVehicle.getPhysicsLocation().y > MAX_PT.y ) {
                            sim.reportVehicleSurfaced(getVehicleName());
                            sim.getRootNode().detachChild(spatial);
                            break;
                        }
                    }
                    //System.out.println("Came here (c) ...");
                    
                    if(enabled(SimulationMode.TASK_ALLOC_MODE) ||
                            enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION)) {
                        
                        //System.out.println("Came here (d) ...");
                        
                        if( !exploitingTarget ) {
                            //System.out.println("Came here (e) ...");
                            if(inPlaceTurnActivated) {
                                //System.out.println("Came here (f) ...");
                            }
                            taskAllocAlgorithm.applyUpdateRules(tpf);
                            //System.out.println("Came here (g) ...");
                        } else {
                            //System.out.println("Still exploiting target!");
                        }
                    }
                    
                    //System.out.println("Came here (h) ...");
                    
                    if( enabled(SimulationMode.GENERAL_TESTS_MODE) || 
                        (
                           (
                            (enabled(SimulationMode.SEARCH_MODE) ||
                            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) ||
                            
                            (enabled(SimulationMode.TASK_ALLOC_MODE) ||
                            (enabled(SimulationMode.INTEGRATION_MODE) && (activeMissionStage.equals(MissionStage.TASK_ALLOCATION) || 
                                activeMissionStage.equals(MissionStage.SOURCE_SEARCH))))
                            )
                            && inPlaceTurnActivated
                        ) 
                      ) {
                        
                            //System.out.println("Came here (i) ...");
                            if( enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.SOURCE_SEARCH) ) {
                                executeInPlaceTurning(tpf, true);
                            } else {
                                executeInPlaceTurning(tpf, false);
                                //System.out.println("Came here (j) ...");
                            }
                    }

                    if(
                        enabled(SimulationMode.INIT_SELF_ORG_MODE) || 
                       (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))
                      ) { selfOrgAlgorithm.applyUpdateRules(); }  
                
                    if(
                        (enabled(SimulationMode.SEARCH_MODE) ||
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) && 
                         !inPlaceTurnActivated
                      ) 
                    { 
                        searchAlgorithm.applyUpdateRules(); 
                        //System.out.println("Came here (k) ...");
                    }
                    
                }

                //System.out.println("Came here (l) ...");
                
                if(turnNotNeeded) {
                    state = State.Move;
                    turnNotNeeded = false;
                    //System.out.println("Came here (m) ...");
                } else {
                    state = State.Turn;
                    //System.out.println("Came here (n) ...");
                }
                
                if(  (enabled(SimulationMode.TASK_ALLOC_MODE) ||
                     (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))) 
                      && moveOnToNextTarget ) {
                    //System.out.println("Target found was reset ...");
                    targetFound = false;
                    temporarilySkippedTargets.add(closestTargetName);
//                    System.out.print("Skipped targets so far: ");
//                    for(String tar : temporarilySkippedTargets) {
//                         System.out.print(tar + " ");
//                    }
//                    System.out.println();
                    //System.out.println("Came here (o) ...");
                }
                
                if(targetFound && 
                        !(enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.SOURCE_SEARCH))) { 
                    //System.out.println("Came here (p) ...");
                    if( 
                        (enabled(SimulationMode.SEARCH_MODE) ||
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) 
                      ) {
                        state = State.Update;
                        //System.out.println("Came here (q) ...");
                        break;
                    }
                    state = State.DoTask;
                    //System.out.println("Came here (r) ...");
                }
                break;
               
            //========\\
            case Move:
            //========//   
                
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": MOVE state ..."); }

                if( residualEnergy <= FAILURE_ENERGY ) { 
                    state = State.Idle;
                    setActiveVelocity(activeVelocity.multLocal(VEHICLE_FAILURE_SPEED_SCALE));
                    break;
                }
                
                if( vehExceededBoundary() ) { 
                    sim.getRootNode().detachChild(spatial);
                    //sim.removeVehicleRecords(spatial.getName());
                    break;
                }
                
                runExploreSubCycle(tpf, true);
                //System.out.println("Move (in move state) ...");
                
                state = State.Update;
                if(targetFound) { 
                    if( 
                        (enabled(SimulationMode.SEARCH_MODE) ||
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
                      ) { break; }
                    state = State.DoTask; 
                }
                //timeSinceLastSearchStep = System.currentTimeMillis();
                break;
                
            //========\\
            case Turn:
            //========//
                
                // Debugging
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": TURN state ..."); }
                
                //System.out.println("Target found at this point (1)? " + targetFound);
                
                // Residual energy related
                if( residualEnergy <= FAILURE_ENERGY ) { 
                    resetCompositeTurnFlags();
                    state = State.Idle;
                    setActiveVelocity(activeVelocity.multLocal(VEHICLE_FAILURE_SPEED_SCALE));
                    break;
                }
                
                // Boundary checks
                if( vehExceededBoundary() ) { 
                    sim.getRootNode().detachChild(spatial);
                    //sim.removeVehicleRecords(spatial.getName());
                    break;
                }
                
                // Target Check
                // NEW - START
                // If, at any time, the target is found, reset all flags and 
                // move to "Update" state to transfer control to the search 
                // algorithm and let it make the decision about which state to
                // move to.
                if( targetFound && 
                   (
                     ((enabled(SimulationMode.SEARCH_MODE) ||
                      (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))) ||
                        
                     ((enabled(SimulationMode.TASK_ALLOC_MODE) ||
                      (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))))
                   )    
                  ) {
                    
                    //System.out.println("Target set to found again!");
                    
                    if( !(searchAlgorithm != null && !searchAlgorithm.getClass().isInstance(SSDHCP.class)) && 
                            !(enabled(SimulationMode.TASK_ALLOC_MODE) && moveOnToNextTarget)) {
                        resetCompositeTurnFlags();
                    }

                    //System.out.println("Redirection to Update state through the Turn state ...");
                    state = State.Update;
                    //System.out.println("Broke from turn state to update state: "+spatial.getName());
                    break;
                }
                // NEW - END
                
//                if(System.currentTimeMillis() - timeSinceLastSearchStep > INTER_SEARCH_TIME) {
//                    compositeTurnInProgress = false;
//                }
                
                if(compositeTurnInProgress) {
                    
                    if( (inPlaceTurnActivated || targetSweepingModeActive) && sequenceBreakSolicited && 
                            !activeMissionStage.equals(MissionStage.SOURCE_SEARCH) ) {
                        
                        resetCompositeTurnFlags();
                        updateVehicleSpeed(AT_TASK_VEH_SPEED);
                        //sequenceBreakSolicited = false;
                        
                        System.out.println("This is where it broke in Turn State: "+spatial.getName());
                        System.out.println("inPlaceTurnActivated: "+inPlaceTurnActivated);
                        System.out.println("targetSweepingModeActive: "+targetSweepingModeActive);
                        System.out.println("sequenceBreakSolicited: "+sequenceBreakSolicited);
                        
                        state = State.Update;
                        break;
                    }
                    
                    if( USE_SDHCP_SEARCH && sequenceBreakSolicited ) {
                        resetCompositeTurnFlags();
                        updateVehicleSpeed(AT_TASK_VEH_SPEED);
                        sequenceBreakSolicited = false;
                        state = State.DoTask;
                        break;
                    }
                    
                    if( !seqStartTimeCaptured ) {
                        sequenceStartTime = sim.getCurrentTick();
                        seqStartTimeCaptured = true;
                    }
                    
                    if(
                        (enabled(SimulationMode.SEARCH_MODE) ||
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) && 
                         USE_RPSO_SEARCH && LEARNING_ENABLED && LEARNING_PHASE_ACTIVE && rewardAccumStarted) {
                        learningAlgorithm.applyUpdateRules();
                        rewardAccumStarted = false;
                        compositeTurnJustEnded = false;
                    }

                    //System.out.println("Composite turn started ...");
                    
                    if( !turnInProgress && !travelInProgress ) {
                        
                        if( compositeTurnStepsDone < compositeTurnSeq.size() ) {

    //                        System.out.println("Started turn sequence checking ...");

                            // First, check if composite turn seq. has been 
                            // interrupted and if so, get remaining seq. steps
                            // and store them in "savedSequenceStates" for later
                            // continuation
                            if( motionInterrupted ) {
                                CompositeTurnSequence seqToSave = new CompositeTurnSequence();
                                for( int s = compTurnStepInd; s < compositeTurnSeq.size(); ++s ) {
                                    seqToSave.add(compositeTurnSeq.getSequence(s));
                                }
                                savedSequenceStates.push(seqToSave);
                                resetCompositeTurnFlags();
                                if( interruptingMotion.equals(MotionType.SEQUENCE) ) {
                                    if( switchToMotionSequence != null ) {
                                       compositeTurnSeq = switchToMotionSequence; 
                                    } else {
                                        throw new NullPointerException("No motion sequence to switch to is set!");
                                    }
                                }
                                //System.out.println("This is where it broke (2) in Turn State: "+spatial.getName());
                                state = State.Update;
                                break;
                            }
                            
                            Pair<String,Object> currCompTurnStep = 
                                    compositeTurnSeq.getSequence(compTurnStepInd);

                            if(currCompTurnStep.key.equals("turn_direction")) {

    //                            System.out.println("Turn direction entered ...");

                                turnDirection = (String)currCompTurnStep.val;
                                compositeTurnStepsDone++;
                                currCompTurnStep = compositeTurnSeq.getSequence(++compTurnStepInd);

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
                                
                                if( targetSweepingModeActive ) {
                                    ++sweepNumber;              
                                    if(DEBUG_TARGET_SWEEPING) System.out.println("Sweep number: " + sweepNumber);
                                }
                                
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
                    
                    
                    // New - Modifying PSO
//                    prevSelfBestSol = selfBestSol;

                    //System.out.println("Turn steps done: "+turnStepsDone);
                    if(turnInProgress) {
                        
                        // =========================================
                        // New: 7/12/2016
                        if( activeTurnRadius == MIN_TURN_RADIUS ) {
                            updateVehicleSpeed(MIN_VEH_SPEED);
                        } else {
                            updateVehicleSpeed(MAX_VEH_SPEED);
                        }
                        // =========================================

//                        System.out.println("Turning in progress ...");
//                        System.out.println("Turn step: "+turnStepsDone+", turn resolution: "+TURN_RESOLUTION);
//                        if( turnStepsDone > 2 * TURN_RESOLUTION ) { // Check why you had to add 2
//                            resetTurnFlags();
//                            break;
//                        }
                        if(!turningStarted) {
                            turningStarted = true;
                            
                            // New
                            //vehSpeedSnapshot = activeVelocity.length();
                            //activeVelocity = activeVelocity.normalize().mult(VEH_TURNING_SPEED);
                            // New
                            
                            //System.out.println("Going to start turning ...");
                            turn(turnDirection, turnAngle, tpf);
                        }
                        traveledDist = turnIncStartLoc.distance(physicsVehicle.getPhysicsLocation());
                        //System.out.println("Traveled Dist: "+traveledDist);
                        
                        // =====================================================
                        // Revised code - START
                        // =====================================================
                        if(traveledDist >= chordLen && !turnCompleted) {
                            turnStepsDone++;
                            turn(turnDirection, turnAngle, tpf); 
                        }
                        
                        runExploreSubCycle(tpf, true);
                        //System.out.println("Target found at this point (2)? " + targetFound);
                        
                        //System.out.println("Move (turn in progress) ...");
                        
                        if(traveledDist >= chordLen && turnCompleted) {
                            resetTurnFlags();
                            break;
                        }
                        // =====================================================
                        // Revised code - END
                        // =====================================================
                        // =====================================================
                        // Original code - START
                        // =====================================================
//                        if(traveledDist < chordLen) {
//                            runExploreSubCycle(tpf, true);
//                        } else if(turnCompleted){
//                            runExploreSubCycle(tpf, true);
//                            resetTurnFlags();
//                            break;
//                        } else {
//                            turnStepsDone++;
//                            turn(turnDirection, turnAngle, tpf);  
//                            runExploreSubCycle(tpf, true);
//                        }
                        // =====================================================
                        // Original code - END
                        // =====================================================
                        
                    } else if(travelInProgress) {
                        
                        // =========================================
                        // New: 7/12/2016
                        updateVehicleSpeed(MAX_VEH_SPEED);
                        // =========================================

                        traveledDist = travelStartLoc.distance(physicsVehicle.getPhysicsLocation());
//                        System.out.println("Traveled Dist: "+traveledDist+", Distance to travel: "+distToTravel);
                        
                        runExploreSubCycle(tpf, true);
                        //System.out.println("Move (travel in progress) ...");
                        
                        if(traveledDist >= distToTravel) {
                            //System.out.println("travel completed ... ("+vehicle.getName()+")");
                            resetTravelFlags();
                        }
                        
                        if( targetSweepingModeActive ) {
                            applyTargSweepRules(); 
                        }
                        
                    } else {
                        runExploreSubCycle(tpf, true);
                        //System.out.println("Move just after turn completed ...");
                    }
                    
                    // New - modifying PSO
//                    if(prevSelfBestSol != null && selfBestSol != null && selfBestSol > prevSelfBestSol) {
//                        resetCompositeTurnFlags();
//                        state = State.Update;
//                    }
                    
                    //state = State.Update;   // NEW 2
                    
                } else {
                    
                    if( stateChangeStarted ) {
                        compTurnSeqMotionEnded = true;
                        notifyFSMStateChangeReqEnded();
                    }
                    
                    compositeTurnJustEnded = true;
                    
                    if(
                        (enabled(SimulationMode.SEARCH_MODE) ||
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) && 
                         USE_RPSO_SEARCH && LEARNING_ENABLED && LEARNING_PHASE_ACTIVE && !rewardAccumStarted) {
                        rewardBucket = 0;
                        rewardAccumStarted = true;
                    }
                    
                    runExploreSubCycle(tpf, true);
                    //System.out.println("Move after explore subcycle ...");
                    resetCompositeTurnFlags();
                    
                    if(
                        (enabled(SimulationMode.SEARCH_MODE) ||
                        (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) || 
                            
                        (enabled(SimulationMode.INIT_SELF_ORG_MODE) || 
                        (enabled(SimulationMode.INTEGRATION_MODE) && 
                            (activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION) || activeMissionStage.equals(MissionStage.SOURCE_SEARCH)))) ||    
                            
                         enabled(SimulationMode.GENERAL_TESTS_MODE)
                      ) {   // Search mode
                        
                        if(USE_RPSO_SEARCH) {
                            // Update speed: Velocity's magnitude is the speed in the new direction.
                            // Please not that this is taking place after the turn has completed. For
                            // turning speed, look at updateVehicleVelocity()
                            activeSpeed = searchUpdateVelocity.length();
                            activeVelocity = activeVelocity.normalize().mult(activeSpeed);
                            limitVehicleSpeed();
                        }
                       
                       //System.out.println("This is where it broke (3) in Turn State: "+spatial.getName());
                       state = State.Update;
                    }
                    
//                    if(sim.getStateManager().getState(GeneralTestsMode.class).isEnabled()) {
//                        if(searchUpdateVelocity != null) {
//                            // Update speed
//                            //activeSpeed = searchUpdateVelocity.length();
//                            //activeVelocity = activeVelocity.normalize().mult(activeSpeed);
//                            limitVehicleSpeed();
//                        }
//                        state = State.Update;
//                    }
                    
                }
                
                //System.out.println("Target found at this point (3)? " + targetFound);
                
                // Used in task allocation mode
                if(/*targetFound ||*/ 
                    (sim.getStateManager().getState(TaskAllocMode.class) != null &&
                        sim.getStateManager().getState(TaskAllocMode.class).isEnabled()) ||
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))
                  ) { 
                    
                    if( !taskAllocAlgorithm.started() ){
                        taskAllocAlgorithm.start();
                        state = State.DoTask; 
                    } else if( inPlaceTurnActivated ) {
                        if( !inPlaceTurnStarted ) {
                            executeInPlaceTurning(tpf, false);
                            activeTurnRadius = solicitedTurnRadius;
                            inPlaceTurnStarted = true;
                        } else {
                            taskAllocAlgorithm.applyUpdateRules(tpf);
                        }
                    }
                    
                    if( compositeTurnInProgress ) {
                        //System.out.println("This is where it broke (4) in Turn State: "+spatial.getName());
                        state = State.Turn;
                    } else {
                        //System.out.println("Why came here??");
                        state = State.DoTask;
                    }
                }
                
                break;
                
            //========\\
            case DoTask:
            //========//
                
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": DO_TASK state ..."); }
                
                    if( enabled(SimulationMode.TASK_ALLOC_MODE) && !taskAllocAlgorithm.started() ) {
                        taskAllocAlgorithm.start();
                    }
                
                //if(!targetFound) {
//                    generateRandomTurn();
//                    state = State.Turn;
                //}
                
                if(
                    enabled(SimulationMode.SEARCH_MODE)
//                    (enabled(SimulationMode.SEARCH_MODE) ||
//                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
                  ) {
                    
                    if( !(searchAlgorithm.getClass().isInstance(SSDHCP.class) ) ) { // Shouldn't stop in the case of SSDHCP
                        System.out.println("Entered!!!");
                        setActiveVelocity(activeVelocity.multLocal(VEHICLE_FAILURE_SPEED_SCALE));
                    } else {
                        state = State.Update;
                        System.out.println("Did not enter :)");
                    }
                    
                } else if(
                    enabled(SimulationMode.TASK_ALLOC_MODE) ||
                   (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))    
                         ) {

                    if(exploitingTarget) {
                        
                        if(DEBUG_TASK_ALLOC) System.out.println("Entered: 'exploiting target'");
                        
                        long timeToCompareTo = (DIFFERENTIATE_PROCESSING_TIMES ? 
                                (!closestIsPreferred ? OTHER_TYPE_EXPLOITATION_TIME : EXPLOITATION_TIME) : EXPLOITATION_TIME);
                        
                        if(DEBUG_EXPLOITATION) System.out.println("Comparing elapsed exploitation time to: " + timeToCompareTo);
                        if(DEBUG_EXPLOITATION) System.out.println("Preferred type: " + preferredTargetType.toString() + ", processing: " +sim.getTargetType(closestTargetName));
                        
                        if( System.currentTimeMillis() - timeExploitationStarted > timeToCompareTo || taskAllocAlgorithm.stopped()) {
                            
                            if(DEBUG_EXPLOITATION) System.out.println("Done exploiting. "+this.getVehicleName());
                            
                            // Get previously unprocessed targets
                            String[] newlySensedTargs =  getSensedTargetsList();
                            
                            // Create a list of only the ones that the AUVs is 
                            // currently allowed to process
                            ArrayList<String> newlyProcessTargsList = new ArrayList(newlySensedTargs.length);
                            if( typeOfTargToProcess != null ) {
                                for( String inRangeTarg : newlySensedTargs ) {
                                    if( sim.getTargetType(inRangeTarg).equals(typeOfTargToProcess) ) {
                                        newlyProcessTargsList.add(inRangeTarg);
                                    }
                                }
                                // added it to processed targets ... can now unset type to process
                                unsetTypeOfTargToProcess();
                            } else {
                                newlyProcessTargsList.addAll(Arrays.asList(newlySensedTargs));
                            }
                            
                            // Add them to the processed targets list
                            processedTargetsList.addAll(newlyProcessTargsList);
                            
                            ArrayList<String> newlyProcessedTargs = new ArrayList<String>(newlySensedTargs.length);
                            newlyProcessedTargs.addAll(Arrays.asList(newlySensedTargs));
                            
                            // Report them to the simulator as the newly processed targets
                            sim.reportProcessedTargets(newlyProcessedTargs);
                            
                            exploitingTarget = false;
                            targetFound = false;
                            moveOnToNextTarget = true;
                            
                            if(USE_TETHERED_RW) {
                                // This is a lazy approach used only for time restrictions (implement full mechanism in the future)
                                seqStartPos = physicsVehicle.getPhysicsLocation();
                                countSeqExecsWithoutTarg = 0;
                            }
                            
                            state = State.Update;
                            
                        } else {
                            if(DEBUG_EXPLOITATION) System.out.println("Exploitation time hasn't passed yet. "+this.getVehicleName() + ", Diff: " + (System.currentTimeMillis() - timeExploitationStarted));
                        
                            // -------------------------
                            // To avoid stopping an trying to process while that target is actually not in range now
                            sense();
                            if(!targetFound || (typeOfTargToProcess != null && !isTargTypeAllowedForProcessInRange()) ) {
                                
//                                System.out.println("This is where it fails ...");
//                                activeVelocity.set(activeVelocity.normalize().mult(AT_TASK_VEH_SPEED));
//                                state = State.Turn;
//                                break;
                                exploitingTarget = false;
                                exploitTarget = false;
                                moveOnToNextTarget = true;
                                targetFound = false;
                                
                                if(USE_TETHERED_RW) {
                                    // This is a lazy approach used only for time restrictions (implement full mechanism in the future)
                                    seqStartPos = physicsVehicle.getPhysicsLocation();
                                    countSeqExecsWithoutTarg = 0;
                                }

                                state = State.Update; 
                                
                            }
                            // -------------------------
                        
                        }
                    } else {

                        if(taskAllocAlgorithm.started() && prevFSMState != State.Update) {
                            
                            if(DEBUG_TASK_ALLOC) System.out.println("Trying to apply update rules of the task alloc alg");
                            taskAllocAlgorithm.applyUpdateRules(tpf);
                            if(compositeTurnInProgress) {
                                state = State.Turn;
                            }  
                        }

                        if(exploitTarget     
                                // Disabled it as it did not make sense ... not sure if it will break something!! Check
//                                &&                           
//                                (timeExploitationStarted == 0 || 
//                                (System.currentTimeMillis() - timeExploitationStarted > EXPLOITATION_TIME)) 
                          ) {
                            
                            if(DEBUG_TASK_ALLOC) System.out.println("Entered: 'exploit target'");
                            
                            // Found a target and started exploiting it ... clear skipped targets to allow rediscovery
                            if( !temporarilySkippedTargets.isEmpty() ) {
                                temporarilySkippedTargets.clear();
                            }
                            
                            setActiveVelocity(activeVelocity.multLocal(VEHICLE_FAILURE_SPEED_SCALE));
                            
                            timeExploitationStarted = System.currentTimeMillis();
                            exploitingTarget = true;
                            
                            if(DEBUG_EXPLOITATION) System.out.println("New exploitation triggered. "+this.getVehicleName());
                            
                        } else if(moveOnToNextTarget) {     // Find the next target
                            
                            if(DEBUG_TASK_ALLOC) System.out.println("Entered: 'move on to next target'");
                            
                            if( taskAllocAlgorithm.stopped() ) {
                                state = State.Update;
                                break;
                            }
                            
                            // Reset target found flag as current target has already been processed
                            targetFound = false;
                            
                            if(physicsVehicle.getLinearVelocity().length() <= VEHICLE_FAILURE_SPEED_SCALE) {      // If was processing a target
                                activeVelocity.set(initialVelocity.normalize().mult(AT_TASK_VEH_SPEED));
                            }
                            
                            if(!compositeTurnInProgress) {
                                if( !straightTravelRequested() ) {
                                    //generateRandomTurn();
                                    //generateLimitingRandomTurn();
                                    
                                    if(USE_TETHERED_RW) {
                                        genTetheredRandTurn();
                                    } else if(USE_BUBBLE_CHAIN_RW) {
                                        genBubbleChainRandWalk();
                                    } else if(USE_RETRACTED_SEQ_RW) {
                                        genRetractedSeqRandTurn();
                                    }
                                    
                                    //flock();
                                } else {
                                    System.out.println("Straight travel requested!!");
                                    sense();
                                }
                            } else {
                                System.out.println("There is an active turn sequence!!");
                            }                       
                            state = State.Turn;
                        }
                    }
                }

                break;
                
            //========\\
            case Follow:
            //========//
                
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": FOLLOW state ..."); }
                
                follow(followTurnRadius, neighborEstVel, tpf);
                break;
                
            //========\\
            case Idle:
            //========//
               
                if(STATE_TRACKER_ENABLED) { System.out.println(this.getVehicleName() + ": IDLE state ..."); }
                break;
        }
        
        prevFSMState = state;
        
//        if( BRAIN_MODE_ENABLED ) {
//            brain.makeDecision();
//        }
        
        firstFSMUpdate = false;
    }
    
    private boolean isTargTypeAllowedForProcessInRange() {
        
        // Get previously unprocessed targets
        String[] newlySensedTargs =  getSensedTargetsList();
        
        for( String inRangeTarg : newlySensedTargs ) {
            if( sim.getTargetType(inRangeTarg).equals(typeOfTargToProcess) ) {
                return true;
            }
        }
        
        return false;
    }
    
    private void runExploreSubCycle(float tpf, boolean communicate) {
        
        move(tpf);
        
        if( enabled(SimulationMode.GENERAL_TESTS_MODE) || inPlaceTurnActivated || 
                (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.SOURCE_SEARCH))) {
            return;
        }
        
        sense();
        
        if(
            (enabled(SimulationMode.SEARCH_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) && 
             USE_RPSO_SEARCH && LEARNING_ENABLED && LEARNING_PHASE_ACTIVE && compositeTurnJustEnded && rewardAccumStarted
          ) {
            rewardBucket += iterPartCount;
            //System.out.println("Current reward:"+rewardBucket);
        }
        
        if( USE_RPSO_SEARCH ) {
            processData();
        }
        
        if( USE_RPSO_SEARCH && communicate ) {
            communicate();
        } 
    }
    
    private void executeInPlaceTurning(float tpf, float turnRadius, String direction, boolean forFloating) {
        
        updateTurningParams(turnRadius);
        //move(tpf);
        turnNotNeeded = false;
        compositeTurnInProgress = true;
        seqStartTimeCaptured = false;
        
        // The multiplier has been added to reduce the cumulative error that
        // results from adding errors at the end of each full turn angle
        compositeTurnSeq.add("turn_direction", direction);
        compositeTurnSeq.add("turn_angle", (forFloating ? 1 : 1000) * FastMath.TWO_PI);
    }
    
    private void executeInPlaceTurning(float tpf, boolean forFloating) {
        if( solicitedTurnRadius != 0 ) {
            executeInPlaceTurning(tpf, solicitedTurnRadius, solicitedTurnDirection, forFloating);
        } else {
            executeInPlaceTurning(tpf, MIN_TURN_RADIUS, "left", forFloating);
        }
        
    }
    
    private void communicate() {
        
        if(
            enabled(SimulationMode.TASK_ALLOC_MODE) ||
           (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))
          ) { return; }
        
        if(commSend) {  // Send
           broadcastSelfBestPos();
        } else {    // Receive
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
    
    private void resetCompositeTurnFlags() {
        compositeTurnInProgress = false;
        compositeTurnStepsDone = 0;
        compTurnStepInd = 0;
        seqStartTimeCaptured = false;
        compositeTurnSeq.clear();
        turnNotNeeded = false;
        resetTurnFlags();
        resetTravelFlags();
        inGracePeriod = true;
        elapsedGracePeriodTicks = 0;
    }
    
    private void resetGracePeriodVars() {
        inGracePeriod = false;
        elapsedGracePeriodTicks = 0;
    }
    
    public void limitVehicleSpeed() {
        
        activeSpeed = activeVelocity.length();
        
        if( activeSpeed < MIN_VEH_SPEED) {
            activeSpeed = MIN_VEH_SPEED;
        } else if( activeSpeed > MAX_VEH_SPEED) {
            activeSpeed = MAX_VEH_SPEED;
        }
        
        activeVelocity = activeVelocity.normalize().mult(activeSpeed);
    }
    
    public void updateTurningParams(float turnRadius) {
        if(turnRadius < MIN_TURN_RADIUS) {
           turnRadius = MIN_TURN_RADIUS;
        }
        activeTurnRadius = turnRadius;
        activeChordLen = 2*turnRadius*FastMath.sin(MAX_TURN_ANGLE);     
    }
    
    @Override
    public void move(float tpf) {
        
        limitVehicleSpeed();
        
        //vehicle.setHeadingDir(activeVelocity);
        
        if(accelerateUpwards) {
            //System.out.println("Accelerating upwards for some reason!");
            if( solicitedUpwardsAccel != 0 ) {
                accelerateUpwards(solicitedUpwardsAccel);
            } else {
                accelerateUpwards(UPWARD_ACCELERATION);
            }
            
        }
        
        if(decelerate) {
            decelerate();
        }
        
        normVelVec = activeVelocity.normalize();
        if(VEHICLE_PATH_DEBUG && vehPathCounter == DEBUG_PATH_REPLOT && activeVelocity.length() > 0) {
            DebugUtils.plotArrow( physicsVehicle.getPhysicsLocation(), normVelVec, DEBUD_PATH_COLOR, DEBUD_PATH_WIDTH, sim);
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
        
//        if(normVelVec.dot(Vector3f.UNIT_Y) < 0) {
//            angleWithY = - angleWithY;
//        }

        yawXAngle.fromAngleAxis(angleWithZ, Vector3f.UNIT_Y);

        //physicsVehicle.setPhysicsRotation(yawXAngle);   
        
//        if( angleWithY > FastMath.HALF_PI ) {
            pitchYAngle.fromAngleAxis(angleWithY - FastMath.HALF_PI, vehPitchAxis);
//        } else {
//            pitchYAngle.fromAngleAxis(angleWithY, Vector3f.UNIT_X);
//        }
        vehRot.set(pitchYAngle);
        vehRot.multLocal(yawXAngle);
            
        physicsVehicle.setPhysicsRotation(vehRot);
        
//        if(physicsVehicle.getPhysicsLocation().y < 15) {
//            activeVelocity.add(new Vector3f(0, 0.05f, 0));
//        }
        
        physicsVehicle.setLinearVelocity(activeVelocity);
        keepInBounds();  
        
//        if(Math.random() > 0.995f) {
//            state = State.Turn;
//        }
        
        // =====================================================================
        // Position tracking
        
        // Targetted approach
        
//        distFromLastPosRecord = prevActVelocity.length() * (System.currentTimeMillis() - lastPosRecordTime) / 1000;
//        
//        if( distFromLastPosRecord > POS_INTER_RECORD_DIST) {
//            if( positionHistory.size() > POSITION_HISTORY_lEN ) {
//                positionHistory.poll();
//            }
//
//            // Estimate current position
//            estimatedSelfPos = prevSelfPosition.addLocal(prevActVelocity.normalize().mult(distFromLastPosRecord));
//
//            // Push it to into history
//            positionHistory.offer(estimatedSelfPos);
//            lastPosRecordTime = System.currentTimeMillis();
//            prevActVelocity = activeVelocity;
//
//            // Store it as previous position for next round
//            prevSelfPosition = estimatedSelfPos;
//
//            if(DEBUG_SELF_POS_HIST) {
//                DebugUtils.plotCrossHair(estimatedSelfPos, 2, 0.5f, ColorRGBA.Red, sim);
//            }
//            
//            System.out.println("Position History of vehicle ("+spatial.getName()+"): " + positionHistory.toString());
//        }
        
        
        // Fake-it approach
        if(!enabled(SimulationMode.GENERAL_TESTS_MODE)) {
            estimateSelfPosition();
        }
        
//        if( DEBUG_SELF_BEST_ANCHOR && selfBestPointer != null ) {
//            DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), selfBestPointer.subtract(physicsVehicle.getPhysicsLocation()), ColorRGBA.Red, 2, sim);
//        }
        
        // =====================================================================
        if( LIMITED_ENERGY_VEHICLE ) {
            
            if( DEBUG_ENERGY_LEVEL_ENABLED ) {
                System.out.println("Residual Energy: " + residualEnergy);
            }
            
            distanceTraveled += tpf * activeVelocity.length();

            if( distanceTraveled >= 1 ) {
                // Energy consumption
                float consumedEnergy = ((WATER_DENSITY
                                        * DRAG_COEFFICIENT
                                        * WETTED_VEHICLE_AREA //vehicle.getSurfaceArea()
                                        * FastMath.pow(activeVelocity.length(), 2)
                                        )/(2 * PROPULSION_EFFICIENCY)) 
                                        + (HOTEL_LOAD / activeVelocity.length());

                residualEnergy -= consumedEnergy;

                //System.out.println("Consumed Energy: " + consumedEnergy);
                //System.out.println("Active Speed: " + activeVelocity.length());

                distanceTraveled = 0.0f;
            }

            //System.out.println("Distance Traveled: " + distanceTraveled);
        }
        // =====================================================================
        
        overallMissionDistTraveled += tpf * activeVelocity.length();
    }
       
    public void processData() {
        
        if(
            enabled(SimulationMode.TASK_ALLOC_MODE) ||
           (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))
          ) { return; }
        
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
//                    ArrayList<Pair> vectors = new ArrayList<Pair>(Solutions.size()-1);
//                    Pair solPair;
//                    Vector3f diffVec;
//                    for(int i = 0; i < Solutions.size() - 1; i++) {
//                        diffVec = (Solutions.get(i).t).subtract(Solutions.get(i+1).t);
//                        solPair = new Pair(Solutions.get(i+1).t, diffVec);
//                        vectors.add(solPair);
//                    }
//                    DebugUtils.plotArrowSequence(vectors, sim);

                    DebugUtils.plotArrow(origin, dest, ColorRGBA.Red, 2, sim);

                }
            }
            
        } else {
            //System.out.println("Position and/or particle buffer not full yet! Size: "+positionHistory.size());
//            posHistPullTrials++;
//            
//            if(posHistPullTrials > MAX_POS_HIST_PULL_TRIALS && prevSelfBestPos != null) {
//                //System.out.println("Cloning ...");            
//                selfBestPointer = prevSelfBestPointer.clone();
//                selfBestPos = prevSelfBestPos.clone();
//                selfBestSol = prevSelfBestSol;
//                if(DEBUG_PREV_SELF_BEST_ANCHOR) {
//                    DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), selfBestPos.subtract(physicsVehicle.getPhysicsLocation()), ColorRGBA.Green, 2, sim);
//                }
//                posHistPullTrials = 0;
//            }
            
        }
        
        
        
        // Store the, just generated, position marker and the corresponding 
        // particle count near that position
//        posToPartCountMap.put(posMarkerHist.peek(), iterPartCount);
        
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
    
    public Vector3f estimatePos() {
        estimateSelfPosition();
        return estimatedSelfPos.clone();
    }
    
    private boolean sense() {
        
        //System.out.println("SENSING!");
        
        if(
            (enabled(SimulationMode.SEARCH_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
          ) {
            northParticlesCount = 0;
            southParticlesCount = 0;
            eastParticlesCount = 0;
            westParticlesCount = 0;
            northPartNames = new ArrayList<String>();
            southPartNames = new ArrayList<String>();
            eastPartNames = new ArrayList<String>();
            westPartNames = new ArrayList<String>();
            iterPartCount = 0;
        }
        
        if(
            (enabled(SimulationMode.TASK_ALLOC_MODE)  || 
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))) ||
                
            (enabled(SimulationMode.INIT_SELF_ORG_MODE) || 
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))) || 
           
            (enabled(SimulationMode.SEARCH_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
          ) {
            northVehiclesCount = 0;
            southVehiclesCount = 0;
            eastVehiclesCount = 0;
            westVehiclesCount = 0;
            bottomVehiclesCount = 0;
            northVehNames = new ArrayList<String>();
            southVehNames = new ArrayList<String>();
            eastVehNames = new ArrayList<String>();
            westVehNames = new ArrayList<String>();
            bottomVehNames = new ArrayList<String>();
            
            staticVehNames = new ArrayList<String>();
            movingVehNames = new ArrayList<String>();
            
            iterVehCount = 0;
        }   
            
        if(
            (enabled(SimulationMode.SEARCH_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) ||
                
            (enabled(SimulationMode.TASK_ALLOC_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION)))
          ) {    
            northTargetsCount = 0;
            southTargetsCount = 0;
            eastTargetsCount = 0;
            westTargetsCount = 0;
            bottomTargetsCount = 0;
            northTargNames = new ArrayList<String>();
            southTargNames = new ArrayList<String>();
            eastTargNames = new ArrayList<String>();
            westTargNames = new ArrayList<String>();
            bottomTargNames = new ArrayList<String>();
            iterTargCount = 0;
        }

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
            //targetFound = false;
            
            //for(float angleX = -sensFieldAngle; angleX < sensFieldAngle; angleX += FastMath.QUARTER_PI * 0.4) {

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
                
                
                if(
                    (enabled(SimulationMode.INIT_SELF_ORG_MODE) ||
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))) ||
                    
                    (enabled(SimulationMode.SEARCH_MODE) || 
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) ||
                        
                    (enabled(SimulationMode.TASK_ALLOC_MODE) ||
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION)))     
                   ) {
                    
                    // Collisions with vehicles
                    CollisionResults vehColRes = new CollisionResults();
                    for(Spatial s: vehicleTargets){
                        // Skip self
                        if( s.getName().equals(spatial.getName()) ) {
                            continue;
                        }
                        s.collideWith(ray, vehColRes);
                    }

                    if(vehColRes.size() > 0){
                      
                        target = vehColRes.getClosestCollision().getGeometry();
                        float targDist = vehColRes.getClosestCollision().getDistance();
                        
                        // Check if this is a vehicle
                        String vName = target.getParent().getParent().getName();
                        
                        //System.out.println("Target being checked: "+vName);

                        if( vName.substring(0, 2).equals("V_") ) {
                            
                            switch(i) {
                                case 0:
                                    if(!northVehNames.contains(vName)) {
                                        northVehNames.add(vName);
                                        northVehiclesCount++;
                                        if( ENABLE_VEH_VEH_COLL_DETECTION ) { trackVehVehCollision(targDist); }
//                                        System.out.println("Vehicle count for "+spatial.getName()+"'s NORTH sensor is: "+northVehiclesCount);
                                    }
                                    break;
                                case 1:
                                    if(!southVehNames.contains(vName)) {
                                        southVehNames.add(vName);
                                        southVehiclesCount++;
                                        if( ENABLE_VEH_VEH_COLL_DETECTION ) { trackVehVehCollision(targDist); }
//                                        System.out.println("Vehicle count for "+spatial.getName()+"'s SOUTH sensor is: "+southVehiclesCount);
                                    }
                                    break;
                                case 2:
                                    if(!eastVehNames.contains(vName)) {
                                        eastVehNames.add(vName);
                                        eastVehiclesCount++;
                                        if( ENABLE_VEH_VEH_COLL_DETECTION ) { trackVehVehCollision(targDist); }
//                                        System.out.println("Vehicle count for "+spatial.getName()+"'s EAST sensor is: "+eastVehiclesCount);
                                    }
                                    break;
                                case 3:
                                    if(!westVehNames.contains(vName)) {
                                        westVehNames.add(vName);
                                        westVehiclesCount++;
                                        if( ENABLE_VEH_VEH_COLL_DETECTION ) { trackVehVehCollision(targDist); }
//                                        System.out.println("Vehicle count for "+spatial.getName()+"'s WEST sensor is: "+westVehiclesCount);
                                    }
                                    break;
                                case 4:
                                    if(!bottomVehNames.contains(vName)) {
                                        bottomVehNames.add(vName);
                                        bottomVehiclesCount++;
                                        if( ENABLE_VEH_VEH_COLL_DETECTION ) { trackVehVehCollision(targDist); }
//                                        System.out.println("Vehicle count for "+spatial.getName()+"'s BOTTOM sensor is: "+bottomVehiclesCount);
                                    }
                                    break;
                            }
                            
                            if( sim.getUWVehiclesMap().get(vName).getActiveVelocity().length() < 1 ) {
                                staticVehNames.add(vName);
                            } else {
                                movingVehNames.add(vName);
                            }
                        }
                    }
                }
                
                if(
                    (enabled(SimulationMode.SEARCH_MODE) || 
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
                  ) {
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
                            //pName = particle.getParent().getName();

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

                        //break;
                    } 
                }

                // Collisions with terrain
                if(sensorName.equals(SensorName.BottomSensor)) {
                    
                    CollisionResults terrColRes = new CollisionResults();

                    List<Spatial> terrainQuads = ((TerrainQuad)((Node)sim.getTerrain()).getChild("terrain")).getChildren();
                    //List<Spatial> terrainQuads = ((TerrainQuad)((Node)sim.getStateManager().getState(SimulationCore.class).getTerrain()).getChild("terrain")).getChildren();

                    for(Spatial s : terrainQuads) {
                        //System.out.println( ((TerrainQuad)s).toString() );
                        ((TerrainQuad)s).collideWith(ray, terrColRes);
                    }     

                    if(terrColRes.size() > 0){

                        CollisionResult hit = terrColRes.getClosestCollision();

                        Vector3f closestTerrainPoint = hit.getContactPoint();

                        float reponseDistance = 0;
                        if(
                            (enabled(SimulationMode.INIT_SELF_ORG_MODE) || 
                            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))) ||
                                
                            (enabled(SimulationMode.SEARCH_MODE) || 
                            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
                          ) {
                            reponseDistance = SENSING_RANGE;
                        } else if(
                            enabled(SimulationMode.TASK_ALLOC_MODE) ||
                            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))
                                 ) {
                            reponseDistance = 0.25f * SENSING_RANGE;
                        }

                        if( physicsVehicle.getPhysicsLocation().distance(closestTerrainPoint) < reponseDistance ) {
                            //activeVelocity = activeVelocity.setY(0.4f * activeVelocity.length());
                            //activeVelocity = activeVelocity.setY(activeVelocity.getY() + 0.00001f);
                            //rampUp();
                            //terrainCollAvoidTime = System.currentTimeMillis();
                            
                            //System.out.println("Came here!");
                            accelerateUpwards = true;
                        }


                        //state = State.Turn;
                        //break;
                    } else if(System.currentTimeMillis() - terrainCollAvoidTime > MAX_TERR_COLL_AVOID_TIME) {
                        //activeVelocity = activeVelocity.setY(0.0f);
                        //rampDown();
                    }
                }
                
                if(
                    (enabled(SimulationMode.SEARCH_MODE) || 
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) || 
                        
                    (enabled(SimulationMode.TASK_ALLOC_MODE) || 
                    (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION)))
                  ) {
                  
                    //System.out.println("Ultimate search targets size: "+ultimateSearchTargets.size());
                    
                    // Collisions with one of the ultimate targets
                    CollisionResults ultimateTargetColRes = new CollisionResults();

                    for(Spatial s: ultimateSearchTargets){
                        
                        //System.out.println("Spatial name: " + s.getName());
                        s.collideWith(ray, ultimateTargetColRes);
                    }

                    if(ultimateTargetColRes.size() > 0){
                        
                        //System.out.println("Collision results size" + ultimateTargetColRes.size());

                        CollisionResult hit = ultimateTargetColRes.getClosestCollision();
                        closestTargetName = hit.getGeometry().getParent().getParent().getName();
                        Vector3f closestTarget = hit.getContactPoint();


                        Iterator colls = ultimateTargetColRes.iterator();
                        CollisionResult coll;
                        String tName;
                        while( colls.hasNext() ) {
                            coll = (CollisionResult)colls.next();
                            targ = coll.getGeometry();

                            //System.out.println("T_NAME: "+targ.getParent().getParent().getName());
                            tName = targ.getParent().getParent().getName();
                            //tName = particle.getParent().getName();

                             if( tName.substring(0, 7).equals("barrel_") || tName.equals("ShipWreck") ) {  

                                switch(i) {
                                    case 0:
                                        if(!northTargNames.contains(tName)) {
                                            northTargNames.add(tName);
                                            northTargetsCount++;
    //                                        System.out.println("Target count for "+spatial.getName()+"'s NORTH sensor is: "+northTargetsCount);
                                        }
                                        break;
                                    case 1:
                                        if(!southTargNames.contains(tName)) {
                                            southTargNames.add(tName);
                                            southTargetsCount++;
    //                                        System.out.println("Target count for "+spatial.getName()+"'s SOUTH sensor is: "+southTargetsCount);
                                        }
                                        break;
                                    case 2:
                                        if(!eastTargNames.contains(tName)) {
                                            eastTargNames.add(tName);
                                            eastTargetsCount++;
    //                                        System.out.println("Target count for "+spatial.getName()+"'s EAST sensor is: "+eastTargetsCount);
                                        }
                                        break;
                                    case 3:
                                        if(!westTargNames.contains(tName)) {
                                            westTargNames.add(tName);
                                            westTargetsCount++;
    //                                        System.out.println("Target count for "+spatial.getName()+"'s WEST sensor is: "+westTargetsCount);
                                        }
                                        break;
                                    case 4:
                                        if(!bottomTargNames.contains(tName)) {
                                            bottomTargNames.add(tName);
                                            bottomTargetsCount++;
    //                                        System.out.println("Target count for "+spatial.getName()+"'s BOTTOM sensor is: "bottomTargetsCount);
                                        }
                                        break;
                                }
                             }
                        }

                        //System.out.println("Closest target name: "+closestTargetName);
                        
                        // Set targetFound = true if closest collision < sensing range and target
                        // has not been previously processed
                        if( physicsVehicle.getPhysicsLocation().distance(closestTarget) < SENSING_RANGE &&
                                !processedTargetsList.contains(closestTargetName) ) {

                            //activeVelocity = new Vector3f(0,0,0);
                            
                            // Order is important here (be careful)
                            if(targetEverEncountered) {
                                firstTargetEncounter = false;
                            }

                            if( enabled(SimulationMode.TASK_ALLOC_MODE) ) {
                                if( !temporarilySkippedTargets.contains(closestTargetName) ) {
                                    targetFound = true;
                                    sim.notifyTargetFound();
                                } else {
                                    targetFound = false;
                                }
                            } else {
                                targetFound = true;
                                sim.notifyTargetFound();
                            }

                            if( (enabled(SimulationMode.TASK_ALLOC_MODE) || enabled(SimulationMode.INTEGRATION_MODE)) 
                                    && DIFFERENTIATE_PROCESSING_TIMES ) {
                                closestIsPreferred = sim.getTargetType(closestTargetName).equals(preferredTargetType);
                            }
                            
                            if(!targetEverEncountered) {
                               targetEverEncountered = true;
                            }
                             
                        } else {
                            targetFound = false;
                        }

                        break;
                    }
                    
                }
                
                if(SENS_DEBUG){
                    Geometry line = makeDebugLine(ray, SENSING_RANGE, lineColor);
                    sightLines[i][j++] = line;
                    ((Node)getSpatial().getParent()).attachChild(line);
                }
            }            
        }
        
        if(
            (enabled(SimulationMode.SEARCH_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH)))
          ) {
            iterPartCount = northParticlesCount + southParticlesCount + eastParticlesCount + westParticlesCount;
            partCountHist.offer(iterPartCount);
    //        if(iterPartCount > 0) {
    //            System.out.println("Total particle count for " + spatial.getName() + " is: " + iterPartCount);
    //        }
        }
        
        if(
            (enabled(SimulationMode.INIT_SELF_ORG_MODE) || 
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))) ||
                
            (enabled(SimulationMode.SEARCH_MODE) || 
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))) || 
                
            (enabled(SimulationMode.TASK_ALLOC_MODE) ||
            (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION)))
          ) {
            iterVehCount = northVehiclesCount + southVehiclesCount + eastVehiclesCount + westVehiclesCount;
            //System.out.println("Iteration vehicles: "+iterVehCount);
            
            if( iterVehCount > 0 ) {
                timeOfLastNeighEncounter = sim.getCurrentTick();
            }
            
            if( iterVehCount < MIN_ACCEPT_NUM_NEIGH && !dropBelowAcceptNeighCaptured ) {
                timeNeighDropBelowThres = sim.getCurrentTick();
                dropBelowAcceptNeighCaptured = true;
                AcceptNumNeighReached = false;
                dropBelowAcceptNeighEnded = false;
            }
            
            if( iterVehCount >= MIN_ACCEPT_NUM_NEIGH && dropBelowAcceptNeighCaptured ) {
                AcceptNumNeighReached = true;
                dropBelowAcceptNeighEnded = true;
                dropBelowAcceptNeighCaptured = false;
            }
        }    
         
        if(
            enabled(SimulationMode.TASK_ALLOC_MODE) ||
           (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TASK_ALLOCATION))
          ) {
            
            if( timeOfFirstTargEncounter == 0 ) {
                timeOfFirstTargEncounter = sim.getCurrentTick();
            }
            
            iterTargCount = northTargetsCount + southTargetsCount + eastTargetsCount + westTargetsCount;
            //System.out.println("Iteration targets: "+iterTargCount);
            
            if( targetSweepingModeActive ){
                if( sweepedTargets == null ) {
                    sweepedTargets = new ArrayList<String>();
                    perSweepTargs = new HashMap<Integer, ArrayList<String>>();
                }
                
                String[] iterTargs = getSensedTargetsList();
                
                for( String trgt : iterTargs ) {
                    
                    // Overall sweeped targets
                    if( !sweepedTargets.contains(trgt) ) {
                        sweepedTargets.add(trgt);
                    }
                    
                    // Only, current sweep's targets
                    currSweepTargs = perSweepTargs.get(sweepNumber);
                    if( currSweepTargs == null ) {
                        currSweepTargs = new ArrayList<String>();
                        currSweepTargs.add(trgt);
                        perSweepTargs.put(sweepNumber, currSweepTargs);
                    } else if( !perSweepTargs.get(sweepNumber).contains(trgt) ) {
                        perSweepTargs.get(sweepNumber).add(trgt);
                    }
                }  
            }
            
        }
        
        return targetFound;
        
    }
    
    public Vector3f getAvgNeighDirection() {

        // Get vehicles in sensing range
        String[] sensedVehicles = getSensedVehiclesList();
        
        // Holds current neighbor's estimated velocity
        Vector3f estNeighborVel;

        // Will hold the alignment direction
        Vector3f alignDir = new Vector3f(0, 0, 0);

        if(sensedVehicles != null && sensedVehicles.length > 0) {
            
            // Build alignment direction vector
            for(String neigh : sensedVehicles) {
                estNeighborVel = simpleEstNeighborVel(neigh);
                alignDir = alignDir.add(estNeighborVel);
            }
            alignDir = alignDir.mult(1/sensedVehicles.length);  
           
            // Find the average alognment direction of neighbors
            return alignDir;
        }
        
        return null;                
        
    }
    
    public int getSensedTargetsCount() {
        return iterTargCount;
    }
    
    public int getSensedNeighCount() {
        return iterVehCount;
    }
    
    private void trackVehVehCollision(float sensedDistance) {
        if( sensedDistance < 0.2 * SENSING_RANGE ) { // is it a collision in the first place
            if( elapsedVehVehCollTestTime < VEH_VEH_COLLISION_TEST_INTERVAL ) {
                numCollisionsInPeriod++;
                if( DEBUG_VEH_VEH_COLLISION ) {
                    System.out.println("Vehicle Collision Detected. Number of collisions so far: "+numCollisionsInPeriod);
                }
            } else {
                elapsedVehVehCollTestTime = 0.0f;
                numCollisionsInPeriod = 0;
            }
        } else if( DEBUG_VEH_VEH_COLLISION ) {
            //System.out.println("Distance: "+sensedDistance);
        }
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
    
    // Returns path length
    
    public float reorient(Vector3f newOrientationDir) {
        
        if( !USE_TETHERED_RW ) {
            if( newOrientationDir.length() < 3 ) {
                prevOrientMode = getActiveOrientMode();
                enableReorientationMode(OrientMode.OPT_REORIENT_ALG);
            } else {
                enableReorientationMode(prevOrientMode);
            }
        }

        enhanReorAlgTookEffect = false;
        float pathLength = 0;
        
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
                return 0; // nothing needs to be done (length of target orientation vector is 0)
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
        
        if( DEBUG_REORIENTATION_ALGS ) {
            System.out.println("Axis: "+axis);
            System.out.println("Quadrant: "+quadrant);
        }
        
        // Make the re-orientation turn based on the quadrant/axis the orientation
        // vector lies in/on
        if(axis.equals("+veActVelAxis")) {
            return 0; // no turn needed; just keep going straight
        } else if(axis.equals("-veActVelAxis")) {   // re-orient towards backward direction
            
            if( OPTIMIZED_REORIENTATION_ALG ) {
                
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", FastMath.PI/3);  // 60 deg
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", 5*FastMath.PI/3);  // 300 deg
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", FastMath.PI/3);  // 60 deg

            } else if( ADVANCED_REORIENTATION_ALG ) {
                
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", FastMath.PI);
                
                pathLength = FastMath.PI*activeTurnRadius;
            } else {
                
                compositeTurnSeq.add("travel", 2 * activeTurnRadius);
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", 1.5f * FastMath.PI);
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", 0.5f * FastMath.PI);

                pathLength = 2*(1 + FastMath.PI)*activeTurnRadius;
            }

        } else if(axis.equals("+vePerpenAxis")) {   // re-orient towards vehicle's east direction
            
            if(ORIG_REORIENTATION_ALG) {
                
                compositeTurnSeq.add("travel", activeTurnRadius);
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", 1.5f * FastMath.PI);
                compositeTurnSeq.add("travel", activeTurnRadius);
                
                pathLength = (2 + 1.5f*FastMath.PI)*activeTurnRadius;
                
            } else if(ENHANCED_REORIENTATION_ALG || ADVANCED_REORIENTATION_ALG) {
                
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", FastMath.HALF_PI);
                enhanReorAlgTookEffect = true;
                
                pathLength = FastMath.HALF_PI*activeTurnRadius;
                
            } else if( OPTIMIZED_REORIENTATION_ALG ) {
                
                float theta = FastMath.PI/6;    // 30 deg
                float alpha = FastMath.PI/3;    // 60 deg
                float b = 2 * FastMath.sqrt(2) * activeTurnRadius * FastMath.sin(theta/2);
                
                compositeTurnSeq.add("travel", b);
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", FastMath.TWO_PI - alpha);
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", theta); 
            }
            
        } else if(axis.equals("-vePerpenAxis")) {   // re-orient towards vehicle's west direction
            
            if(ORIG_REORIENTATION_ALG) {
                compositeTurnSeq.add("travel", activeTurnRadius);
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", 1.5f * FastMath.PI);
                compositeTurnSeq.add("travel", activeTurnRadius); 
                
                pathLength = (2 + 1.5f*FastMath.PI)*activeTurnRadius;
            } else if(ENHANCED_REORIENTATION_ALG || ADVANCED_REORIENTATION_ALG) {
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", FastMath.HALF_PI);
                enhanReorAlgTookEffect = true;
                
                pathLength = FastMath.HALF_PI*activeTurnRadius;
                
            } else if( OPTIMIZED_REORIENTATION_ALG ) {
                
                float theta = FastMath.PI/6;    // 30 deg
                float alpha = FastMath.PI/3;    // 60 deg
                float b = 2 * FastMath.sqrt(2) * activeTurnRadius * FastMath.sin(theta/2);
                
                compositeTurnSeq.add("travel", b);
                compositeTurnSeq.add("turn_direction", "left");
                compositeTurnSeq.add("turn_angle", FastMath.TWO_PI - alpha);
                compositeTurnSeq.add("turn_direction", "right");
                compositeTurnSeq.add("turn_angle", theta); 
            }

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
            
            if(ORIG_REORIENTATION_ALG) {
            
                // Calculate the distance the vehicle needs to travel before turning
                float delta =  activeTurnRadius * ( 1 - (FastMath.cos(theta) / (1 + FastMath.sin(theta))));

                //System.out.println("Theta: "+(theta*FastMath.RAD_TO_DEG));
                //System.out.println("Delta: "+delta);

                compositeTurnSeq.add("turn_direction", firstDir);
                compositeTurnSeq.add("turn_angle", FastMath.HALF_PI);
                compositeTurnSeq.add("travel", delta);
                compositeTurnSeq.add("turn_direction", firstDir);
                compositeTurnSeq.add("turn_angle", FastMath.HALF_PI);
                compositeTurnSeq.add("travel", activeTurnRadius);
                compositeTurnSeq.add("turn_direction", firstDir);
                compositeTurnSeq.add("turn_angle", FastMath.PI);
                compositeTurnSeq.add("turn_direction", secDir);
                compositeTurnSeq.add("turn_angle", theta);
                compositeTurnSeq.add("travel", activeTurnRadius - delta);
            
                pathLength = 2*(1 + FastMath.PI + (theta/2))*activeTurnRadius;
                
            } else if(ENHANCED_REORIENTATION_ALG || ADVANCED_REORIENTATION_ALG) {
                compositeTurnSeq.add("turn_direction", secDir);
                compositeTurnSeq.add("turn_angle", theta);
                enhanReorAlgTookEffect = true;
                
                pathLength = theta*activeTurnRadius;
                
            } else if( OPTIMIZED_REORIENTATION_ALG ) {
                
                float lambda = FastMath.HALF_PI - normDirVec.angleBetween(activeVelocity.normalize());
                float beta = normDirVec.angleBetween(activeVelocity.normalize());
                float alpha = FastMath.acos(0.5f*(1-FastMath.sin(-lambda)));
                theta = beta - alpha;
                float b = 2 * FastMath.sqrt(2) * activeTurnRadius * FastMath.sin(theta/2);
                
                compositeTurnSeq.add("travel", b);
                compositeTurnSeq.add("turn_direction", firstDir);
                compositeTurnSeq.add("turn_angle", FastMath.TWO_PI - alpha);
                compositeTurnSeq.add("turn_direction", secDir);
                compositeTurnSeq.add("turn_angle", theta); 
                
                if( DEBUG_OPT_REORIENT_ALG ) { 
                    System.out.println("Angle of new vector from vehile direction: "+normDirVec.angleBetween(activeVelocity.normalize())+" ("+normDirVec.angleBetween(activeVelocity.normalize())*FastMath.RAD_TO_DEG+")");
                    System.out.println("b: "+b);
                    System.out.println("Theta: "+theta+" ("+theta*FastMath.RAD_TO_DEG+")");
                    System.out.println("Lambda: "+lambda+" ("+lambda*FastMath.RAD_TO_DEG+")");
                    System.out.println("Alpha: "+alpha+" ("+alpha*FastMath.RAD_TO_DEG+")");
                    System.out.println("Beta: "+beta+" ("+beta*FastMath.RAD_TO_DEG+")");
                }
            }
            
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
            float delta = activeTurnRadius * ( 1 - (FastMath.cos(theta) / (1 + FastMath.sin(theta))));
            
            //System.out.println("Theta: "+(theta*FastMath.RAD_TO_DEG));
            //System.out.println("Delta: "+delta);
            if(ORIG_REORIENTATION_ALG || ENHANCED_REORIENTATION_ALG) {
                compositeTurnSeq.add("travel", activeTurnRadius + delta);
                compositeTurnSeq.add("turn_direction", firstDir);
                compositeTurnSeq.add("turn_angle", 1.5f * FastMath.PI);
                compositeTurnSeq.add("turn_direction", secDir);
                compositeTurnSeq.add("turn_angle", theta);       
                compositeTurnSeq.add("travel", activeTurnRadius - delta);  

                pathLength = 2*(1 + (3*FastMath.QUARTER_PI) + (theta/2))*activeTurnRadius;
            } else if(ADVANCED_REORIENTATION_ALG) {
                compositeTurnSeq.add("turn_direction", secDir);
                compositeTurnSeq.add("turn_angle", (FastMath.HALF_PI + theta));
                pathLength = (FastMath.HALF_PI + theta)*activeTurnRadius;
                
            } else if( OPTIMIZED_REORIENTATION_ALG ) {
                
                float lambda = normDirVec.angleBetween(perpenVecToUse);
                float beta = lambda + FastMath.HALF_PI;
                float alpha = FastMath.acos(0.5f*(1-FastMath.sin(lambda)));
                
                theta = beta - alpha;
                float b = 2 * FastMath.sqrt(2) * activeTurnRadius * FastMath.sin(theta/2);
                
                compositeTurnSeq.add("travel", b);
                compositeTurnSeq.add("turn_direction", firstDir);
                compositeTurnSeq.add("turn_angle", FastMath.TWO_PI - alpha);
                compositeTurnSeq.add("turn_direction", secDir);
                compositeTurnSeq.add("turn_angle", theta); 
                
                if( DEBUG_OPT_REORIENT_ALG ) { 
                    System.out.println("Angle of new vector from vehile direction: "+normDirVec.angleBetween(activeVelocity.normalize())+" ("+normDirVec.angleBetween(activeVelocity.normalize())*FastMath.RAD_TO_DEG+")");
                    System.out.println("b: "+b);
                    System.out.println("Theta: "+theta+" ("+theta*FastMath.RAD_TO_DEG+")");
                    System.out.println("Lambda: "+lambda+" ("+lambda*FastMath.RAD_TO_DEG+")");
                    System.out.println("Alpha: "+alpha+" ("+alpha*FastMath.RAD_TO_DEG+")");
                    System.out.println("Beta: "+beta+" ("+beta*FastMath.RAD_TO_DEG+")");
                }
            }
            
        }  
        
        return pathLength;
    }
    
    public void generateRandomTurn() {
        
        // First, clear all active turn flags and variables
        resetCompositeTurnFlags();
        
        // Set speed to at-task speed
        updateVehicleSpeed(AT_TASK_VEH_SPEED);
        
        // Generate a random angle between 0 and MAX_RAND_WALK_TURN_ANGLE
        float turnAng =  FastMath.nextRandomFloat() * 
                (Math.random() > 0.99 ? RARE_TURN_ANGLE : MAX_RAND_WALK_TURN_ANGLE);
        
        // Pick a turn direction randomly
        String turnDir = (Math.random() > 0.5 ? "right" : "left");
        
        float dist = FastMath.nextRandomFloat() * MAX_RAND_TRAVEL_DIST;
        
        // Create a composite turn sequence that only contains the just generated turn angle
        compositeTurnSeq.add("turn_direction", turnDir);
        compositeTurnSeq.add("turn_angle", turnAng); 
        compositeTurnSeq.add("travel", dist); 
        // Activate the turn sequence
        compositeTurnInProgress = true;
        
    }
    
    public void generateLimitingRandomTurn() {
        
        // First, clear all active turn flags and variables
        resetCompositeTurnFlags();
        
        // Set speed to at-task speed
        updateVehicleSpeed(AT_TASK_VEH_SPEED);
        
        // Generate a random angle between Pi/2 and 3Pi/2
        float turnAng, frequentAngle, rareAngle = FastMath.QUARTER_PI;
              
        frequentAngle = FastMath.nextRandomFloat() * 1.5f * FastMath.PI;
        if( frequentAngle < FastMath.HALF_PI ) frequentAngle += FastMath.HALF_PI;
        
        turnAng = FastMath.nextRandomFloat() * 
                (Math.random() > 0.99 ? rareAngle : frequentAngle); 
        
        // Pick a turn direction randomly
        String turnDir = (Math.random() > 0.5 ? "right" : "left");
        
        // Create a composite turn sequence that only contains the just generated turn angle
        compositeTurnSeq.add("turn_direction", turnDir);
        compositeTurnSeq.add("turn_angle", turnAng); 
        
        //  Added to limit spread
        compositeTurnSeq.add("turn_direction", turnDir);
        compositeTurnSeq.add("turn_angle", FastMath.HALF_PI); 
        
        // Predefined distance
        compositeTurnSeq.add("travel", 0.5f); 
        
        // Activate the turn sequence
        compositeTurnInProgress = true;
    }

    public void genRetractedSeqRandTurn() {
        
        // A target has been found before filling the buffer with 3 
        // unsuccessful target-finding sequences: no need to keep track of 
        // these sequences; clear the buffer.
        if( seqRetractionBuffer.size() < 3 && targetFound ) {
            if(DEBUG_SEQ_RETRACTION) System.out.println("Clearing!!");
            seqRetractionBuffer.clear();
        }
        
        // If the buffer has been filled with 3 unsuccessful target-finding 
        // sequences: retract these sequences and return
        if( seqRetractionBuffer.size() == 3 ) {
            
            if(DEBUG_SEQ_RETRACTION) System.out.println("Retracting sequence ...");

            CompositeTurnSequence retractionTurnSeq = new CompositeTurnSequence();
            CompositeTurnSequence currentSequence;
            Pair<String, Object> currentPair, replacementPair;
            float nextAngle = 0;
            
            // Reverse direction
            retractionTurnSeq.add("turn_direction", "left");
            retractionTurnSeq.add("turn_angle", FastMath.PI/3);  // 60 deg
            retractionTurnSeq.add("turn_direction", "right");
            retractionTurnSeq.add("turn_angle", 5*FastMath.PI/3);  // 300 deg
            retractionTurnSeq.add("turn_direction", "left");
            retractionTurnSeq.add("turn_angle", FastMath.PI/3);  // 60 deg
            
            if(DEBUG_SEQ_RETRACTION) System.out.println("========================================");
            if(DEBUG_SEQ_RETRACTION) System.out.println("Retracked sequence (turn): Backwards");
            
            while( !seqRetractionBuffer.isEmpty() ) {
                
                //if(DEBUG_SEQ_RETRACTION) System.out.println("My size is :"+seqRetractionBuffer.size());
                
                currentSequence = seqRetractionBuffer.pop();
                
                // Reverse the sequence
                for(int i = currentSequence.size() - 1; i >= 0 ; i--) {
                    currentPair = currentSequence.get(i);
                    
                    //System.out.println("Poped sequence: ("+currentPair.key+", "+currentPair.val+")");
                    
                    if( (currentPair.key).equals("travel") ) {
                        retractionTurnSeq.add(currentPair);
                        if(DEBUG_SEQ_RETRACTION) System.out.println("Retracked sequence (travel): " + currentPair.val);
                    } else if( (currentPair.key).equals("turn_angle") ) { // order is important here
                        nextAngle = (Float)currentPair.val;
                    } else if( (currentPair.key).equals("turn_direction") ) {                           
                        replacementPair = new Pair("turn_direction", ( ((String)currentPair.val).equals("left") ? "right" : "left" ));
                        retractionTurnSeq.add(replacementPair);
                        retractionTurnSeq.add("turn_angle", nextAngle);
                        if(DEBUG_SEQ_RETRACTION) System.out.println("Retracked sequence (turn): " + replacementPair.val);
                    }
                }
            }
            
            if(DEBUG_SEQ_RETRACTION) System.out.println("========================================");
            
            // First, clear all active turn flags and variables
            resetCompositeTurnFlags();

            compositeTurnSeq = retractionTurnSeq;

            // Activate the turn sequence
            compositeTurnInProgress = true;
            
            return;
        } 
        
        // First, clear all active turn flags and variables
        resetCompositeTurnFlags();
        
        // Set speed to at-task speed
        updateVehicleSpeed(AT_TASK_VEH_SPEED);
        
        // Pick a turn direction randomly
        String turnDir = (Math.random() > 0.5 ? "right" : "left");
        
        if(DEBUG_SEQ_RETRACTION) System.out.println("Original sequence (turn): " + turnDir);
       
        float escapeAngle = FastMath.nextRandomFloat() * FastMath.PI;

        // Add the just generated turn angle to the sequence followed by a small
        // distance to travel
        compositeTurnSeq.add("turn_direction", turnDir);
        compositeTurnSeq.add("turn_angle", escapeAngle); 
        //compositeTurnSeq.add("travel", 0.5f); 
        compositeTurnSeq.add("travel", FastMath.nextRandomFloat() * 10); 

        if(DEBUG_SEQ_RETRACTION) System.out.println("Original sequence (turn): " + turnDir);
        if(DEBUG_SEQ_RETRACTION) System.out.println("Original sequence (travel): " + 0.5f);

        if( !startPosStored ) {
            // This is a lazy approach used only for time restrictions (implement full mechanism in the future)
            // Store sequence start position
            seqStartPos = physicsVehicle.getPhysicsLocation();
            startPosStored = true;
        }
        
        seqRetractionBuffer.push((CompositeTurnSequence)compositeTurnSeq.clone());
        
        // Activate the turn sequence
        compositeTurnInProgress = true;
    }    
    
    public void genTetheredRandTurn() {
        
        // A target has been found before filling the buffer with 3 
        // unsuccessful target-finding sequences: no need to keep track of 
        // these sequences; clear the buffer.
        if( seqRetractionBuffer.size() < 3 && targetFound ) {
            if(DEBUG_TETHERED_RW) System.out.println("Clearing!!");
            seqRetractionBuffer.clear();
        }
        
        // If the buffer has been filled with 3 unsuccessful target-finding 
        // sequences: retract these sequences and return
        if( seqRetractionBuffer.size() == 3 ) {
            
            countSeqExecsWithoutTarg++;
            
            if(DEBUG_TETHERED_RW) System.out.println("Count of sequences without a target: " + countSeqExecsWithoutTarg);

            seqRetractionBuffer.clear();
            
            // Find current position (at the end of sequence)
            seqEndPos = physicsVehicle.getPhysicsLocation();
            
            Vector3f homeVector = seqStartPos.subtract(seqEndPos);
            
            if(DEBUG_TETHERED_RW) DebugUtils.plotArrow(seqEndPos, homeVector, TETHER_COLOR, 2f, sim);
            
            if( !orientModeSaved ) {

                // Get and store active reorientation mode
                savedOrientMode = getActiveOrientMode();
                orientModeSaved = true;

                // Switch orientation mode to the optimized orientation mode
                enableReorientationMode(OrientMode.OPT_REORIENT_ALG);

            }

            // Build reorientation sequence
            reorient(homeVector);
           
            // Travel straight ...
            compositeTurnSeq.add("travel", homeVector.length());

            // Activate the turn sequence
            compositeTurnInProgress = true;
            
            return;
        } 
        
        // First, clear all active turn flags and variables
        resetCompositeTurnFlags();
        
        // Set speed to at-task speed
        updateVehicleSpeed(AT_TASK_VEH_SPEED);
        
        // Pick a turn direction randomly
        String turnDir = (Math.random() > 0.5 ? "right" : "left");
        
        //if(DEBUG_TETHERED_RW) System.out.println("Original sequence (turn): " + turnDir);
       
        float escapeAngle = FastMath.nextRandomFloat() * FastMath.PI;

        // Exp. response threshold function to increase likelihood of choosing 
        // larger distances when target not found for long time
        float expThresFunction = 1 - FastMath.exp(-countSeqExecsWithoutTarg/MAX_SEQUENCES_WITHOUT_TARG_THRESH);

        float maxDist = 10;
        
        // As the number of sequences executed without finding a target increases
        // over time, the likelihood of selecting a large jump increases
        if( FastMath.nextRandomFloat() < expThresFunction ) {
            maxDist = 40;
        }
        
        if(DEBUG_TETHERED_RW) System.out.println("Max. distance: " + maxDist);
        float distToUse = FastMath.nextRandomFloat() * maxDist;
        if(DEBUG_TETHERED_RW) System.out.println("Used distance: " + distToUse);
        
        // Add the just generated turn angle to the sequence followed by a small
        // distance to travel
        compositeTurnSeq.add("turn_direction", turnDir);
        compositeTurnSeq.add("turn_angle", escapeAngle); 
        //compositeTurnSeq.add("travel", 0.5f); 
        compositeTurnSeq.add("travel", distToUse); 

        //if(DEBUG_TETHERED_RW) System.out.println("Original sequence (turn): " + turnDir);
        //if(DEBUG_TETHERED_RW) System.out.println("Original sequence (travel): " + 0.5f);

        if( !startPosStored ) {
            // Store sequence start position
            seqStartPos = physicsVehicle.getPhysicsLocation();
            startPosStored = true;
        }
        
        seqRetractionBuffer.push((CompositeTurnSequence)compositeTurnSeq.clone());
        
        // Activate the turn sequence
        compositeTurnInProgress = true;
    }
    
    private void genBubbleChainRandWalk() {
        
        // First, clear all active turn flags and variables
        resetCompositeTurnFlags();
        
        // Set speed to at-task speed
        updateVehicleSpeed(AT_TASK_VEH_SPEED);
        
        // Pick a turning direction randomly
        String turnDir = (Math.random() > 0.5 ? "right" : "left");
        String oppTurnDir = (turnDir.equals("right") ? "left" : "right");
        
        // Pick a limited (capped by 3 units) random distance to travel
        float travelDist = FastMath.rand.nextFloat() * 10;
        
        // With low probability, generate long distance to try escaping 
        // being stuck in-place if there are not targets nearby
        if( !limitBubbleChainJumps && FastMath.nextRandomFloat() > 0.85f ) {
            int[] distances = new int[]{50, 15, 30, 45, 20, 56};
            travelDist = distances[FastMath.rand.nextInt(6)];
        }
        
        // With a 50% probability, continue straight, i.e. don't reorient
        if( FastMath.nextRandomFloat() > 0.5 ) {
        
            // Pick a random turn angle
            float turnAng = FastMath.rand.nextFloat() * FastMath.TWO_PI;

            // Build direction of travel based on selected angle and vehicle's 
            // heading direction
            Quaternion turnQuatern = new Quaternion();
            Vector3f vehHeadingDir = activeVelocity.normalize();        
            Vector3f vehPitchAxis =  Vector3f.UNIT_Y.cross(vehHeadingDir);
            Vector3f vehVertAxis = vehHeadingDir.cross(vehPitchAxis);
            turnQuatern.fromAngleAxis(turnAng, vehVertAxis);    
            Vector3f travelDir = turnQuatern.mult(vehHeadingDir);

            // ReorientVehicle, in-place, towards that new direction (using the
            // optimized orientation algorithm

            if( !orientModeSaved ) {

                // Get and store active reorientation mode
                savedOrientMode = getActiveOrientMode();
                orientModeSaved = true;

                // Switch orientation mode to the optimized orientation mode
                enableReorientationMode(OrientMode.OPT_REORIENT_ALG);

            }
            
            // Build reorientation sequence
            reorient(travelDir);
        
        }
        
        // Travel straight ...
        compositeTurnSeq.add("travel", travelDist); 
        
        // with a small probability (<= 0.1), don't return (to try escaping being
        // stuck in-place if there are not targets nearby)
        if( FastMath.nextRandomFloat() < 0.9f ) {
            // 2) Reverse direction ...
            compositeTurnSeq.add("turn_direction", turnDir);
            compositeTurnSeq.add("turn_angle", FastMath.PI/3);  // 60 deg
            compositeTurnSeq.add("turn_direction", oppTurnDir);
            compositeTurnSeq.add("turn_angle", 5*FastMath.PI/3);  // 300 deg
            compositeTurnSeq.add("turn_direction", turnDir);
            compositeTurnSeq.add("turn_angle", FastMath.PI/3);  // 60 deg

            // 3) Travel same distance ...
            compositeTurnSeq.add("travel", travelDist); 
        }
        
        // Activate the turn sequence
        compositeTurnInProgress = true;
    }
    
    public void generateAtTargetRandTurn() {
        
        // First, clear all active turn flags and variables
        resetCompositeTurnFlags();
        
        // Generate a random angle between 0 and 2Pi
        float turnAng =  FastMath.nextRandomFloat() * FastMath.TWO_PI;
        
        // Pick a turn direction randomly
        String turnDir = (Math.random() > 0.5 ? "right" : "left");
        
        // Create a composite turn sequence that only contains the just generated turn angle
        compositeTurnSeq.add("turn_direction", turnDir);
        compositeTurnSeq.add("turn_angle", turnAng); 
        
        // Activate the turn sequence
        compositeTurnInProgress = true;
        
    }
    
    public void turn(String turnDirection, float targetTurnAngle, float tpf) {
        
        turnStepAngle = MAX_TURN_ANGLE;
        chordLen = activeChordLen;
        
        if(totalTurnAngleSoFar + turnStepAngle > targetTurnAngle) {
            turnStepAngle -= (totalTurnAngleSoFar + turnStepAngle - targetTurnAngle);
            //chordLen = 2 * activeTurnRadius * FastMath.cos(turnStepAngle/2);  // ORIGINAL
            chordLen = 2 * activeTurnRadius * FastMath.sin(turnStepAngle);      // NEW
            //chordLen = activeTurnRadius * turnStepAngle;
            turnCompleted = true;
        }
        
        totalTurnAngleSoFar += turnStepAngle;
        
        if(!turnDirection.equals("left")) {
            turnStepAngle = - turnStepAngle;
        }
        
        turnQuat = new Quaternion();  
        turnQuat.lookAt(activeVelocity, Vector3f.UNIT_Y);
        turnQuat.fromAngleAxis(turnStepAngle, Vector3f.UNIT_Y);

        activeVelocity = turnQuat.mult(activeVelocity);
        turnIncStartLoc = physicsVehicle.getPhysicsLocation();

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
    
    public Vector3f simpleEstNeighborPos(String neighborName) {
        
        // Get actual neighbor location 
        Vector3f actNeighborLoc = sim.getUWVehiclesMap().get(neighborName).getPhysicsVehicle().getPhysicsLocation();
        
        // Add some error
        float errorX = (float) Math.random() * NEIGH_MAX_POS_EST_ERROR * actNeighborLoc.getX();     // If actual is 30, estimated is 30-36
        errorX = (Math.random() > 0.5 ? errorX : -errorX);                       // add it or subtract it: 24-36
        
        float errorY = (float) Math.random() * NEIGH_MAX_POS_EST_ERROR * actNeighborLoc.getY();
        errorY = (Math.random() > 0.5 ? errorY : -errorY);
        
        float errorZ = (float) Math.random() * NEIGH_MAX_POS_EST_ERROR * actNeighborLoc.getZ();
        errorZ = (Math.random() > 0.5 ? errorZ : -errorZ);
        
        float xComp = actNeighborLoc.x + errorX;
        float yComp = actNeighborLoc.y + errorY;
        float zComp = actNeighborLoc.z + errorZ;
        
        Vector3f estNeighLoc = new Vector3f(xComp, yComp, zComp);
        
        //System.out.println("Actual neighbor loc.: "+actNeighborLoc);
        //System.out.println("Estimated neighbor loc.: "+estNeighLoc);
        
        return estNeighLoc;
        
    }
    
    public Vector3f simpleEstNeighborVel(String neighborName) {
        
        Vector3f nieghborVel = sim.getUWVehiclesMap().get(neighborName).getPhysicsVehicle().getLinearVelocity();
        
        float velMag = nieghborVel.length();
        
        float errorX = (float) Math.random() * NEIGH_MAX_VEL_EST_ERROR * velMag;     // If actual is 30, estimated is 30-36
        errorX = (Math.random() > 0.5 ? errorX : -errorX);        // add it or subtract it: 24-36
        
        float errorY = (float) Math.random() * NEIGH_MAX_VEL_EST_ERROR * velMag;
        errorY = (Math.random() > 0.5 ? errorY : -errorY);
        
        float errorZ = (float) Math.random() * NEIGH_MAX_VEL_EST_ERROR * velMag;
        errorZ = (Math.random() > 0.5 ? errorZ : -errorZ);
        
        float xComp = nieghborVel.x + errorX;
        float yComp = nieghborVel.y + errorY;
        float zComp = nieghborVel.z + errorZ;
        
        Vector3f estimatedVel = new Vector3f(xComp, yComp, zComp);
        
        //System.out.println("Actual neighbor vel.: "+nieghborVel);
        //System.out.println("Estimated neighbor vel.: "+estimatedVel);
        
        return estimatedVel;
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
        
        System.out.println("Actual neighbor vel.: "+nieghborVel);
        System.out.println("Estimated neighbor vel.: "+estimatedVel);
        
        return estimatedVel;
    }
    
    public Vector3f simpleEstTargetPos(String targetName) {
        
        // Get actual target location 
        Vector3f actTargetLoc = sim.getTargsControlsMap().get(targetName).getPhysicsLocation();
        
        // Add some error
        float errorX = (float) Math.random() * NEIGH_MAX_POS_EST_ERROR * actTargetLoc.getX();     // If actual is 30, estimated is 30-36
        errorX = (Math.random() > 0.5 ? errorX : -errorX);                       // add it or subtract it: 24-36
        
        float errorY = (float) Math.random() * NEIGH_MAX_POS_EST_ERROR * actTargetLoc.getY();
        errorY = (Math.random() > 0.5 ? errorY : -errorY);
        
        float errorZ = (float) Math.random() * NEIGH_MAX_POS_EST_ERROR * actTargetLoc.getZ();
        errorZ = (Math.random() > 0.5 ? errorZ : -errorZ);
        
        float xComp = actTargetLoc.x + errorX;
        float yComp = actTargetLoc.y + errorY;
        float zComp = actTargetLoc.z + errorZ;
        
        Vector3f estTargLoc = new Vector3f(xComp, yComp, zComp);
        
        //System.out.println("Actual target loc.: "+actTargetLoc);
        //System.out.println("Estimated target loc.: "+estTargLoc);
        
        return estTargLoc;
        
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
        if(FastMath.abs(perpenDist) < activeTurnRadius) {
            System.out.println("Original turn radius: "+perpenDist);
            turnRadius = FastMath.sign(perpenDist) * activeTurnRadius;
        } else {
            turnRadius = perpenDist;
        }
        
        return turnRadius;
    }
    
    public void follow(float turnRadius, Vector3f neighborEstVel, float tpf) {
        
        float followAngle = activeVelocity.normalize().angleBetween(neighborEstVel.normalize());
        
        //Quaternion alignmentQuat = new Quaternion();
        //alignmentQuat.fromAngleAxis(turnAngle, activeVelocity.normalize());
        
        if(followAngle < FastMath.PI) {
            this.turn("right", followAngle, tpf);
        } else {
            this.turn("left", followAngle, tpf);
        }
        
    }
    
    public void chaseGroup() {
        // This is wrong correct it!!!!
        
        //@TODO: complete implementation, to be called in the update loop based on brain decisions.
        if( flockAlgorithm == null ) {
           flockAlgorithm = new ModifiedReynolds(this, sim); 
        }
        
        chasingStartTime = sim.getCurrentTick();
        
        flockAlgorithm.applyUpdateRules(); // make sure this is correct here
        
    }
    
    private void changeVelocityTo(Vector3f neighborEstVel, float tpf) {
//        estimateTurnRadius(float neighborDistEst)
//        follow(activeTurnRadius, Vector3f neighborEstVel, float tpf);
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
        
        //for(float angleX = -commFieldAngle; angleX < commFieldAngle; angleX += FastMath.QUARTER_PI * 0.1) {
          
        float angleWithActVel, angleAroundActVel;
            
        for(int r = 0; r < COMM_RESOLUTION; r++) {
            
            rayOrigin = spatial.getWorldTranslation().add(activeVelocity.normalize().mult(0.01f));
            rayDirection.set(activeVelocity.normalize());
            lineColor = COMM_RANGE_COLOR;
        
            if(COMM_DEBUG && commLines[i] != null){
                //System.out.println("In detach: "+(Node)getSpatial().getParent());
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
            
//            rayDirection.set(activeVelocity.normalize().mult(commRange));
//            aimDirection.fromAngleAxis(angleX, Vector3f.UNIT_Y);
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
                    
                    if( vehID.substring(0, 2).equals("V_") &&
                            !neighborList.contains(vehID) && 
                            !vehID.equals(spatial.getName()) &&
                            !inNeighborIgnoreList(vehID)) {
//                        System.out.println("Taken: "+vehID);
                        neighborList.add(vehID);
                    } 
//                    else {
//                        System.out.println("Excluded: "+vehID);
//                    }
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
        //return targetFound;        
    }
    
    private void buildDynamicNetwork(String vehID, List<String> neighborList, HashMap<String, Float> neighborDistances) {
        
        try {
            // Build an UWSN out of the vehicle and its neighbors

            int nodeCount, linkCount, zoneCount;
            linkCount = 2 * neighborList.size();
            nodeCount = zoneCount = (linkCount/2) + 1;
//            linkCount = neighborList.size();
//            nodeCount = zoneCount = linkCount + 1;
            
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
            if(SEND_REC_DEBUG_ENABLED) {
                System.out.println("Re-initialized communication ...");
            }
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
            if(SEND_REC_DEBUG_ENABLED) {
                System.out.println("Broadcasting to neighbors ...");
            }
            //System.out.println("Neighbors: "+neighborList.toString());
            sim.getCommMedium().conveyMessage(message);
            sentMsgsCount++;
            return true;
        } else {
            if(SEND_REC_DEBUG_ENABLED) {
                System.out.println("Neighbor list is empty ...");
            }
            return false;
        }

//            for(Integer neigh : neighborNodes) {
//               System.out.println("Broadcasted Message: " + sim.getCommMedium().getMessage(centerNodeID, neigh).getData("text")); 
//            }
    }
    
    private List<Message> listenToNeighbors() {
     
        if(!commInitiated) {
            if(SEND_REC_DEBUG_ENABLED) {
                System.out.println("Re-initialized communication ...");
            }
           initCommunication(); 
        }
        
        commState = CommState.receive;
        
        ArrayList<Message> neighborMessages = new ArrayList<Message>();
        SimpleMessage msg;
        
        // Listen to neighbors
        if( !neighborList.isEmpty() ) {
            if(SEND_REC_DEBUG_ENABLED) {
                System.out.println("Listening to neighbors ...");
            }
            for(String neigh : neighborList) {
                while((msg = (SimpleMessage)sim.getCommMedium().getMessage(neigh, VEHICLE_NAME)) != null) {
                    if(SEND_REC_DEBUG_ENABLED) {
                        System.out.println(VEHICLE_NAME+" received a message from "+neigh+". Content: "+ msg.getData("SELF_BEST_POINTER"));
                    }
                    neighborMessages.add(msg);
                    receivedMsgsCount++;
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
        //int vehInd;
        //vehInd = vehicles.indexOf(vehicle.getModel());
        //System.out.println("Index: "+vehInd);
        //vehicles.remove(vehInd);
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
        //return (float)( Math.random() * (Math.random() < 0.5 ? -MAX_SPEED : MAX_SPEED) );
    }

    private void setInitLocation(ContainerType containerType, Vector3f containerLocation) {
        
        // New way
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
        
        // Old way
        //physicsVehicle.setPhysicsLocation(new Vector3f(randPos(), 30, randPos()));
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
            //vehicle.setVelX(-Math.abs(vehicle.getVel().x));
            //vehicle.setVel(Vector3f.ZERO);
            this.setActiveVelocity(Vector3f.ZERO);
            //VEHICLE_PATH_DEBUG = false;
        }
        if( physicsVehicle.getPhysicsLocation().y > MAX_PT.y ) {
            boundedLoc.setY(MAX_PT.y);
            //vehicle.setVelY(-Math.abs(vehicle.getVel().y));
            //vehicle.setVel(Vector3f.ZERO);
            //this.setActiveVelocity(Vector3f.ZERO);
            //VEHICLE_PATH_DEBUG = false;
        }
        if( physicsVehicle.getPhysicsLocation().z > MAX_PT.z ) {
            boundedLoc.setZ(MAX_PT.z);
            //vehicle.setVel(Vector3f.ZERO);
            this.setActiveVelocity(Vector3f.ZERO);
            //VEHICLE_PATH_DEBUG = false;
            //vehicle.setVelZ(-Math.abs(vehicle.getVel().z));
        }

        if( physicsVehicle.getPhysicsLocation().x < MIN_PT.x ) {
            boundedLoc.setX(MIN_PT.x);
            //vehicle.setVel(Vector3f.ZERO);
            this.setActiveVelocity(Vector3f.ZERO);
            //VEHICLE_PATH_DEBUG = false;
            //vehicle.setVelX(Math.abs(vehicle.getVel().x));
        }
        if( physicsVehicle.getPhysicsLocation().y < MIN_PT.y ) {
            boundedLoc.setY(MIN_PT.y);
            //vehicle.setVel(Vector3f.ZERO);
            //this.setActiveVelocity(Vector3f.ZERO);
            //VEHICLE_PATH_DEBUG = false;
            //vehicle.setVelY(Math.abs(vehicle.getVel().y));
        }
        if( physicsVehicle.getPhysicsLocation().z < MIN_PT.z ) {
            boundedLoc.setZ(MIN_PT.z);
            //vehicle.setVel(Vector3f.ZERO);
            this.setActiveVelocity(Vector3f.ZERO);
            //VEHICLE_PATH_DEBUG = false;
            //vehicle.setVelZ(Math.abs(vehicle.getVel().z));
        }

        physicsVehicle.setPhysicsLocation(boundedLoc);
    }
    
    public boolean vehExceededBoundary() {
       
        if( physicsVehicle.getPhysicsLocation().x >= MAX_PT.x ||
            //physicsVehicle.getPhysicsLocation().y >= MAX_PT.y ||
            physicsVehicle.getPhysicsLocation().z >= MAX_PT.z ||
            physicsVehicle.getPhysicsLocation().x <= MIN_PT.x ||
            physicsVehicle.getPhysicsLocation().y <= -5 ||
            physicsVehicle.getPhysicsLocation().z <= MIN_PT.z ) {
            return true;
        }
        return false;
        
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

    @Override
    public void prePhysicsTick(PhysicsSpace space, float tpf) {

    }

    @Override
    public void physicsTick(PhysicsSpace space, float tpf) {
        physTickCount++;
    }
    
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
    
    // -------------------
    // Search (PSO) API
    // -------------------
    
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

    public Vector3f getNeighborhoodBestPosition() {
        
        ArrayList<Message> neighborMessages = (ArrayList)listenToNeighbors();
        
        if(PSO_DEBUG_ENABLED && neighborMessages.size() > 0) {
            System.out.println("Number of neighbor messages: "+neighborMessages.size());
        }

        Integer bestNeighborSol = 0, sol;
        Vector3f bestNeighborSolPointer = null;
        
        for(Message msg : neighborMessages) {
            //System.out.println("Processing neighbor message ...");
            sol = (Integer)msg.getData("SELF_BEST_SOLUTION");
            if(PSO_DEBUG_ENABLED) {
                System.out.println("Neighbor solution: "+sol);
                System.out.println("Best Neighbor solution: "+bestNeighborSol);
            }
            if( sol > bestNeighborSol ) {
                bestNeighborSol = sol;
                bestNeighborSolPointer = (Vector3f)msg.getData("SELF_BEST_POINTER");
            }
        }
        
        return bestNeighborSolPointer;
    }
    
    public void updateVehicleVelocity(Vector3f newVelocity) {
        //System.out.println("Entered position update ...");
        
        searchUpdateVelocity = newVelocity;

        //System.out.println("Active velocity: "+activeVelocity);
        //System.out.println("New velocity: "+searchUpdateVelocity);
        
        //System.out.println("Distance between velocities: "+searchUpdateVelocity.distance(activeVelocity));
        
        //@TODO: you may need to remove this check
        if( searchUpdateVelocity.distance(activeVelocity) < TURN_TRIGGER_THRESHOLD) {
            if(TURN_DEBUG_ENABLED) {
                System.out.println("Active velocity: "+activeVelocity);
                System.out.println("No turn needed ...");
            }
            turnNotNeeded = true;
        } else {
            
            //DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), searchUpdateVelocity, ColorRGBA.Magenta, 2f, sim);
        
            if(elapsedGracePeriodTicks >= GRACE_PERIOD) {
                resetGracePeriodVars();
            } else {
                elapsedGracePeriodTicks++;
                //System.out.println("Incremented: "+elapsedGracePeriodTicks);
            }
            
            if(!compositeTurnInProgress && !inGracePeriod) {
                
                if(TURN_DEBUG_ENABLED) {
                    System.out.println("Entered turning ...");
                }
                
                if(DEBUG_TURN_DIR_ENABLED) {
                    DebugUtils.plotArrow(physicsVehicle.getPhysicsLocation(), searchUpdateVelocity, TURN_DIRECTION_ARROW_COLOR, 2, sim);
                }

                // Re-orient vehicle using the current (constant) vehicle velocity
                float actVelMagnitude = activeVelocity.length();
                
                // Check if the magnitude of update velocity is less than the vehicle'e 
                // sensing range. Only update velocity if it is greater than sensing
                // range.
                //if(travelDist > SENSING_RANGE) {
                // Disabled because it doesn't make sense to ignore an update pointing
                // in a completely different direction, but has a magnitude less than
                // the sensing range.
                    
                    if(
                        enabled(SimulationMode.SEARCH_MODE) ||
                       (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.TARGET_SEARCH))
                      ) {
                        
                        // Turning speed is always the vehicle's minimum speed
                        //activeVelocity = activeVelocity.normalize().mult(MIN_VEH_SPEED);
                        
                        // Changed ... reasoning: only if the active turn radius is equal to 
                        // the minimum turn radius, should the speed be minimum, otherwise,
                        // it can exceed the min speed as there is no need to slow down. Slowing
                        // down is only needed at the MTR to minimize traveld dist during turning.
                        if(activeTurnRadius == MIN_TURN_RADIUS) {
                            activeVelocity = activeVelocity.normalize().mult(MIN_VEH_SPEED);
                        }
                        
                        // Change the vehicle's orientation
                        reorientPathLength = reorient(searchUpdateVelocity.normalize().mult(actVelMagnitude));
                
                        if(USE_RPSO_SEARCH) {
                            compositeTurnSeq.add("travel", searchUpdateVelocity.length());
                        }
                        
                        if(USE_RPSO_SEARCH && LEARNING_ENABLED && LEARNING_PHASE_ACTIVE) {
                            
                            // Discretize vehicle's current state
                            float minDist = Float.MAX_VALUE, theMin;
                            int minDistInd = -1;
                            for( int s = 0; s < directionStates.length; s++ ) {
                               theMin = Math.min(minDist, directionStates[s].distance(activeVelocity.normalize()));
                               if(theMin < minDist) {
                                  minDistInd = s; 
                                  minDist = theMin;
                               }
                            }
                            currentStateIndex = minDistInd;
                            
                            // Discretize action to be taken as a result of PSO update
                            minDist = Float.MAX_VALUE;
                            minDistInd = -1;
                            for( int s = 0; s < directionStates.length; s++ ) {
                               theMin = Math.min(minDist, directionStates[s].distance(searchUpdateVelocity.normalize()));
                               if(theMin < minDist) {
                                  minDistInd = s; 
                                  minDist = theMin;
                               }
                            }
                            currentActionIndex = minDistInd;
                        }
                        
                        
                    } else if(
                        enabled(SimulationMode.INIT_SELF_ORG_MODE) ||
                       (enabled(SimulationMode.INTEGRATION_MODE) && activeMissionStage.equals(MissionStage.INITIAL_SELF_ORGANIZATION))
                             ) {
                        // Change the vehicle's orientation
                        reorientPathLength = reorient(searchUpdateVelocity);
                    }
                    
                    // Block turning updates and hand control to the state machine. When
                    // it finishes the turn, it will notify the search algorithm to provide
                    // turn updates.
                    compositeTurnInProgress = true;
                    
                //}

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

    // -------------------
    // Search (CSF) API
    // -------------------
    
    public boolean broadcastTargetFound() {
        
        if( targetFound ) {
            Message msg = new SimpleMessage();
            msg.addData("TARGET_FOUND", spatial.getName());
            boolean msgBroadcasted = (boolean)broadcast(msg);
            if(CSF_DEBUG_ENABLED && msgBroadcasted) {
                System.out.println(spatial.getName()+" broadcasted a message ...");
            }
            return msgBroadcasted;  
        } else {
            return false;
        }
        
    }
    
    public boolean checkNeighForFoundTarget() {
        
        //System.out.println(spatial.getName()+" is listening to neighbors ...");
        
        ArrayList<Message> neighborMessages = (ArrayList)listenToNeighbors();
        
        if(CSF_DEBUG_ENABLED && neighborMessages.size() > 0) {
            System.out.println("Number of neighbor messages: "+neighborMessages.size());
        }

        String neighName = null;
        
        for(Message msg : neighborMessages) {
            if(CSF_DEBUG_ENABLED) {
                System.out.println("Processing neighbor message ...");
            }
            neighName = (String)msg.getData("TARGET_FOUND");
            this.addNeighborToIgnoreList(neighName);
            if(CSF_DEBUG_ENABLED) {
                System.out.println("Neighbor name "+neighName+" found the target!");
            }
        }
        
        return (neighName != null);
    }
    
    public void setTargetFound(boolean flag) {
        targetFound = flag;
        //targetEverEncountered = flag;
    }
    
    public boolean targetEverEncountered() {
        return targetEverEncountered;
    }
    
    public boolean isFirstTargetEncounter() {
        return firstTargetEncounter;
    }
    
    // -------------------
    // Task Allocation API
    // -------------------
    public String[] getSensedVehiclesList() {    // ToDo: check why if any is null we return null?!
        
        if(northVehNames  == null ||
           southVehNames  == null ||
           eastVehNames   == null ||
           westVehNames   == null || 
           bottomVehNames == null) {
            return new String[]{};
        }
        
        Set<String> allVehiclesSet = new HashSet<String>();

        allVehiclesSet.addAll(northVehNames);
        allVehiclesSet.addAll(southVehNames);
        allVehiclesSet.addAll(eastVehNames);
        allVehiclesSet.addAll(westVehNames);
        allVehiclesSet.addAll(bottomVehNames);
        
        String[] allVehList = new String[allVehiclesSet.size()];
        
        return allVehiclesSet.toArray(allVehList);
    }
    
    public ArrayList<String> getSensedNorthVehiclesList() {
        if(northVehNames  == null) return new ArrayList<String>(0);
        return northVehNames;
    }
    
    public ArrayList<String> getSensedSouthVehiclesList() {
        if(southVehNames  == null) return new ArrayList<String>(0);
        return southVehNames;
    }
    
    public ArrayList<String> getSensedEastVehiclesList() {
        if(eastVehNames  == null) return new ArrayList<String>(0);
        return eastVehNames;
    }
    
    public ArrayList<String> getSensedWestVehiclesList() {
        if(westVehNames  == null) return new ArrayList<String>(0);
        return westVehNames;
    }
    
    public String[] getSensedStaticVehiclesList() {
        if(staticVehNames  == null) {
            return new String[]{};
        }
        
        Set<String> staticVehiclesSet = new HashSet<String>();
        staticVehiclesSet.addAll(staticVehNames);
        
        String[] staticVehList = new String[staticVehiclesSet.size()];
        
        return staticVehiclesSet.toArray(staticVehList);
    }
    
    public String[] getSensedMovingVehiclesList() {
        if(movingVehNames  == null) {
            return new String[]{};
        }
        
        Set<String> movingVehiclesSet = new HashSet<String>();
        movingVehiclesSet.addAll(movingVehNames);
        
        String[] movingVehList = new String[movingVehiclesSet.size()];
        
        return movingVehiclesSet.toArray(movingVehList);
    }
    
    public String[] getSensedTargetsList() {
        
        if(northTargNames  == null ||
           southTargNames  == null ||
           eastTargNames   == null ||
           westTargNames   == null || 
           bottomTargNames == null) {
            return new String[]{};
        }
        
        Set<String> allSensedTargsSet = new HashSet<String>();

        allSensedTargsSet.addAll(northTargNames);
        allSensedTargsSet.addAll(southTargNames);
        allSensedTargsSet.addAll(eastTargNames);
        allSensedTargsSet.addAll(westTargNames);
        allSensedTargsSet.addAll(bottomTargNames);
        
        // This AUV previously processed targets;
        // ignore previously processed targets 
        if(processedTargetsList.size() > 0) {
            
            ArrayList<String> sensedTargetsList = new ArrayList<String>(allSensedTargsSet);
            int targIndex;

            for (String processedTarget : processedTargetsList) {
                if (sensedTargetsList.contains(processedTarget)) {
                    targIndex = sensedTargetsList.indexOf(processedTarget);
                    sensedTargetsList.remove(targIndex);
                }
            }
            targsToConsiderArr = new String[sensedTargetsList.size()];
            targsToConsiderArr = sensedTargetsList.toArray(targsToConsiderArr);
            
        } else { // This AUV has not processed any targets yet;
                 // consider all sensed targets
            targsToConsiderArr = new String[allSensedTargsSet.size()];
            targsToConsiderArr = allSensedTargsSet.toArray(targsToConsiderArr);
        }

        return targsToConsiderArr;
    }
    
    public boolean neighborsAround() {
        String[] sensedVehicles = getSensedVehiclesList();
        return ( sensedVehicles != null && sensedVehicles.length > 0 );
    }
    
    public void updateVehicleBehavior(TaskAction action) {
        switch(action) {
            case ExploitTarget:
                exploitTarget = true;
                moveOnToNextTarget = false;
                exploitingTarget = false;
                break;
            case LookForAnother:
                exploitTarget = false;
                moveOnToNextTarget = true;
                exploitingTarget = false;
        }
    }
    
    public void setTypeOfTargToProcess(TargetType type) {
        typeOfTargToProcess = type;
    }
    
    public void unsetTypeOfTargToProcess() {
        typeOfTargToProcess = null;
    }
    
    // -------------------
    // Learning API
    // -------------------
    
    public int getNextTurningState() {
        
        return currentActionIndex;
    }
    
    public int getCurrentTurningState() {
        
        return currentStateIndex;
    }

    public int getCurrentTurningAction() {
        
        return currentActionIndex;
    }
    
    public float getTurningReward() {
        return rewardBucket;
    }
    
    // -------------------
    // Turning API - START
    // -------------------
    public void execCompositeTurn(ArrayList<Pair<String, Object>> compositeTurnSeq) {
        
        Iterator seqIter = compositeTurnSeq.iterator();
        Pair<String, Object> sequence;
        
        while(seqIter.hasNext()) {
            sequence = (Pair<String, Object>)seqIter.next();
            this.compositeTurnSeq.add(sequence);
        }
        
    }
    
    // -------------------
    // Turning API - END
    // -------------------
    
    // -------------------
    // VTS API - START
    // -------------------
    public Vector3f getEstimatedInitPos() {
        return estimatedInitPos;
    }
    
    // -------------------
    // VTS API - END
    // -------------------
    
    // -------------------
    // Turning API - END
    // -------------------
    
    // ---------------------
    // Resources API - START
    // ---------------------
    public float getResidualEnergy() {
        return residualEnergy;
    }
    
    public float getInitBatteryEnergy() {
        return INITIAL_BATTERY_ENERGY;
    }
    
    // ---------------------
    // Resources API - END
    // ---------------------
    
    // ---------------------
    // Brain API - START
    // ---------------------
    
    // ---------------------
    // Brain API - END
    // ---------------------
    
    // =========================================================================
    // Vehicle Control API - END
    // =========================================================================    
    
    public Vector3f measureMagNorthDir() {

        float errorX = (Math.random() > 0.5f ? 1 : -1) * FastMath.nextRandomFloat() * MAX_MAG_NORTH_EST_ERROR;
        float errorZ = (Math.random() > 0.5f ? 1 : -1) * FastMath.nextRandomFloat() * MAX_MAG_NORTH_EST_ERROR;
       
        float upwardsVel = (APPLY_UPWARDS_VELOCITY ? DEFAULT_UPWARDS_VELOCITY : 0);

        Vector3f magNorth = new Vector3f(errorX, upwardsVel, 1 + errorZ);

        return magNorth.normalize();
    }
    
    public boolean turnNeeded() {
        return !turnNotNeeded;
    }
    
    public void solicitTurnNeeded() {
        turnNotNeeded = false;
    }
    
    public void setActiveVelocity(Vector3f newVelocity) {
        activeVelocity = newVelocity;
    }
    
    public Vector3f getActiveVelocity() {
        return activeVelocity;
    }
    
    public float getMinVehicleSpeed() {
        return MIN_VEH_SPEED;
    }
    
    public float getVehicleSpeed() {
        return VEHICLE_SPEED;
    }
    
    public float getMaxVehicleSpeed() {
        return MAX_VEH_SPEED;
    }
    
    public float getActiveTurnRadius() {
        return activeTurnRadius;
    }
    
    public boolean compositeTurnInProgress() {
        return compositeTurnInProgress;
    }
    
    public void announceCompTurnInProgress() {
        compositeTurnInProgress = true;
    }
    
    public float getReorientPathLength() {
        return reorientPathLength;
    }
    
    public boolean enhanReorAlgTookEffect() {
        return enhanReorAlgTookEffect;
    }
    
    public void solicitFlockingAlgRuleUpdate() {
        if( flockAlgorithm != null && flockAlgorithm.started() ) {
            flockAlgorithm.applyUpdateRules();
        } else {
            System.out.println("Flocking algorithm not set or needs to be started first!");
        }
    }
    
    public boolean advancedReorientAlgEnabled() {
        return ADVANCED_REORIENTATION_ALG;
    }
    
    public boolean enhancedReorientAlgEnabled() {
        return ENHANCED_REORIENTATION_ALG;
    }
    
    // Flag setters and getters
    public static void toggleVehiclePathDebug() {
        VEHICLE_PATH_DEBUG = !VEHICLE_PATH_DEBUG;
    }
    public static boolean getVehiclePathDebug() {
        return VEHICLE_PATH_DEBUG;
    }
    
    public static SearchAlgorithms[] getAvailableSearchAlgorithms() {
        return SearchAlgorithms.values();
    }
    
    public static TaskAllocAlgorithms[] getAvailableTaskAllocAlgorithms() {
        return TaskAllocAlgorithms.values();
    }
    
    public static IntegrationAlgorithms[] getAvailableIntegAlgorithms() {
        return IntegrationAlgorithms.values();
    }
    
    public static void setSearchAlgorithm(int index) {
        switch(index) {
            case 0: USE_RPSO_SEARCH     = true; break;
            case 1: USE_CSF_SEARCH      = true; break;
            case 2: USE_VTS_SEARCH      = true; break;
            case 3: USE_RDPSO_SEARCH    = true; break;
            case 4: USE_BFM_SEARCH      = true; break;
            case 5: USE_BDFSO_SEARCH    = true; break;
            case 6: USE_DHCP_SEARCH     = true; break;
            case 7: USE_SDHCP_SEARCH    = true; break;
            case 8: USE_SSDHCP_SEARCH   = true; break;
            case 9: USE_SRW_SEARCH      = true; break;    
            case 10: USE_SSW_SEARCH     = true; break;
            case 11: USE_FSRW_SEARCH    = true; break;    

        }
        disableOtherSearchAlgs(index);
    }
    
    public static void setTaskAllocAlgorithm(int index) {
        switch(index) {
            case 0: USE_ERT_TASK_ALLOC      = true; break;
            case 1: USE_PRT_TASK_ALLOC      = true; break;
            case 2: USE_MBB_TASK_ALLOC      = true; break;  
            case 3: USE_BB_TASK_ALLOC       = true; break;
            case 4: USE_HYB_TASK_ALLOC      = true; break;
            case 5: USE_BL_TASK_ALLOC      = true; break;
        }
        //System.out.println("Task Alloc Algorithm set to: "+index);
        disableOtherTaskAllocAlgs(index);
    }
    
    public static void setIntegAlgorithm(int index) {
        switch(index) {
            case 0: USE_BRAINLESS_ALL_STAGE_INTEG = true; break;
            case 1: USE_TWO_STAGE_INTEG           = true; break;
            case 2: USE_ALL_STAGE_INTEG           = true; break;
        }
        //System.out.println("Task Alloc Algorithm set to: "+index);
        disableOtherIntegAlgs(index);
    }

    private static void disableOtherSearchAlgs(int usedAlgIndex) {

        for(SearchAlgorithms searchAlg : SearchAlgorithms.values()) {
            if(searchAlg.index() != usedAlgIndex) {
                switch(searchAlg.index()) {
                    case 0: USE_RPSO_SEARCH     = false; break;
                    case 1: USE_CSF_SEARCH      = false; break;
                    case 2: USE_VTS_SEARCH      = false; break;
                    case 3: USE_RDPSO_SEARCH    = false; break;
                    case 4: USE_BFM_SEARCH      = false; break;
                    case 5: USE_BDFSO_SEARCH    = false; break;
                    case 6: USE_DHCP_SEARCH     = false; break; 
                    case 7: USE_SDHCP_SEARCH    = false; break;     
                    case 8: USE_SSDHCP_SEARCH   = false; break; 
                    case 9: USE_SRW_SEARCH      = false; break;
                    case 10: USE_SSW_SEARCH     = false; break;
                    case 11: USE_FSRW_SEARCH    = false; break;
                        
                }
            }
        }
        
        // Verify and use this instead:
//        USE_RPSO_SEARCH  = ( usedAlgIndex == SearchAlgorithms.RPSO.index() );
//        USE_CSF_SEARCH   = ( usedAlgIndex == SearchAlgorithms.CSF.index()  );
//        USE_VTS_SEARCH   = ( usedAlgIndex == SearchAlgorithms.VTS.index()  );
//        USE_RDPSO_SEARCH = ( usedAlgIndex == SearchAlgorithms.RDPSO.index());
//        USE_BFM_SEARCH   = ( usedAlgIndex == SearchAlgorithms.BFM.index()  );
//        USE_BDFSO_SEARCH = ( usedAlgIndex == SearchAlgorithms.BDFSO.index());
//        USE_DHCP_SEARCH  = ( usedAlgIndex == SearchAlgorithms.DHCP.index() );
//        USE_SDHCP_SEARCH = ( usedAlgIndex == SearchAlgorithms.SDHCP.index() );
//        USE_SSDHCP_SEARCH= ( usedAlgIndex == SearchAlgorithms.SSDHCP.index() );
//        USE_SRW_SEARCH   = ( usedAlgIndex == SearchAlgorithms.SRW.index()  );
//        USE_SSW_SEARCH   = ( usedAlgIndex == SearchAlgorithms.SSW.index()  );
//        USE_FSRW_SEARCH  = ( usedAlgIndex == SearchAlgorithms.FSRW.index() );
    }
    
    private static void disableOtherTaskAllocAlgs(int usedAlgIndex) {

        for(TaskAllocAlgorithms taskAllocAlg : TaskAllocAlgorithms.values()) {
            if(taskAllocAlg.index() != usedAlgIndex) {
                switch(taskAllocAlg.index()) {
                    case 0: USE_ERT_TASK_ALLOC      = false; break;
                    case 1: USE_PRT_TASK_ALLOC      = false; break;
                    case 2: USE_MBB_TASK_ALLOC      = false; break;
                    case 3: USE_BB_TASK_ALLOC       = false; break;
                    case 4: USE_HYB_TASK_ALLOC      = false; break;
                    case 5: USE_BL_TASK_ALLOC      = false; break;
                }
            }
        }
    }
    
    private static void disableOtherIntegAlgs(int usedAlgIndex) {

        for(IntegrationAlgorithms integAlg : IntegrationAlgorithms.values()) {
            if(integAlg.index() != usedAlgIndex) {
                switch(integAlg.index()) {
                    case 0: USE_BRAINLESS_ALL_STAGE_INTEG      = false; break;
                    case 1: USE_TWO_STAGE_INTEG                = false; break;
                    case 2: USE_ALL_STAGE_INTEG                = false; break;
                }
            }
        }
    }
    
    public static SearchAlgorithms getActiveSearchAlgorithm() {
        
        if(USE_RPSO_SEARCH)    return SearchAlgorithms.RPSO;
        if(USE_CSF_SEARCH)     return SearchAlgorithms.CSF;
        if(USE_VTS_SEARCH)     return SearchAlgorithms.VTS;
        if(USE_RDPSO_SEARCH)   return SearchAlgorithms.RDPSO;
        if(USE_BFM_SEARCH)     return SearchAlgorithms.BFM;
        if(USE_BDFSO_SEARCH)   return SearchAlgorithms.BDFSO;
        if(USE_DHCP_SEARCH)    return SearchAlgorithms.DHCP;
        if(USE_SDHCP_SEARCH)    return SearchAlgorithms.SDHCP;
        if(USE_SSDHCP_SEARCH)  return SearchAlgorithms.SSDHCP;
        if(USE_SRW_SEARCH)     return SearchAlgorithms.SRW;   
        if(USE_SSW_SEARCH)     return SearchAlgorithms.SSW;
        if(USE_FSRW_SEARCH)    return SearchAlgorithms.FSRW;
       
//        for(SearchAlgorithms searchAlg : SearchAlgorithms.values()) {
//            switch(searchAlg.index()) {
//                case 0: if(USE_RPSO_SEARCH)  return 0;
//                case 1: if(USE_CSF_SEARCH)   return 1;
//                case 2: if(USE_VTS_SEARCH)   return 2;
//                case 3: if(USE_RDPSO_SEARCH) return 3;
//                case 4: if(USE_BFM_SEARCH)   return 4;
//                case 5: if(USE_BDFSO_SEARCH) return 5;
//                case 6: if(USE_DHCP_SEARCH)  return 6;
//                case 7: if(USE_SDHCP_SEARCH)  return 7;
//                case 8: if(USE_SSDHCP_SEARCH)return 8;
//                case 9: if(USE_SRW_SEARCH)   return 9;   
//                case 10: if(USE_SSW_SEARCH)   return 10;
//                case 11: if(USE_FSRW_SEARCH) return 11;
//            }
//        }
        return SearchAlgorithms.RPSO;
    }
    
    public static TaskAllocAlgorithms getActiveTaskAllocAlgorithm() {
        
        if(USE_ERT_TASK_ALLOC)  return TaskAllocAlgorithms.EXP_RESP_THRESHOLD;
        if(USE_PRT_TASK_ALLOC)  return TaskAllocAlgorithms.POLY_RESP_THRESHOLD;
        if(USE_MBB_TASK_ALLOC)  return TaskAllocAlgorithms.MINI_BRAIN_BASED;
        if(USE_BB_TASK_ALLOC)   return TaskAllocAlgorithms.BEACON_BASED_TA;
        if(USE_HYB_TASK_ALLOC)  return TaskAllocAlgorithms.HYBRID_TA;
        if(USE_BL_TASK_ALLOC)  return TaskAllocAlgorithms.BLIND_TA;
        
        return TaskAllocAlgorithms.EXP_RESP_THRESHOLD;
    }
    
    public static IntegrationAlgorithms getActiveIntegAlgorithm() {
        
        if(USE_TWO_STAGE_INTEG)  return IntegrationAlgorithms.TWO_STAGE_INTEGRATION;
        if(USE_ALL_STAGE_INTEG)  return IntegrationAlgorithms.ALL_STAGE_INTEGRATION;
        if(USE_BRAINLESS_ALL_STAGE_INTEG)  return IntegrationAlgorithms.BRAINLESS_ALL_STAGE_INTEGRATION;
        
        return IntegrationAlgorithms.BRAINLESS_ALL_STAGE_INTEGRATION;
    }
    
    /**
     * Takes a boolean array of flags indicating active modes in the 
     * following order:<br />
     * <ul>
     *  <li>GENERAL_TESTS_MODE</li>
     *  <li>INIT_SELF_ORG_MODE</li>
     *  <li>SEARCH_MODE</li>
     *  <li>TASK_ALLOC_MODE</li>
     *  <li>INTEGRATION_MODE</li>
     * </ul>
     * @param modeFlags
     */
    public void setActiveModes(boolean[] modeFlags) { 
        simulationModes.put(SimulationMode.GENERAL_TESTS_MODE, modeFlags[0]);
        simulationModes.put(SimulationMode.INIT_SELF_ORG_MODE, modeFlags[1]);
        simulationModes.put(SimulationMode.SEARCH_MODE,        modeFlags[2]);
        simulationModes.put(SimulationMode.TASK_ALLOC_MODE,    modeFlags[3]);
        simulationModes.put(SimulationMode.INTEGRATION_MODE,   modeFlags[4]);
    }
    
    public boolean isTargetFound() {
        return targetFound;
    }
    
    public boolean isTargetEverEncountered() {
        return targetEverEncountered;
    }
    
    public void clearNeighborIgnoreList() {
        if(ignoreList != null) {
            ignoreList.clear();
        } else {
            System.out.println("Ignore list not initialized yet!");
        }
    }
    
    public boolean addNeighborToIgnoreList(String neighborName) {
        if(ignoreList != null) {
            if(!ignoreList.contains(neighborName)) {
                ignoreList.add(neighborName);
                return true;
            } else {
                if(SEND_REC_DEBUG_ENABLED) {
                    System.out.println("Neighbor already in ignore list!");
                }
                return false;
            }
        } else {
            System.out.println("Ignore list not initialized yet!");
            return false;
        }
    }
    
    public boolean inNeighborIgnoreList(String neighborName) {
        if(ignoreList != null) {
            if(ignoreList.contains(neighborName)) {
                return true;
            } else {
                if(SEND_REC_DEBUG_ENABLED) {
                    System.out.println("Neighbor not in ignore list!");
                }
                return false;
            }
        } else {
            System.out.println("Ignore list not initialized yet!");
            return false;
        }
    }
    
    public float getSensingRange() {
        return SENSING_RANGE;
    }
    
    public void solicitInPlaceTurn(float radius, String direction) {
        inPlaceTurnActivated = true;
        solicitedTurnRadius = radius;
        solicitedTurnDirection = direction;
    }
    
    public void solicitInPlaceTurn() {
        solicitInPlaceTurn(MIN_TURN_RADIUS, "left");
    }
    
    public void cancelInPlaceTurn() {
        inPlaceTurnActivated = false;
        inPlaceTurnStarted = false;
    }
        
    public static Vector3f getDropOffLocation() { // You should change that in the future
        return CONTAINER_LOCATION;
    }
    
    public int getSentMsgCount() {
        return sentMsgsCount;
    }
    
    public int getReceivedMsgCount() {
        return receivedMsgsCount;
    }
    
    public int getCurrentTick() {
        return sim.getCurrentTick();
    }
    
    public boolean isTimeSet( TriggeringEvent triggeringEvent ) {
        switch(triggeringEvent) {
            case LAST_NEIGHBOR_ENCOUNTER:
                return ( timeOfLastNeighEncounter != 0 );
            case NUM_NEIGH_DROPPED_BELOW_THRES:
                return ( timeNeighDropBelowThres != 0 ); 
            case START_OF_SEQUENCE:
                return ( sequenceStartTime != 0 );
            case START_OF_TRIGGER:
                return ( timeOfTrigger != 0 );
            case START_OF_CHASING:
                return ( chasingStartTime != 0 );
            case START_OF_SEARCH:
                return ( searchStartTime != 0 );
            case START_OF_SELF_ORG:
                return ( selfOrgStartTime != 0 );     
            case START_OF_MISSION:
                return ( missionStartTime != 0 );                   
            case FIRST_TARGET_ENCOUNTER:
                return ( timeOfFirstTargEncounter != 0 );
            default:
                return false;  
        }
    }
    
    public int getStartTimeOf( TriggeringEvent triggeringEvent) {
        switch(triggeringEvent) {
            case LAST_NEIGHBOR_ENCOUNTER:
                return timeOfLastNeighEncounter;
            case NUM_NEIGH_DROPPED_BELOW_THRES:
                return timeNeighDropBelowThres;    
            case START_OF_SEQUENCE:
                return sequenceStartTime;
            case START_OF_TRIGGER:
                return timeOfTrigger;
            case START_OF_CHASING:
                return chasingStartTime;
            case START_OF_SEARCH:
                return searchStartTime;
            case START_OF_SELF_ORG:
                return selfOrgStartTime;
            case START_OF_MISSION:
                return missionStartTime;
            case FIRST_TARGET_ENCOUNTER:
                return timeOfFirstTargEncounter;
            default:
                return -1;  
        }
    }
    
    public void setSearchStartTime(int time) {
        searchStartTime = time;
    }
    
    public int getNumVehVehCollInTestPeriod() {
        return numCollisionsInPeriod;
    }
    
    public ElapsedTime getElapsedMissionTime() {
        
        float elapsedTimeRatio = ((float)(getCurrentTick() - missionStartTime)) / MISSION_TIME_CONSTRAINT;
        
        if( elapsedTimeRatio > 1 ) {
            return ElapsedTime.EXCEEDED_CONSTRAINT;
        } else if( elapsedTimeRatio > 0.8f && elapsedTimeRatio <= 1 ) {
            return ElapsedTime.VERY_LONG;
        } else if( elapsedTimeRatio > 0.6f && elapsedTimeRatio <= 0.8f ) {
            return ElapsedTime.LONG;
        } else if( elapsedTimeRatio > 0.4f && elapsedTimeRatio <= 0.6f ) {
            return ElapsedTime.MEDIUM;
        } else if( elapsedTimeRatio > 0.2f && elapsedTimeRatio <= 0.4f ) {
            return ElapsedTime.SHORT;
        } else {
            return ElapsedTime.VERY_SHORT;
        }
    }
    
    public void updateVehicleSpeed(float newAgentSpeed) {
        setActiveVelocity(activeVelocity.normalize().mult(newAgentSpeed));
        limitVehicleSpeed();
    }
    
    // =========================================================================
    // Vehicle speed timer: used to change vehicle speed for limited time
    // =========================================================================
    public void enableSpeedTimer() {
        speedTimerEnabled = true;
        speedTimerTimedOut = false;
    }
    
    public void disableSpeedTimer() {
        speedTimerEnabled = false;
        speedTimerTimedOut = false;
    }
    
    public void setSpeedTimer(int value) {
        if( speedTimerEnabled ) {
            formerVehicleSpeed = activeVelocity.length();
            speedTimerTimedOut = false;
            speedTimer = value;
        }
    }
    
    public void clearSpeedTimer() {
        if( speedTimerEnabled ) {
            updateVehicleSpeed(formerVehicleSpeed); // restore old speed
            speedTimer = 0;
            speedTimerTimedOut = true;
        }
    }
    
    public boolean speedTimerTimedOut() {
        return speedTimerTimedOut;
    }
    // =========================================================================
    
    /**
     * Saves the state of the active mission stage and global stage's algorithm 
     * to enable later continuation after an incidental behavior 
     */
    public void saveState() {
        // Implement!
        
        stateSavingTime = getCurrentTick();
    }
    
    public int getSavedStateTime() {
        return stateSavingTime;
    }
    
    public void recoverPreviousState() {
        // Implement!
    }
    
    public void recoverOrigMissionStageState() {
        // Implement!
    }
    
    public void enqueueMotionAlgorithm( Pair<MotionAlgorithm, Duration> motionAlgorithmInfo ) {
        try {
            // add to queue for execution starting next tick
            motionAlgQueue.put(motionAlgorithmInfo);
        } catch (InterruptedException ex) {
            Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
        }
    }
    
    public void enqueueCompositeTurnSeq( CompositeTurnSequence sequence ) {
        try {
            // add to queue for execution starting next tick
            compTurnSeqQueue.put(sequence);
        } catch (InterruptedException ex) {
            Logger.getLogger(UWVehicleControl.class.getName()).log(Level.SEVERE, ex.getMessage(), ex);
        }
    }
    
    private void executeMotionAlgorithm( Pair<MotionAlgorithm, Duration> algorithmInfo ) {
        
        // Extract algorithm information
        MotionAlgorithm motionAlgorithm = algorithmInfo.key;
        Duration algDuration = algorithmInfo.val;
        
        // Is this correct here??
        // if new and old motion algorithms are the same: no need to make any
        // changes
        if( motionAlgorithm.equals(previousMotionAlgorithm) ) {
            System.out.println("Same algorithm ... returning");
            return;
        }
                
        System.out.println("motion algorithm: " + motionAlgorithm.name());
        System.out.println("algorithm duration: " + algDuration.name());


        // Make sure the current state (algorithms/sequences, AUV speed, 
        // direction, etc. that are running) has been saved before making any 
        // changes.
        if( getSavedStateTime() != getCurrentTick() ) {
            saveState();
        }
        
        // Should the change be perminent? or until the trigger has been satisfied?!
        // Address this ... timers and state restoration?
        
        // Handle the special case where "motionAlgorithm" is "OF_BEST_REWARDED_ACTION"
        if( motionAlgorithm.equals(MotionAlgorithm.OF_BEST_REWARDED_ACTION) ) {
            motionAlgorithm = brain.getHigherMentalFunc().getHighestRewActForCurrSituation()
                    .getMotorFunction().getParameters().type.getMotionAlgorithm();
        }  
        
        // Handle the special case where "motionAlgorithm" is "RANDOM_AMONG_ALGS"
        MotionAlgorithm[] motionAlgs = MotionAlgorithm.values();
        while( motionAlgorithm.equals(MotionAlgorithm.RANDOM_AMONG_ALGS) ) {
            motionAlgorithm = motionAlgs[FastMath.rand.nextInt(motionAlgs.length)];
        }
        
        switch(algDuration) {
            case UNTIL_INTERRUPTED:
    
            case RANDOM:
            case PRESET:
            case INDEFINITE:
            case TASK_CAP:

            case DUR_OF_SEQUENCE:

            case TIME_TO_SENSE_TARGETS:
            case TIME_TO_REACH_TARGETS:

            case TIME_TO_SENSE_VEHICLES:
            case TIME_TO_REACH_VEHICLES:

            case TIME_TO_ESCAPE_NEIGHBORS:

            case TIME_TO_SATISFY_GOAL:

            case WHILE_TARGETS_BELOW_THRES:
            case WHILE_REWARD_LVL_PERSISTS:

            case OF_BEST_REWARDED_ACTION:
        }
        
        // NOTES:
        // Be careful:
        // - If algorithm's duration is not set, it will run indefinitely
        // - Goal works only after duration is set, otherwise, it is useless
        
        switch(motionAlgorithm) {
            case LOG_SPIRAL:
                // Log spiral search algorithm
                searchAlgorithm = new ConstrainedSpiralFlocking(this, sim);
                searchAlgorithm.setDuration(algDuration);    
                searchAlgorithm.setGoal(MotionGoal.FIND_VEHICLES);
                break;
            case FLOCKING:
                // C. W. Reynolds flocking algorithm
                flockAlgorithm = new ModifiedReynolds(this, sim);
                flockAlgorithm.setDuration(algDuration);
                break;
            case SENSE_TARGET_THEN_RANDOM_WALK:
                // Linger around the targets
                this.motionAlgorithm = new TargetLingering(this, sim);
                searchAlgorithm.setDuration(algDuration);  
                break;
            case TOWARDS_NEIGHBORS_AVERAGE:
                // Move towards the average neighbor position
                this.motionAlgorithm = new NeighborsAverage(this, sim);
                searchAlgorithm.setDuration(algDuration);  
                break;
            case RANDOM_WALK_BIG_JUMPS:
                searchAlgorithm = new SimpleRandomWalk(this, sim);
                searchAlgorithm.setDuration(algDuration);  
                ((SimpleRandomWalk)searchAlgorithm).setJumpLength(SRW_BIG_JUMP_TICKS);
                break;
            case RANDOM_WALK_SMALL_JUMPS:
                searchAlgorithm = new SimpleRandomWalk(this, sim);
                searchAlgorithm.setDuration(algDuration);  
                ((SimpleRandomWalk)searchAlgorithm).setJumpLength(SRW_SMALL_JUMP_TICKS);
                break;
            case SAME_AS_BEFORE:
                // Do nothing!                
                break;
            case PREDEFINED_MISSION_STAGE_ALG:
                if( (this.motionAlgorithm != null || motionSequence != null ) ) {
                    
                    System.out.println("Original mission stage's algorithm was suspended!!");
                    // there is an active motion alg or seq, i.e. original
                    // mission stage's algorithm has been suspended
                    recoverOrigMissionStageState();
                    // Implement ... --^
                } else {
                    System.out.println("Original mission stage's algorithm was NOT previously suspended!!");
                }
        }
        
        
        previousMotionAlgorithm = motionAlgorithm;
        
    }
    
    private void executeCompositeTurnSeq(CompositeTurnSequence sequence) {
        
        // Make sure the current state (algorithms/sequences, AUV speed, 
        // direction, etc. that are running) has been saved before making any 
        // changes.
        if( getSavedStateTime() != getCurrentTick() ) {
            saveState();
        }
        
        // Notify vehicle control of a required turn
        solicitTurnNeeded();
        announceCompTurnInProgress();
        compositeTurnSeq.clear();
        
        seqStartTimeCaptured = false;
        
        execCompositeTurn(sequence);

    }
    
    public void requestFSMStateChange(StateChangeReason reason, MotionAlgorithm motionAlg, Duration duration) {
        fsmStateChangeReason = reason;
        fsmStateChangeRequested = true;
        stateChangeStarted = false;

        enqueueMotionAlgorithm( new Pair(motionAlg, duration) );
        switchToMotionAlgorithm = motionAlg;
    }
    
    public void requestFSMStateChange(StateChangeReason reason, CompositeTurnSequence compTurnSeq) {
        fsmStateChangeReason = reason;
        fsmStateChangeRequested = true;
        stateChangeStarted = false;

        enqueueCompositeTurnSeq( compTurnSeq );
    }
    
    public void notifyFSMStateChangeReqEnded() {
        fsmStateChangeRequested = false;
        stateChangeStarted = false;
        recoverPreviousState();
    }
    
    public int getMissionTimeConstraint() {
        return MISSION_TIME_CONSTRAINT;
    }
    
    public int getMissionStartTime() {
        return missionStartTime;
    }
    
    public boolean isRewardLevelPersistent() {
        return brain.getHigherMentalFunc().isRewardLevelPersistent();
    }
    
    public Duration getDurOfBestRewardedAction() {
        return brain.getHigherMentalFunc().getHighestRewActForCurrSituation()
                .getMotorFunction().getParameters().duration;
    }
    
    public int getElapsedTimeOfActiveMotion() {
        return elapsedTimeOfActiveMotion - getCurrentTick();
    }
    
    public int getPresetDurOfActiveMotion() {
        return presetDurOfActiveMotion;
    }
    
    public void reportMotionAlgStopped(long key) {
        completedMotionAlgKey = key;
    }
    
    public void reportMissionStageCompleted(MissionStage missionStage) {
        missionStageComplStatuses.put(missionStage, true);
    }
    
    public boolean isMissionStageCompleted(MissionStage missionStage) {
        return missionStageComplStatuses.get(missionStage);
    }
    
    public long getCompletedMotionAlgKey() {
        return completedMotionAlgKey;
    }
    
    public long getActiveMotionAlgKey() {
        return activeMotionAlgKey;
    }
    
    public MotionType getActiveMotionType() {
        return activeMotionType;
    }
    
    public void setActiveMotionAlgKey(long key) {
        activeMotionAlgKey = key;
    }
    
    public boolean compTurnSeqMotionEnded() {
        return compTurnSeqMotionEnded;
    }
    
    public void restoreSavedCompTurnSeq() {
        if( !savedSequenceStates.isEmpty() ) {
            
            // Get topmost sequence on the stack and store it the motion sequence 
            // to switch to
            switchToMotionSequence = savedSequenceStates.pop();
            
            // The current sequence, if any, will be pushed automatically onto 
            // the stack the next time the "Turn" state of the FSM is visited if
            // the "motionInterrupted" flag is set.
            motionInterrupted = true;
           
        }
    }
    
    public MiniBrain getBrain() {
        if( BRAIN_MODE_ENABLED ) {
            return brain;
        }
        return null;
    }
    
    public void setActiveMissionStage(MissionStage stage) {
        activeMissionStage = stage;
    }
    
    public void setActiveMissionLvlAction(Enum action) {
        activeMissionLvlAction = action;
    }
    
    public void initActiveMissionAction() {
        
        switch( activeMissionStage ) {
            case INITIAL_SELF_ORGANIZATION:
                switch((InitSelfOrgAlgorithms)activeMissionLvlAction) {
                    case MAG_NORTH_BASED:
                        selfOrgAlgorithm = new InitialSOAlgorithm(this, sim);
                        flockAlgorithm = new ModifiedReynolds(this, sim);
                        break;
                    case NONE:
                        //Implement ...
                }
                break;
            case TARGET_SEARCH:
                switch((SearchAlgorithms)activeMissionLvlAction) {
                    case RPSO:
                        searchAlgorithm = new RPSO(this, sim);
                        if( LEARNING_ENABLED ) {
                            learningAlgorithm = new RLearning(this, sim);
                        }
                        break;
                    case CSF:
                        searchAlgorithm = new ConstrainedSpiralFlocking(this, sim);
                        break;
                    case VTS:
                        searchAlgorithm = new VirtualTetherSearch(this, sim);
                        break;
                    case RDPSO:
                        searchAlgorithm = new RDPSO(this, sim);
                        break;
                    case BFM:
                        searchAlgorithm = new BFM(this, sim);
                        break;
                    //case BDFSO:
                    //    searchAlgorithm = new BDFSO(this, sim);
                    //    break;
                    case DHCP:
                        searchAlgorithm = new DHCP(this, sim);
                        break;
                    case SDHCP:
                        searchAlgorithm = new SDHCP(this, sim);
                        break;
                    case SSDHCP:
                        searchAlgorithm = new SSDHCP(this, sim);
                        break;
                    case SRW:
                        searchAlgorithm = new SimpleRandomWalk(this, sim);
                        break;
                    case SSW:
                        searchAlgorithm = new SimpleSweeping(this, sim);
                        break;
                    case FSRW:
                        searchAlgorithm = new FlockedSRW(this, sim);
                }   
                break;
            case TASK_ALLOCATION:
                switch((TaskAllocAlgorithms)activeMissionLvlAction) {
                    case EXP_RESP_THRESHOLD:
                        taskAllocAlgorithm = new ExpRespThresh(this, sim);
                        break;
                    case POLY_RESP_THRESHOLD:
                        taskAllocAlgorithm = new PolyRespThresh(this, sim);
                        break;
                    case MINI_BRAIN_BASED:
                        //Implement ...
                        break;
                    case BEACON_BASED_TA:
                        taskAllocAlgorithm = new BeaconBasedTaskAlloc(this, sim);
                        break;
                    case HYBRID_TA:
                        taskAllocAlgorithm = new HybridTaskAlloc(this, sim);
                        break;
                    case BLIND_TA:
                        taskAllocAlgorithm = new BlindTaskAlloc(this, sim);
                }
            
                processedTargetsList = new ArrayList<String>();
                UPWARD_ACCELERATION = 0.00001f;
                break;
            case SOURCE_SEARCH:
                switch((SurfacingAlgorithms)activeMissionLvlAction) {
                    case SSRF:
                    default:
                        surfacingAlgorithm = new SimpleSurfacing(this, sim);  
                }  
        }
        
        disableOtherModes(SimulationMode.INTEGRATION_MODE);
        
        activeMissionActInitialized = true;
    }
    
    public void updateActiveMissionAction(float tpf) {
        
        switch( activeMissionStage ) {
            case INITIAL_SELF_ORGANIZATION:
                switch((InitSelfOrgAlgorithms)activeMissionLvlAction) {
                    case MAG_NORTH_BASED:
                        System.out.println("Inside the update function ...");
                        selfOrgAlgorithm.applyUpdateRules();
                        //flockAlgorithm.applyUpdateRules();      // This is probably wrong (will need some more checks)
                        // Updates are solicited from within the SO algorithm
                        break;
                    case NONE:
                        //Implement ...
                }
                break;
            case TARGET_SEARCH:
                searchAlgorithm.applyUpdateRules();
                if( LEARNING_ENABLED ) {
                    learningAlgorithm.applyUpdateRules();       // same here ...
                }   
                break;
            case TASK_ALLOCATION:
                taskAllocAlgorithm.applyUpdateRules(tpf);
                break;
            case SOURCE_SEARCH:
                surfacingAlgorithm.applyUpdateRules();
        }
    }
    
    public void startActiveMissionAction() {
        
        switch( activeMissionStage ) {
            case INITIAL_SELF_ORGANIZATION:
                switch((InitSelfOrgAlgorithms)activeMissionLvlAction) {
                    case MAG_NORTH_BASED:
                        //System.out.println("Inside the start function ...");
                        selfOrgAlgorithm.start();
                        flockAlgorithm.start();      // This is probably wrong (will need some more checks)
                        break;
                    case NONE:
                        //Implement ...
                }
                break;
            case TARGET_SEARCH:
                searchAlgorithm.start();
                if( LEARNING_ENABLED ) {
                    learningAlgorithm.start();       // same here ...
                }   
                break;
            case TASK_ALLOCATION:
                taskAllocAlgorithm.start();
                break;
            case SOURCE_SEARCH:
                surfacingAlgorithm.start();
        }
    }
    
    public void disableOtherModes(SimulationMode mode) {
        
        SimulationMode currMode;
        for( Entry<SimulationMode, Boolean> entry : simulationModes.entrySet() ) {
            currMode = entry.getKey();
            simulationModes.put(currMode, false);
        }
        
        simulationModes.put(mode, true);
    }
    
    private boolean enabled(SimulationMode mode) {
        return simulationModes.get(mode);
    }
    
    public boolean isSimModeEnabled(SimulationMode mode) {
        return enabled(mode);
    }

    public boolean isFirstFSMUpdate() {
        return firstFSMUpdate;
    }
    
    public int getElapsedMissionIntTime() {
        return getCurrentTick() - missionStartTime;
    }
    
    public float getOverallDistTaveled() {
        return overallMissionDistTraveled;
    }
    
    // Fake it approach for detecting task-indicator lights
    public boolean taskIndicatorLightOn(TaskDescriptor descriptor) {
        switch(descriptor) {
            case LARGE_NUM_TARGS:           return largNumTargsDetected;
            case MEDIUM_NUM_TARGS:          return mediumNumTargsDetected;
            case SMALL_NUM_TARGS:           return smallNumTargsDetected;
            case UNIFORM_MIXTURE_OF_TYPES:  return uniformMixTargTypesDetected;
            case MOSTLY_TYPE_I_TARGS:       return dominantType_I_Targs;
            case MOSTLY_TYPE_II_TARGS:      return dominantType_II_Targs;
            case MOSTLY_TYPE_III_TARGS:     return dominantType_III_Targs;
            case SINGLE_TARGET_TASK:        return singleTargetTask;
        }
        return false;
    }
    
    public void turnTaskIndicatorLightsOn(ArrayList<TaskDescriptor> descriptors) {
        
        singleTargetTask = descriptors.contains(TaskDescriptor.SINGLE_TARGET_TASK);
        
        largNumTargsDetected = descriptors.contains(TaskDescriptor.LARGE_NUM_TARGS);
        mediumNumTargsDetected = descriptors.contains(TaskDescriptor.MEDIUM_NUM_TARGS);  
        smallNumTargsDetected = descriptors.contains(TaskDescriptor.SMALL_NUM_TARGS);
        
        if( (largNumTargsDetected & mediumNumTargsDetected & smallNumTargsDetected == true) ||          // all true
            (largNumTargsDetected ^ mediumNumTargsDetected ^ smallNumTargsDetected == false) ) {        // none or two are true
            if(!singleTargetTask){
                System.out.println("Confusing numbers of targets reported to task light indecator controller!!");
            }
        }
        
        uniformMixTargTypesDetected = descriptors.contains(TaskDescriptor.UNIFORM_MIXTURE_OF_TYPES);
        dominantType_I_Targs = descriptors.contains(TaskDescriptor.MOSTLY_TYPE_I_TARGS);
        dominantType_II_Targs = descriptors.contains(TaskDescriptor.MOSTLY_TYPE_II_TARGS);
        dominantType_III_Targs = descriptors.contains(TaskDescriptor.MOSTLY_TYPE_III_TARGS);
    }
    
    public void printTaskIndicatorLightNames() {
        for( TaskDescriptor indicator : TaskDescriptor.values() ) {
            if( taskIndicatorLightOn(indicator) ) {
                System.out.println(indicator.toString() + " is ON");
            }
        }
    }
    
    public static float getMinTurnRadius() {
        return MIN_TURN_RADIUS;
    }
    
    public void setTargetSweepingModeActive(boolean active) {
        targetSweepingModeActive = active;
    }
    
    public ArrayList<String> getSweepedTargList() {
        return sweepedTargets;
    }

    public TargetType getPreferredTargType() {
        return preferredTargetType;
    }
    
    public void setPreferredTargType(TargetType newType) {
        preferredTargetType = newType;
    }
    
    public int getSweepNumber() {
        return sweepNumber;
    }
    
    public void solicitTurnSeqBreak(boolean flag) {
        sequenceBreakSolicited = flag;
    }
    
    public boolean sequenceBreakWasSolicited() {
        return sequenceBreakSolicited;
    }
    
    public void forceCompTurnSeqReset() {
        resetCompositeTurnFlags();
    }
    
    public void solicitStraightTravel(boolean status) {
        straightTravelRequested = status;
    }
    
    public boolean straightTravelRequested() {
        return straightTravelRequested;
    }
    
    private void applyTargSweepRules() {
           
        solicitTurnSeqBreak(false);
              
        int score = 0, minScore = 10;
        
        if( sweepNumber == 8 ) {
            //float avgPerSweepCount = 0;
            for( Entry<Integer, ArrayList<String>> entry : perSweepTargs.entrySet() ) {
                if( entry.getValue().size() >= 2 ) {
                   score += 2; 
                } else if( entry.getValue().size() >= 1 && entry.getValue().size() > 2 ) {
                   ++score; 
                } else {
                   --score;  
                }
                //avgPerSweepCount += entry.getValue().size();
            }
            
//            if( !perSweepTargs.isEmpty() ) {
//                avgPerSweepCount = avgPerSweepCount/perSweepTargs.size();
//            }
            
            //if( avgPerSweepCount < 4 ) {
            if( score <  minScore ) {
                solicitTurnSeqBreak(true);
                sweepNumber = 0;
            }
            
            //System.out.println("Average per-sweep number of targets: " + avgPerSweepCount);
            if(DEBUG_TARGET_SWEEPING) System.out.println("Sweep process score: " + score);
        }
        
    }
    
    public void enableReorientationMode(OrientMode mode) {
        switch(mode) {
            case ORIG_REORIENT_ALG:
                ORIG_REORIENTATION_ALG = true;
                ENHANCED_REORIENTATION_ALG = false;
                ADVANCED_REORIENTATION_ALG = false;
                OPTIMIZED_REORIENTATION_ALG = false;
                break;
            case ENHANCED_REORIENT_ALG:
                ORIG_REORIENTATION_ALG = false;
                ENHANCED_REORIENTATION_ALG = true;
                ADVANCED_REORIENTATION_ALG = false;
                OPTIMIZED_REORIENTATION_ALG = false;
                break;
            case ADVANCED_REORIENT_ALG:
                ORIG_REORIENTATION_ALG = false;
                ENHANCED_REORIENTATION_ALG = false;
                ADVANCED_REORIENTATION_ALG = true;
                OPTIMIZED_REORIENTATION_ALG = false;
                break;
            case OPT_REORIENT_ALG:
                ORIG_REORIENTATION_ALG = false;
                ENHANCED_REORIENTATION_ALG = false;
                ADVANCED_REORIENTATION_ALG = false;
                OPTIMIZED_REORIENTATION_ALG = true;
        }
    }
    
    public OrientMode getActiveOrientMode() {
        if( ORIG_REORIENTATION_ALG )        return OrientMode.ORIG_REORIENT_ALG ;
        if( ENHANCED_REORIENTATION_ALG )    return OrientMode.ENHANCED_REORIENT_ALG;
        if( ADVANCED_REORIENTATION_ALG )    return OrientMode.ADVANCED_REORIENT_ALG;
        if( OPTIMIZED_REORIENTATION_ALG )   return OrientMode.OPT_REORIENT_ALG;
        return OrientMode.ADVANCED_REORIENT_ALG;
    }

    public float getFailureEnergy() {
        return FAILURE_ENERGY;
    }
    
    public ArrayList<String> getProcessedTargetsList() {
        return processedTargetsList;
    }
    
    public int getPresetGoalTargProcessings() {
        return PRESET_NUM_TARG_PROCESSINGS;
    }
    
    public void solicitUpwardsAcceleration(float acceleration) {
        accelerateUpwards = true;
        solicitedUpwardsAccel = acceleration;
    }
    
    public void enableStateTracker() {
        STATE_TRACKER_ENABLED = true;
    }
    
    public OrientMode getSavedOrientMode() {
        return savedOrientMode;
    }
 
    public String getClosestTargetName() {
        return closestTargetName;
    }
    
    public void limitBubbleChainJumps(Boolean status) {
        limitBubbleChainJumps = status;
    }
    
    public void changePathDebugColor(ColorRGBA newColor) {
        DEBUD_PATH_COLOR = newColor;
    }
    
    public void changePathDebugColor(float newWidth) {
        DEBUD_PATH_WIDTH = newWidth;
    }

    public void maneuver(String direction, float angle) {
        
        Quaternion quat = new Quaternion();
        
        Vector3f normActVel = getActiveVelocity().normalize();
        Vector3f pitchAxis = normActVel.cross(Vector3f.UNIT_Y);
        Vector3f yawAxis = pitchAxis.cross(normActVel);
        
        quat.fromAngleAxis(FastMath.DEG_TO_RAD * angle, yawAxis); 
        Vector3f firstOrientDir = quat.mult(normActVel);
        
        quat.fromAngleAxis(FastMath.DEG_TO_RAD * -angle, yawAxis);
        Vector3f secondOrientDir = quat.mult(normActVel);
        
        if( direction.equals("left") ) {
            reorient(firstOrientDir);
            reorient(secondOrientDir);
        } else {
           reorient(secondOrientDir);
           reorient(firstOrientDir); 
        }
        
        compositeTurnInProgress = true;
    }
    
    public float getDynamicVelocityRange() {
        return getMaxVehicleSpeed() / getMinVehicleSpeed();
    }
    
    public float getContainerSideLength() {
        float boxSideLength = (float) java.lang.Math.cbrt(sim.getVehiclesContainerVolume());
        boxSideLength += 0.2 * boxSideLength;
        
        return boxSideLength;
    }
    
    public float getMaxMagneticNorthEstError() {
        return FastMath.atan(MAX_MAG_NORTH_EST_ERROR/MAX_MAG_NORTH_EST_ERROR);  // 45 degrees
    }
    
    public ColorRGBA getPathDebugColor() {
        return DEBUD_PATH_COLOR;
    }
}