package swim.sim;

import com.jme3.app.SimpleApplication;
import com.jme3.audio.AudioNode;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.renderer.RenderManager;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.shape.Box;
import com.jme3.water.WaterFilter;
import info.monitorenter.gui.chart.Chart2D;
import info.monitorenter.gui.chart.ITrace2D;
import info.monitorenter.gui.chart.traces.Trace2DLtd;
import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import swim.algorithm.search.SearchAlgorithms;
import swim.algorithm.search.RDPSO;
import swim.algorithm.search.SearchAlgorithm;
import swim.core.Agent;
import swim.core.CommMedium;
import swim.physics.PhysicsHelper;
import swim.sim.uwswarm.Swarms;
import swim.sim.uwswarm.Swarms.Swarm;
import swim.sim.uwswarm.UWVehicle;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.DebugUtils;

import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;

import com.jme3.niftygui.NiftyJmeDisplay;
import de.lessvoid.nifty.Nifty;
import de.lessvoid.nifty.builder.ScreenBuilder;
import de.lessvoid.nifty.builder.LayerBuilder;
import de.lessvoid.nifty.builder.PanelBuilder;
import de.lessvoid.nifty.controls.button.builder.ButtonBuilder;
import de.lessvoid.nifty.controls.dropdown.builder.DropDownBuilder;
import de.lessvoid.nifty.controls.label.builder.LabelBuilder;
import de.lessvoid.nifty.controls.slider.builder.SliderBuilder;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import swim.algorithm.common.TargetConfig;
import swim.algorithm.integration.IntegrationAlgorithms;
import swim.algorithm.taskalloc.TargetType;
import swim.algorithm.taskalloc.TaskAllocAlgorithms;
import swim.gui.StartScreen;

/**
 * Swim - alpha version
 * @author sherif Tolba
 */
public class Simulator extends SimpleApplication { //implements ActionListener
    
    // =========================================================================
    // Config
    // =========================================================================
    
    // Simulation mode
    private static final boolean IN_BULK_SIM_RUN_MODE = false;
    
    public static final boolean USE_COMPLEX_ENVIRONMENT = true;
    
    private int MAX_SIMULATION_TICK_COUNT = 10000;    // Was 10000 for all runs
    
    private int BULK_SIM_RUN_MODE = 0,      // Target search
            //BULK_SIM_RUN_MODE = 3,      // Integration mode
            BULK_SIM_SEARCH_ALG = 9;    // See below
    
//    0: USE_RPSO_SEARCH
//    1: USE_CSF_SEARCH
//    2: USE_VTS_SEARCH
//    3: USE_RDPSO_SEARCH
//    4: USE_BFM_SEARCH
//    5: USE_BDFSO_SEARCH
//    6: USE_DHCP_SEARCH
//    7: USE_SDHCP_SEARCH
//    8: USE_SSDHCP_SEARCH
//    9: USE_SRW_SEARCH
//    10: USE_SSW_SEARCH
//    11:USE_FSRW_SEARCH
    
    private final float[] TARG_TYPE_PERCENTAGES = new float[]{33, 33, 34};
    
    private boolean SEARCH_MODE_ENABLED = false,
                    TASK_ALLOC_MODE_ENABLED = false,
                    INIT_SELF_ORG_MODE_ENABLED = false,
                    INTEGRATION_MODE_ENABLED = false,
                    GENERAL_TESTS_MODE_ENABLED = false;
    
    private static float TARGET_NEIGHBORHOOD_RADIUS = 25;
    
    private final boolean SHOW_PLOT = false;
    
    // Particles
    private final Vector3f  PARTICLE_ORIGIN = new Vector3f(45, 6, 30);
    private final int       NUM_PARTICLES = 1000;
    // Ship wreck
    private final Vector3f  SHIP_WRECK_LOC = PARTICLE_ORIGIN;
    // Robots
    private static int NUM_AGENTS = 50;

    // Physics
    private final boolean   DEBUG_ENABLED = false; 
    
    private final boolean DEBUG_SHOW_VEHICLE_NAME = true;
    
    // File writing variables
    private final String RESULTS_FOLDER = "results/",
                         AVG_DIST_TO_TARG_FILE_NAME = RESULTS_FOLDER+"avg_dist_to_targ",
                         AVG_SPREAD_FILE_NAME = RESULTS_FOLDER+"avg_spread",
                         MAX_DIST_FROM_SOURCE_FILE_NAME = RESULTS_FOLDER+"max_dist_from_source",
                         MAX_DIST_FROM_TARG_FILE_NAME = RESULTS_FOLDER+"max_dist_from_targ",
                         NUM_REACHED_TARG_FILE_NAME = RESULTS_FOLDER+"num_reached_targ",
                         NUM_LOST_AGENTS_FILE_NAME = RESULTS_FOLDER+"num_lost",
                         NUM_SURV_AGENTS_FILE_NAME = RESULTS_FOLDER+"num_surviving",
                         NUM_SENT_MSGS_FILE_NAME = RESULTS_FOLDER+"num_sent_messages",
                         NUM_RECE_MSGS_FILE_NAME = RESULTS_FOLDER+"num_received_messages",
                         DIST_TRAVELED_FILE_NAME = RESULTS_FOLDER+"overall_dist_taveled",
                         NUM_TARG_PROCESSINGS_FILE_NAME = RESULTS_FOLDER+"num_targ_processings",
                         FILE_ENCODING = "UTF-8";
    
    // Mission-level constants
    private float AUV_VALUE = 15000,            // $15,000 (towards the cheaper end)
                  OPERATIONAL_COST = 10000,     // $10,000/day for the whole swarm
                  TYPE_I_TARGET_PROCESSING_VALUE = 20000,   // $20,000
                  TYPE_II_TARGET_PROCESSING_VALUE = 10000,  // $10,000
                  TYPE_III_TARGET_PROCESSING_VALUE = 15000, // $15,000
                  SUNK_SHIP_VALUE = 100000,                 // $100,000
                  MISSION_TIME_CONSTRAINT = (int)FastMath.ceil(0.95f * MAX_SIMULATION_TICK_COUNT);//15000;  // 15000 simulation ticks (can convert to system time later)
    // =========================================================================
    // Config
    // =========================================================================
    
    // Mission-level variables
    private float actualTimeSpentInMission,
                  missionGain,
                  degreeOfTimeCompliance,
                  percentMissionCompletion,
                  percentRecoveredAgents,
                  dollarGain, failureLoss,
                  targetsActualValue = 0,
                  obtainedMissionValue = 0,
                  missionCost,
                  processed_TypeI_Targets = 0,
                  processed_TypeII_Targets = 0,
                  processed_TypeIII_Targets = 0,
                  typeI_Targets, typeII_Targets, typeIII_Targets,
                  missionStartTick, missionEndTick,
                  numSurvivingAUVs = NUM_AGENTS;
    
            
    private boolean missionCompleted = false, 
                    missionGainPrinted = false,
                    printedMissionCompl = false,
                    missionStatsPrinted = false;
    
    public boolean stopped = false;
    
//    private Spatial particleModel;
//    private RigidBodyControl physicsParticle;
    
    
    // Target types
    private float[] targTypeCounts = new float[3];
    
    // Simulation
    private boolean simulationInitialized = false;
    
    // Physics
    private BulletAppState bulletAppState;
    
    // Water
    private FilterPostProcessor fpp;
    private WaterFilter water;
    private Vector3f lightDir = new Vector3f(-4.9f, -1.3f, 5.9f); // same as light source
    private float initialWaterHeight = 25f;
    private AudioNode waves;
    private boolean advancedWaterEnabled = false;
    
    // Agent
    //private RigidBodyControl physicsAgent;
    //private Quaternion currRotation;
    Vector3f currLocation;
    //ArrayList<UWVehicle> uwVehicles;
    private UWVehicle[] uwVehicles;
    private HashMap<String, Agent> vehiclesMap;
    
    // Ship
    private Spatial shipModel;
    private RigidBodyControl physicsShip;
    
    // Ship wreck
    private Spatial shipWreckModel;
    private RigidBodyControl physicsShipWreck;
    
    // Target
    private Spatial targetModel;
    private RigidBodyControl physicsTarget;
    private HashMap<String, RigidBodyControl> targControlsMap;

    // Communication
    private List<Spatial> vehicles;
    private HashMap<String, UWVehicleControl> vehControlsMap;
    //private HashMap<String, AdaptivePSOControl> vehControlsMap;
    private CommMedium commMedium;
    private String[] vehicleNames;
    
    // Particles
    private List<Spatial> particlesList = new ArrayList<Spatial>(NUM_PARTICLES);
    
    // Multi-threading
    private ScheduledThreadPoolExecutor executor;
    
    private Simulator sim;
    
    // Statistics
    public Integer receivedMsgCount = 0;
    
    // Boundary conditions
    //private boolean checkOne, checkTwo;
    
    // Search
    private List<Spatial> targetList = new ArrayList<Spatial>();
    
    // Swarms
    Swarms swarms;
    Swarm initialSwarm;
    String initialSwarmID;
    
    // Terrain
    private Spatial terrain = null;
    private RigidBodyControl terrainPhysics = null;
    
    // Simulation
    private int simulationTicks = 0;
    private int numVehiclesAtTarget = 0;
    
    private float containerVolume;
    private ArrayList<Vector3f> inContainerVehPositions;
    
    // AppStates
    SimulationCore simCore;
    SearchMode searchSim;
    TaskAllocMode taskAllocSim;
    InitSelfOrgMode initSelfOrgSim;
    GeneralTestsMode generalTestsSim;
    IntegrationMode integrationSim;
    
    // GUI States
    StartScreen startScreen;
    Nifty nifty;
    
    // Control variables
    private boolean swarmInitialized = false,
                    commMediumInitialized = false;
    
    
    // Task Allocation
    private HashMap<String, Integer> targetProcessings;
    
    private boolean targetListRetrieved = false;
    private float totalTargetsNum;
    
    private boolean targ33PerentileFound = false,
                    targ66PerentileFound = false,
                    targ99PerentileFound = false;
    
    // Plotting
    
    // Create an ITrace: 
    // Note that dynamic charts need limited amount of values!!! 
    private final ITrace2D trace = new Trace2DLtd(200); 
    
    private int simStartTime;
   
    boolean[] percentagesFlags;
    
    
    // Misc
    private Spatial vehicleModel;
    
    // File writing variables    
    private Path metricValuesFile;
    
    private List<String> avgDistToTargValues     = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         avgSpreadValues         = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         maxDistFromSourceValues = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         maxDistFromTargValues   = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         numReachedTargValues    = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         numLostAgentsValues     = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         numSurvivingAgentsValues= new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         numSentMsgsValues       = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         numReceivedMsgsValues   = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT),
                         overallTraveledDistValues   = new ArrayList<String>((int)MAX_SIMULATION_TICK_COUNT);
    
    
    // Simulation variables
    private int runNumber = -1,
                numRuns;
    
    
    // Sensing aid
    private Map<String, UWVehicleControl> targetBeaconingAUVs;
    
    // Targets
    private HashMap<String, TargetType> targetTypesMap;
    
    // Vehicle recovering
    private ArrayList<String> recoveredVehicles,
                              doneVehicles,
                              releasedVehicles;
    
    //==========================================================================
    //==========================================================================

//    public static void main(String[] args) {
//        
//        Simulator sim = new Simulator();
//        sim.start();
//        
//    }
    
    public Simulator() {}
    
    public Simulator(int runNumber, int numRuns) {
        this.runNumber = runNumber;
        this.numRuns = numRuns;
    }
    
    @Override
    public void simpleInitApp() {
        
        this.sim = this;
        
        if(!IN_BULK_SIM_RUN_MODE) {
        
            // GUI
            NiftyJmeDisplay niftyDisplay = new NiftyJmeDisplay(
                    assetManager, inputManager, audioRenderer, guiViewPort);
            nifty = niftyDisplay.getNifty();
            guiViewPort.addProcessor(niftyDisplay);
            flyCam.setDragToRotate(true);

            nifty.loadStyleFile("nifty-default-styles.xml");
            nifty.loadControlFile("nifty-default-controls.xml");

            // <screen>
            nifty.addScreen("start_screen", new ScreenBuilder("Simulator Entry Screen"){{

                startScreen = new StartScreen(sim, nifty);
                stateManager.attach(startScreen);
                controller(startScreen); // Screen properties       

                // <layer>
                layer(new LayerBuilder("options_layer") {{
                    childLayoutVertical(); // layer properties, add more...

                    // <panel>
                    panel(new PanelBuilder("top_panel") {{
                        childLayoutVertical();
                        backgroundColor("#000D");
                        height("100%");
                        width("15%");

                        // GUI elements
                        control(new LabelBuilder("label_Num_Agents", "Number of AUVs: "+NUM_AGENTS+"\n"){{
                            width("70%");
                        }});
                        control(new SliderBuilder("slider_Num_Agents", false){{
                            min(1);
                            max(200);
                            initial(NUM_AGENTS);
                            buttonStepSize(1);
                            width("70%");
                        }}); // creates a horizontal slider

                        control(new ButtonBuilder("Self_Org_Sim_Button", "Initial SO Simulation"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("25");
                            visibleToMouse(true);
                            interactOnClick("modeButtonHandler(2)");
                        }});
                        
                        control(new DropDownBuilder("dropdown_Search_Sim"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("25");
                            //visibleToMouse(true);
                            //interactOnClick("searchDropDownHandler()");
                        }});

                        control(new ButtonBuilder("Search_Sim_Button", "Target Search Simulation"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("10");
                            visibleToMouse(true);
                            interactOnClick("modeButtonHandler(0)");
                        }});

                        control(new DropDownBuilder("dropdown_Taskalloc_Sim"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("25");
                        }});
                        
                        control(new ButtonBuilder("Task_Alloc_Sim_Button", "Task Allocation Simulation"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("10");
                            visibleToMouse(true);
                            interactOnClick("modeButtonHandler(1)");
                        }});

                        control(new DropDownBuilder("dropdown_Integ_Sim"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("25");
                        }});
                        
                        control(new ButtonBuilder("Integration_Sim_Button", "Integration Simulation"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("10");
                            visibleToMouse(true);
                            interactOnClick("modeButtonHandler(3)");
                        }});

                        control(new ButtonBuilder("General_Tests_Sim_Button", "General Test"){{
                            alignCenter();
                            height("3%");
                            width("70%");
                            marginTop("25");
                            visibleToMouse(true);
                            interactOnClick("modeButtonHandler(4)");
                        }});

    //                    control(new LabelBuilder("pd_label", "Enable Path Debug: \n"));
    //                    
    //                    control(new CheckboxBuilder("chkbx_veh_path_debug") {{
    //                        //checked(UWVehicleControl.getVehiclePathDebug()); // start with uncheck
    //                        //visibleToMouse(true);
    //                        //id("chkbx_veh_path_debug");
    //                        //interactOnClick("debugCheckHandler(1,\""+this.getId()+"\")");
    //                    }});

                    }});

                  }});
                // </layer>
              }}.build(nifty));
            // </screen>

            nifty.gotoScreen("start_screen"); // start the screen
        
        } else {
            flyCam.setDragToRotate(true);
        }
        
        /* This constructor creates a new executor with a core pool size of 4. */
        //executor = new ScheduledThreadPoolExecutor(4);
        
        simStartTime = getCurrentTick();
        
        //initSimulation();
        initPhysics(); 
        setupCamera();
        
//        searchSim = new SearchMode();
//        stateManager.attach(searchSim);
//        searchSim.initialize(stateManager, sim);
//        searchSim.setEnabled(SEARCH_MODE_ENABLED);
//        
//        taskAllocSim = new TaskAllocMode();
//        stateManager.attach(taskAllocSim);
//        taskAllocSim.initialize(stateManager, sim);
//        taskAllocSim.setEnabled(TASK_ALLOC_MODE_ENABLED);
//        
//        initSelfOrgSim = new InitSelfOrgMode();
//        stateManager.attach(initSelfOrgSim);
//        initSelfOrgSim.initialize(stateManager, sim);
//        initSelfOrgSim.setEnabled(INIT_SELF_ORG_MODE_ENABLED);
//        
//        integrationSim = new IntegrationMode();
//        stateManager.attach(integrationSim);
//        integrationSim.initialize(stateManager, sim);
//        integrationSim.setEnabled(INTEGRATION_MODE_ENABLED);
//        
//        generalTestsSim = new GeneralTestsMode();
//        stateManager.attach(generalTestsSim);
//        generalTestsSim.initialize(stateManager, sim);
//        generalTestsSim.setEnabled(GENERAL_TESTS_MODE_ENABLED);
        
        if(IN_BULK_SIM_RUN_MODE) {
            UWVehicleControl.setSearchAlgorithm(BULK_SIM_SEARCH_ALG);
            initSimulator(BULK_SIM_RUN_MODE);
        }
        
        if( USE_COMPLEX_ENVIRONMENT ) {
            createWater();
        }
        
        //createInitialSwarm();
        
        if( USE_COMPLEX_ENVIRONMENT ) {
            addShip();
        }
        
        //addCommMedium();
        
        simulationTicks++;
        
        percentagesFlags = new boolean[20];
        
        for(int i = 0; i < 20; i++) {
            percentagesFlags[i] = false;
        }
        
        //testCSG();
        
        //DebugUtils.plotCrossHair(new Vector3f(0,45,0), 3, 0.5f, sim);
        //DebugUtils.plotArrow(new Vector3f(0,45,0), new Vector3f(7,10,7), ColorRGBA.Blue, 3, sim);
        
        
        //runSearchAlgorhithm(Algorithms.RDPSO);
        
        //addTarget();
        //createInfinitePlane();  
        
        // =====================================================================
        if(TASK_ALLOC_MODE_ENABLED && SHOW_PLOT) {
            // Create a chart:  
            Chart2D chart = new Chart2D();

            trace.setColor(Color.RED);

            // Add the trace to the chart. This has to be done before adding points (deadlock prevention): 
            chart.addTrace(trace);

            // Make it visible:
            // Create a frame. 
            JFrame frame = new JFrame("MinimalDynamicChart");
            // add the chart to the frame: 
            frame.getContentPane().add(chart);
            frame.setSize(400,300);
            // Enable the termination button [cross on the upper right edge]: 
            frame.addWindowListener(
                new WindowAdapter(){
                  public void windowClosing(WindowEvent e){
                      System.exit(0);
                  }
                }
              );
            frame.setVisible(true); 

            /* 
             * Now the dynamic adding of points. This is just a demo! 
             * 
             * Use a separate thread to simulate dynamic adding of date. 
             * Note that you do not have to copy this code. Dynamic charting is just about 
             * adding points to traces at runtime from another thread. Whenever you hook on 
             * to a serial port or some other data source with a polling Thread (or an event 
             * notification pattern) you will have your own thread that just has to add points 
             * to a trace. 
             */

    //        Timer timer = new Timer(true);
    //        TimerTask task = new TimerTask(){
    //
    //          private double m_y = 0;
    //          private long m_starttime = System.currentTimeMillis();
    //          /**
    //           * @see java.util.TimerTask#run()
    //           */
    //          @Override
    //          public void run() {
    //            // This is just computation of some nice looking value.
    //            double rand = Math.random();
    //            boolean add = (rand >= 0.5) ? true : false;
    //            this.m_y = (add) ? this.m_y + Math.random() : this.m_y - Math.random();
    //            // This is the important thing: Point is added from separate Thread.
    //            trace.addPoint(((double) System.currentTimeMillis() - this.m_starttime), this.m_y);
    //          }
    //
    //        };
    //        // Every 20 milliseconds a new value is collected.
    //        timer.schedule(task, 1000, 20);
        }
        // =====================================================================
    
        targetBeaconingAUVs = new HashMap<String, UWVehicleControl>();
        
        recoveredVehicles = new ArrayList<String>();
        doneVehicles = new ArrayList<String>();
        releasedVehicles = new ArrayList<String>();
    }
    
    private void setSimulationMode(int mode) {
        
        //System.out.println("Simulation mode set ...");
        
        switch(mode) {
            case 0: // Search
                SEARCH_MODE_ENABLED = true;
                searchSim = new SearchMode();
                stateManager.attach(searchSim);
                searchSim.initialize(stateManager, sim);
                searchSim.setEnabled(SEARCH_MODE_ENABLED);
                break;
            case 1: // Task Allocation
                TASK_ALLOC_MODE_ENABLED = true;
                taskAllocSim = new TaskAllocMode();
                stateManager.attach(taskAllocSim);
                taskAllocSim.initialize(stateManager, sim);
                taskAllocSim.setEnabled(TASK_ALLOC_MODE_ENABLED);
                break;
            case 2: // Self Organization
                INIT_SELF_ORG_MODE_ENABLED = true;
                initSelfOrgSim = new InitSelfOrgMode();
                stateManager.attach(initSelfOrgSim);
                initSelfOrgSim.initialize(stateManager, sim);
                initSelfOrgSim.setEnabled(INIT_SELF_ORG_MODE_ENABLED);
                break;
            case 3: // Self Organization
                INTEGRATION_MODE_ENABLED = true;
                integrationSim = new IntegrationMode();
                stateManager.attach(integrationSim);
                integrationSim.initialize(stateManager, sim);
                integrationSim.setEnabled(INTEGRATION_MODE_ENABLED);
                break;
            case 4: // General
            default:
                GENERAL_TESTS_MODE_ENABLED = true;
                generalTestsSim = new GeneralTestsMode();
                stateManager.attach(generalTestsSim);
                generalTestsSim.initialize(stateManager, sim);
                generalTestsSim.setEnabled(GENERAL_TESTS_MODE_ENABLED);
                break;
        }
    }

    public void initSimulator(int mode) {
        
        if(!IN_BULK_SIM_RUN_MODE) {
            nifty.removeScreen("start_screen");
        }
        initSimulation();
        setSimulationMode(mode);
        
        if(!swarmInitialized) {
            createInitialSwarm();
            swarmInitialized = true;
        }   
        if(!commMediumInitialized) {
            addCommMedium();
            commMediumInitialized = true;
        }
        
        simulationInitialized = true;
    }
    
    public void toggleFlag(String id) {        
        if( id.equals("chkbx_veh_path_debug") ) {
            UWVehicleControl.toggleVehiclePathDebug();
        }
    }
    
    private void runSearchAlgorhithm(SearchAlgorithms algorithm){
        
        SearchAlgorithm searchAlgorithm = null;
        
        switch(algorithm) {
            case RDPSO:
                searchAlgorithm = new RDPSO(null, this); // Hack ... be careful!
                break;
            case BFM:
                
                break;
            case BDFSO:
                
                break;
            default:
                System.out.println("Unknown search algorithm!");
                System.exit(-1);
        }
        
        searchAlgorithm.setMaxIterations(1000);
        searchAlgorithm.start();
        
    }
    
    @Override
    public void simpleUpdate(float tpf) {

        if( getCurrentTick() % 1000 == 0 ) {
            System.out.println("Elapsed Simulation Ticks: " + getCurrentTick());
        }
        
        if( INTEGRATION_MODE_ENABLED && missionCompleted && !missionStatsPrinted ) {
            
            // Mission performance evaluation params
            
            // Check if target was ever found by any vehicle
            boolean targetEverFound = false;
            float alpha = 0;
            for(Entry<String, UWVehicleControl> auvEntry : vehControlsMap.entrySet()) {
                if( auvEntry.getValue().targetEverEncountered() ) {
                    targetEverFound = true;
                    break;
                }
            }
            
            alpha = (targetEverFound ? 1 : 0);
            
            // Find target processing counts, mission cost, and value
            String targetName;
            TargetType currentType;
            
            for( Entry<String, TargetType> entry : targetTypesMap.entrySet() ) {
                
                targetName = entry.getKey();
                currentType = entry.getValue();
                
                switch(currentType) {
                    case RED_BARREL:
                        // Accrued mission value (of processed targets only)
                        // Target has been processed (at least once)
                        if( targetProcessings.get(targetName) > 0 ) {
                            obtainedMissionValue += TYPE_I_TARGET_PROCESSING_VALUE;
                            ++processed_TypeI_Targets;
                        }
                        // Full mission value (value accrued if all targets
                        // were processed)
                        targetsActualValue += TYPE_I_TARGET_PROCESSING_VALUE;
                        ++typeI_Targets;
                        break;
                    case GREEN_BARREL:
                        if( targetProcessings.get(targetName) > 0 ) {
                            obtainedMissionValue += TYPE_II_TARGET_PROCESSING_VALUE;
                            ++processed_TypeII_Targets;
                        }
                        targetsActualValue += TYPE_II_TARGET_PROCESSING_VALUE;
                        ++typeII_Targets;
                        break;
                    case BLUE_BARREL:
                        if( targetProcessings.get(targetName) > 0 ) {
                            obtainedMissionValue += TYPE_III_TARGET_PROCESSING_VALUE;
                            ++processed_TypeIII_Targets;
                        }
                        targetsActualValue += TYPE_III_TARGET_PROCESSING_VALUE;
                        ++typeIII_Targets;
                        break;
                    case SHIP:
                        if( targetProcessings.get(targetName) > 0 ) {
                            obtainedMissionValue += SUNK_SHIP_VALUE;
                        }
                        targetsActualValue += SUNK_SHIP_VALUE;
                        break;
                }  
            }
            
            // Dollar gain            
            if( targetTypesMap.size() > 1 ) {   // Barrels
                                
                dollarGain = 
                    ((processed_TypeI_Targets * TYPE_I_TARGET_PROCESSING_VALUE) +
                    (processed_TypeII_Targets * TYPE_II_TARGET_PROCESSING_VALUE) +
                    (processed_TypeIII_Targets * TYPE_III_TARGET_PROCESSING_VALUE)) / missionCost;
                
                
                System.out.println("Percentage of type I targets processed = " + (100*processed_TypeI_Targets/typeI_Targets));
                System.out.println("Percentage of type II targets processed = " + (100*processed_TypeII_Targets/typeII_Targets));
                System.out.println("Percentage of type III targets processed = " + (100*processed_TypeIII_Targets/typeIII_Targets));
                
            } else { // Sunk ship
                
                dollarGain = obtainedMissionValue / missionCost; 
            }
            
            if( targetEverFound ) {
                System.out.println("Dollar gain = " + dollarGain);
            }
            
            // Mission failure loss
            failureLoss = 1/(targetsActualValue + missionCost);
            if( !targetEverFound ) {
                System.out.println("Mission failure loss = " + failureLoss);
            }
            
            // Perecnt of recovered AUVs
            percentRecoveredAgents = ((float)recoveredVehicles.size())/NUM_AGENTS;
            System.out.println("Percentage recovered = " + percentRecoveredAgents);
            
            // Degree of time compliance
            degreeOfTimeCompliance = MISSION_TIME_CONSTRAINT/actualTimeSpentInMission;
            System.out.println("Degree of time compliance = " + degreeOfTimeCompliance);
            
            // Mission Gain
            missionGain = degreeOfTimeCompliance * percentRecoveredAgents * (alpha*dollarGain + (1-alpha)*failureLoss);
            
            if( !missionGainPrinted ) {
                System.out.println("Mission Gain = " + missionGain);
                missionGainPrinted = true;
            }
 
            missionStatsPrinted = true;
            
        }
        
        if(!simulationInitialized) {
            
        }
        
        if(!targetListRetrieved && (INTEGRATION_MODE_ENABLED || TASK_ALLOC_MODE_ENABLED)) {
            
            //System.out.println("Entered ...");
            ArrayList<Spatial> targetsList;
            if(INTEGRATION_MODE_ENABLED) {
                targetsList = (ArrayList<Spatial>)integrationSim.getTargetList();
            } else {
                targetsList = (ArrayList<Spatial>)taskAllocSim.getTargetList();
            }
             
            int numTargets = targetsList.size();
            targetProcessings = new HashMap<String, Integer>(numTargets);
            
            for(Spatial targ : targetsList) {
                targetProcessings.put(targ.getName(), 0);
            }
                        
            targetListRetrieved = true;
            
            //System.out.println("Target List Retrieved");
        }

        if(simulationInitialized) {
            
            // Tick counting
            simulationTicks++;
            //System.out.println("Total simulation ticks: "+simulationTicks);
            if(simulationTicks > MAX_SIMULATION_TICK_COUNT) {
                printSimulationTickCount();
                stopped = true;
                this.stop();
            }
            
            // Generate some performance metrics
            // ---------------------------------
            //Average distance from target over time
            calculateAvgDistFromTarget();
            
            // Average spread of the swarm (from their average position)
            calculateAvgSpread();
            
            // Number of lost agents over time
            calculateNumLostAgents();
            
            // Number of agents that found the target over time
            calculateNumReachedTarget();
            
            // Maximum distance from target
            calculateMaxDistFromTarget();
            
            // Maximum distance from source
            calculateMaxDistFromSource();
            
            // Number of surviving agents
            calculateNumSurvivingAgents();
            
            // Number of sent messages
            calculateSentMsgsCount();
            
            // Number of received messages
            calculateReceivedMsgsCount();
            
            // Overall distance traveled
            calculateOverallDistTraveled();
            
            if(TASK_ALLOC_MODE_ENABLED || INTEGRATION_MODE_ENABLED) {
               // Number of target processings
               calculateNumTargProcessings();
            }
            
            if( getCurrentTick() == MAX_SIMULATION_TICK_COUNT - 1 && doneVehicles.size() != numSurvivingAUVs) {
                missionEndTick = getCurrentTick();
                if( !printedMissionCompl ) { 
                    System.out.println("Mission Completed: "+getCurrentTick());
                    printedMissionCompl = true;
                }
                actualTimeSpentInMission = missionEndTick - missionStartTick;
                missionCompleted = true;
            }
            
            
        }
        
//        if(numVehiclesAtTarget > 0.3 * NUM_AGENTS) {
//            System.out.println("Total simulation ticks: "+simulationTicks);
//            System.out.println("30% of vehicles found the target.");
//            this.stop();  
//        }
        
//        boolean turned = false;
        
//        for(int i = 0; i < NUM_AGENTS; i++) {
//            if( uwVehicles[i].getPos().y < 20 ) {
                
//                if( uwVehicles[i].getPos().y < 15 && !turned ) {
//                    uwVehicles[i].turnRight(tpf);
//                    turned = true;
//                } else {
                    //uwVehicles[i].move(tpf);
                    
                
                //uwVehicles[i].pushUp(new Vector3f(0, -14.224491f, 0));
                //uwVehicles[i].getControl().activate();
                
//                System.out.println("=============================================");
//                System.out.println( "Linear Velocity: " + uwVehicles[i].getLinearVelocity() );
//                System.out.println("=============================================");
//                }
//                System.out.println("=============================================");
//                System.out.println("Num Particles: " + uwVehicles[i].getSensorReading());
//                System.out.println("=============================================");
//            }
//        }
        
        
//        checkOne = physicsAgent.getPhysicsLocation().length() > bulletAppState.getPhysicsSpace().getWorldMax().length();
//        checkTwo = physicsAgent.getPhysicsLocation().length() > bulletAppState.getPhysicsSpace().getWorldMin().length();
//        
//        //System.out.println(physicsAgent.getPhysicsLocation().length() + " " + bulletAppState.getPhysicsSpace().getWorldMax().length());
//        //System.out.println(physicsAgent.getPhysicsLocation().length() + " " + bulletAppState.getPhysicsSpace().getWorldMin().length());
//        
//        if( !checkOne && !checkTwo ) {
//            //agent.getControl(AgentController.class).setState(State.idle); 
//            currLocation = physicsAgent.getPhysicsLocation();
//            currRotation = physicsAgent.getPhysicsRotation();
//            //agentForwardDir = agent.getWorldRotation().mult(Vector3f.UNIT_X);
//            physicsAgent.setPhysicsLocation(currLocation.add(new Vector3f((float)Math.random()*tpf*10*currRotation.getX(),
//                    (float)Math.random()*tpf*10*currRotation.getY(),
//                    (float)Math.random()*tpf*10*currRotation.getZ())));
//        }
    }
    
    private void calculateAvgDistFromTarget() {
        
        if(targetList.isEmpty()) {

            if(INTEGRATION_MODE_ENABLED) {
                targetList = integrationSim.getTargetList();
            } else if(SEARCH_MODE_ENABLED) {
                targetList = searchSim.getTargetList();
            }
        }

        if(!targetList.isEmpty()) {
            // Get distance from each vehicle to target
            float distancesSum = 0;
            int vehCount = vehControlsMap.size();

            UWVehicleControl vehControl;
            for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
                vehControl = vehEntry.getValue();

                if(!vehControl.vehExceededBoundary()) {
                    distancesSum += vehControl.getPhysicsVehicle().getPhysicsLocation()
                        .distance(targetList.get(0).getLocalTranslation()); // We currently use one target
                } else {
                    vehCount--;
                }
            }

            float avgDistFromTarg = distancesSum/vehCount;
            avgDistToTargValues.add(String.valueOf(avgDistFromTarg));
        }            

        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(avgDistToTargValues, AVG_DIST_TO_TARG_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
        
    }
    
    private void calculateAvgSpread() {

        // Common variables
        float distanceFromAvg = 0;
        int vehCount = vehControlsMap.size();
        Vector3f vehPos, avgVehPos = Vector3f.ZERO;
        ArrayList<Float> distsFromAvg = new ArrayList<Float>();
        
        // Find average position of all vehicles
        UWVehicleControl vehControl;
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();

            if(!vehControl.vehExceededBoundary()) {                
                vehPos = vehControl.getPhysicsVehicle().getPhysicsLocation();
                avgVehPos.add(vehPos);
            } else {
                vehCount--;
            }
        }
        avgVehPos = avgVehPos.divide(vehCount);
        
        // Get all vehicles' distances from that average
        vehCount = vehControlsMap.size();
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();

            if(!vehControl.vehExceededBoundary()) {                
                vehPos = vehControl.getPhysicsVehicle().getPhysicsLocation();
                distanceFromAvg = vehPos.distance(avgVehPos);
                distsFromAvg.add(distanceFromAvg);
            } else {
                vehCount--;
            }
        }
        
        // Find their sum (used for getting the average)
        float sumDistFromAvgPos = 0;
        for(Float dist : distsFromAvg) {
            sumDistFromAvgPos += dist;
        }

        // Get the average of all vehicle distances from the average position;
        // This will be the average spread value
        float avgDistFromAvgPos = sumDistFromAvgPos/vehCount;
        avgSpreadValues.add(String.valueOf(avgDistFromAvgPos));        

        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(avgSpreadValues, AVG_SPREAD_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
        
    }    
    
    private void calculateNumLostAgents() {
        
        int numLostAgents = 0, numSurvivingAgents = 0;
        
        UWVehicleControl vehControl;
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();

            if(vehControl.vehExceededBoundary() && !vehControl.isTargetFound()) {                
                numLostAgents++;
            }
        }
        
        numSurvivingAgents = vehControlsMap.size() - numLostAgents;
        
        numSurvivingAUVs = numSurvivingAgents;
        
        numLostAgentsValues.add(String.valueOf(numLostAgents)); 
        numSurvivingAgentsValues.add(String.valueOf(numSurvivingAgents)); 
        
        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(numLostAgentsValues, NUM_LOST_AGENTS_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
    }
    
    private void calculateNumReachedTarget() {
        
        if(targetList.isEmpty()) {

            if(INTEGRATION_MODE_ENABLED) {
                targetList = integrationSim.getTargetList();
            } else if(SEARCH_MODE_ENABLED) {
                targetList = searchSim.getTargetList();
            }
        }

        if(!targetList.isEmpty()) {

            int countReachedTarg = 0;

            UWVehicleControl vehControl;
            for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
                vehControl = vehEntry.getValue();
                if( vehControl.isTargetEverEncountered() ) {
                    countReachedTarg++;
                }
            }

            numReachedTargValues.add(String.valueOf(countReachedTarg));
        }            

        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(numReachedTargValues, NUM_REACHED_TARG_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
    }
    
    private void calculateMaxDistFromTarget() {
        
        if(targetList.isEmpty()) {

            if(INTEGRATION_MODE_ENABLED) {
                targetList = integrationSim.getTargetList();
            } else if(SEARCH_MODE_ENABLED) {
                targetList = searchSim.getTargetList();
            }
        }

        if(!targetList.isEmpty()) {
            // Get distance from each vehicle to target
            float maxDistFromTarg = -1;

            UWVehicleControl vehControl;
            for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
                vehControl = vehEntry.getValue();

                if(!vehControl.vehExceededBoundary()) {
                    maxDistFromTarg = Math.max(maxDistFromTarg, vehControl.getPhysicsVehicle().getPhysicsLocation()
                        .distance(targetList.get(0).getLocalTranslation())) ; // We currently use one target
                }
            }
            
            maxDistFromTargValues.add(String.valueOf(maxDistFromTarg));
        }            

        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(maxDistFromTargValues, MAX_DIST_FROM_TARG_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
        
    }
    
    private void calculateMaxDistFromSource() {

        // Get distance from each vehicle to target
        float maxDistFromSource = 0;

        UWVehicleControl vehControl;
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();

            if(!vehControl.vehExceededBoundary()) {
                maxDistFromSource = Math.max(maxDistFromSource, vehControl.getPhysicsVehicle().getPhysicsLocation()
                    .distance(UWVehicleControl.getDropOffLocation())) ; // We currently use one target
            }
        }

        maxDistFromSourceValues.add(String.valueOf(maxDistFromSource));
          

        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(maxDistFromSourceValues, MAX_DIST_FROM_SOURCE_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
        
    }
    
    private void calculateNumSurvivingAgents() {
        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(numSurvivingAgentsValues, NUM_SURV_AGENTS_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
    }
    
    private void calculateSentMsgsCount() {
        
        int totalNumSentMsgs = 0;
        
        UWVehicleControl vehControl;
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();
            totalNumSentMsgs += vehControl.getSentMsgCount();
        }
        
        numSentMsgsValues.add(String.valueOf(totalNumSentMsgs)); 
        
        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(numSentMsgsValues, NUM_SENT_MSGS_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
    }
    
    private void calculateReceivedMsgsCount() {
        int totalNumReceivedMsgs = 0;
        
        UWVehicleControl vehControl;
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();
            totalNumReceivedMsgs += vehControl.getReceivedMsgCount();
        }
        
        numReceivedMsgsValues.add(String.valueOf(totalNumReceivedMsgs)); 
        
        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(numReceivedMsgsValues, NUM_RECE_MSGS_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }        
    }
    
    private void calculateOverallDistTraveled() {
        int overallDist = 0;
        
        UWVehicleControl vehControl;
        for(Entry<String, UWVehicleControl> vehEntry : vehControlsMap.entrySet()) {
            vehControl = vehEntry.getValue();
            overallDist += vehControl.getOverallDistTaveled();
        }
        
        overallTraveledDistValues.add(String.valueOf(overallDist)); 
        
        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            storeMetricResults(overallTraveledDistValues, DIST_TRAVELED_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }        
    }
    
    private void calculateNumTargProcessings() {
        
        if(simulationTicks == MAX_SIMULATION_TICK_COUNT) {
            
            ArrayList<String> numTargProcess = new ArrayList<String>(targetProcessings.size());

            for( Entry<String, Integer> targetProcessing : targetProcessings.entrySet() ) {
                numTargProcess.add(String.valueOf(targetProcessing.getValue()));
            }
            
            storeMetricResults(numTargProcess, NUM_TARG_PROCESSINGS_FILE_NAME+(runNumber != -1 ? "_"+runNumber : "")+".txt");
        }
    }
    
    private void storeMetricResults(List<String> results, String fileName) {
        
        metricValuesFile = Paths.get(fileName);
        
        try {
            Files.write(metricValuesFile, results, Charset.forName(FILE_ENCODING));
        } catch (IOException ex) {
            Logger.getLogger(Simulator.class.getName()).log(Level.SEVERE, null, ex);
        }
                         
    }
    
//    public void removeVehicleRecords(String name) {
//        vehControlsMap.remove(name);
//        vehicles.remove(rootNode.getChild(name));
//        
//        int index = 0;
//        for(UWVehicle veh : uwVehicles) {
//            if(veh.getControl().getSpatial().getName().equals(name)) {
//                uwVehicles[index] = null;
//            }
//            index++;
//        }
//        
//        vehiclesMap.remove(name);
//        
//        index = 0;
//        for(String veh : vehicleNames) {
//            if(veh.equals(name)) {
//                vehicleNames[index] = null;
//            }
//            index++;
//        }
//    }

    public void printSimulationTickCount() {
        System.out.println("Simulation tick count: "+simulationTicks);
    }
    
    public int getCurrentTick() {
        return simulationTicks;
    }
    
    @Override
    public void simpleRender(RenderManager rm) {
        //TODO: add render code
    }
    
    private void initSimulation() {

        // Get and store vehicle info
        vehicleModel = assetManager.loadModel("Models/Agent/agm65/AUV.j3o");
        float vehicleVolume = vehicleModel.getWorldBound().getVolume(); 
        containerVolume = vehicleVolume * NUM_AGENTS;
        inContainerVehPositions = new ArrayList<Vector3f>(NUM_AGENTS);
        
    }
    
    private void initPhysics() {
        
        // Add Bullet physics
        bulletAppState = new BulletAppState();
        
        bulletAppState.setWorldMin(new Vector3f(-120f, 0f, -120f));
        bulletAppState.setWorldMax(new Vector3f(120f, 17f, 120f));
        
        // Attach it to the stateManager
        stateManager.attach(bulletAppState);
        
        bulletAppState.setDebugEnabled(DEBUG_ENABLED);
        
        //attachCoordinateAxes(new Vector3f(0, 25, 0));

        // Set some Physics defaults (Specify the defaults before adding physical 
        // objects to the physics space—changing the defaults later has no effect 
        // on physical objects that are already in the physics space!)
        bulletAppState.getPhysicsSpace().setGravity(new Vector3f(0f, -9.81f, 0f));
        
        // Create a TestWorld to do some tests
        //PhysicsTestHelper.createPhysicsTestWorld(rootNode, assetManager, bulletAppState.getPhysicsSpace());
        terrain = PhysicsHelper.createPhysicsWorld(rootNode, assetManager, bulletAppState.getPhysicsSpace(), this);
        
        terrainPhysics = PhysicsHelper.getTerrainPhysics();
        
        viewPort.setBackgroundColor(ColorRGBA.White); //new ColorRGBA(0.17f, 0.6f, 1.0f, 1.0f)
              
    }
    
    public Spatial getTerrain() {
        return terrain;
    }
    
    public RigidBodyControl getTerrainPhysics() {
        return terrainPhysics;
    }
    
    private void setupCamera() {
        
        flyCam.setMoveSpeed(100); 
     
        // Create a Quaternion to use in rotating the camera
        Quaternion roll320 = new Quaternion();
        
        // Roll it around x-axis 45 degrees
        //roll320.fromAngleAxis( 320*FastMath.DEG_TO_RAD , Vector3f.UNIT_X );   // tilted view
        roll320.fromAngleAxis( -90*FastMath.DEG_TO_RAD , Vector3f.UNIT_X );     // top view
        
        // Get current camera rotation
        Quaternion qRot = cam.getRotation();
        
        // Calculate new rotation
        Quaternion newCamRot = roll320.mult(qRot);
        
        // Rotate the camera
        cam.setRotation(newCamRot);
        
        // Change camera location
        //cam.setLocation(new Vector3f(0f,200f,260f));      // tilted view
        cam.setLocation(new Vector3f(0f,350f,10f));          // top view
        
    }
    
    private void createSimpleWater() {

    }
    
    private void createWater() {
        
        fpp = new FilterPostProcessor(assetManager);
        
        water = new WaterFilter(rootNode, lightDir);
        water.setWaterHeight(initialWaterHeight);
        water.setCenter(new Vector3f(0,30,0));
        water.setRadius(200f);
        water.setWaterColor(ColorRGBA.Cyan);
        //water.setWaterTransparency(0.05f);
        //water.setColorExtinction(new Vector3f(50f,50f,100f));
        //water.setWaveScale(0.008f);
        
        fpp.addFilter(water);
        viewPort.addProcessor(fpp);
        
        waves = new AudioNode(assetManager, "Sounds/Environment/Ocean Waves.ogg", false);
        waves.setLooping(true);
        audioRenderer.playSource(waves);    
        
    } 
     
    private void createInitialSwarm() {
        
        //uwVehicles = new ArrayList(NUM_AGENTS);
        uwVehicles = new UWVehicle[NUM_AGENTS];
        vehicles = new ArrayList<Spatial>(NUM_AGENTS);
        vehControlsMap = new HashMap<String, UWVehicleControl>(NUM_AGENTS);
        //vehControlsMap = new HashMap<String, AdaptivePSOControl>(NUM_AGENTS);
        vehiclesMap = new HashMap<String, Agent>(NUM_AGENTS);
        
        vehicleNames = new String[NUM_AGENTS];
        
        float velFactor = 0.1f;

        for(int i = 0; i < NUM_AGENTS; i++) {  
            //uwVehicles.add(new UWVehicle(new Vector3f(), velFactor, rootNode, bulletAppState, assetManager, sim));
            uwVehicles[i] = new UWVehicle(new Vector3f(), velFactor, rootNode, bulletAppState, assetManager, sim);
       
            //uwVehicles[i] = new UWVehicle(new Vector3f(), velFactor, rootNode, simCore.getBulletAppState(), assetManager, sim);
            
            // Set vehicle's name
            uwVehicles[i].setName("V_"+i);
            vehiclesMap.put("V_"+i, uwVehicles[i]);
            
            // Set vehicle's preferred target type based on predefined percentages
            if( (INTEGRATION_MODE_ENABLED && 
                    !getStateManager().getState(IntegrationMode.class).usingTargetConfiguration(TargetConfig.SHIP)) || 
                    
                (TASK_ALLOC_MODE_ENABLED &&
                    !getStateManager().getState(TaskAllocMode.class).usingTargetConfiguration(TargetConfig.SHIP)) ) {
                uwVehicles[i].getControl().setPreferredTargType(getNextTargetType());
            }
            
            inContainerVehPositions.add(uwVehicles[i].getInContainerVehPos());
            
            // Debugging =======================================================
//            if(i == 0) { uwVehicles[i].setPos(new Vector3f(2, 10, 0)); }        
//            if(i == 1) { uwVehicles[i].setPos(new Vector3f(-2, 10, 0)); }          
//            if(i == 2) { uwVehicles[i].setPos(new Vector3f(0, 10, 2)); }
            // Debugging =======================================================

            vehicles.add(uwVehicles[i].getModel());
            vehicleNames[i] = "V_"+i;
            vehControlsMap.put(uwVehicles[i].getModel().getName(), uwVehicles[i].getControl());
            
            if(DEBUG_SHOW_VEHICLE_NAME) {
                DebugUtils.plotSpatialName(vehicles.get(i), 2f, ColorRGBA.Green, this);
            }
            
            //uwVehicles[i].setTargets(targetList);
            if(SEARCH_MODE_ENABLED) {
                uwVehicles[i].setAuxSearchTargets(stateManager.getState(SearchMode.class).getParticlesList());
                uwVehicles[i].setUltimateSearchTargets(stateManager.getState(SearchMode.class).getTargetList());       
            } else if(TASK_ALLOC_MODE_ENABLED) {
                uwVehicles[i].setUltimateSearchTargets(stateManager.getState(TaskAllocMode.class).getTargetList());
            } else if(INTEGRATION_MODE_ENABLED) {
                if( UWVehicleControl.getActiveSearchAlgorithm().equals(SearchAlgorithms.RPSO) ) {
                    uwVehicles[i].setAuxSearchTargets(stateManager.getState(IntegrationMode.class).getParticlesList());
                }
                uwVehicles[i].setUltimateSearchTargets(stateManager.getState(IntegrationMode.class).getTargetList());
            }
            
        }
        
        // Debugging =======================================================
//        vehControlsMap.get("V_0").setVelocity(new Vector3f(-10, 0, 0));
        //vehControlsMap.get("V_1").setVelocity(new Vector3f(10, 0, 0));
        //vehControlsMap.get("V_2").setVelocity(new Vector3f(0, 0, -10));
        // Debugging =======================================================
    
        for(int i = 0; i < NUM_AGENTS; i++) {  
            uwVehicles[i].getControl().setCommPartners(vehicles);
            uwVehicles[i].getControl().setVehicleTargets(vehicles);
        }
        
        // =====================================================================
        // Mission parameters
        missionCost = NUM_AGENTS * AUV_VALUE + OPERATIONAL_COST;
        
        // =====================================================================
        
        // Test swarms (this needs to be update if used ... remove detached AUVs)
        swarms = new Swarms();
        initialSwarmID = swarms.addSwarm(vehiclesMap);
        initialSwarm = swarms.getSwarm(initialSwarmID);
        
    }
    
    private TargetType getNextTargetType() {
        
        int selectedInd = -1;
        
        if( 100 * targTypeCounts[0]/NUM_AGENTS < TARG_TYPE_PERCENTAGES[0] &&
            100 * targTypeCounts[1]/NUM_AGENTS < TARG_TYPE_PERCENTAGES[1] &&
            100 * targTypeCounts[2]/NUM_AGENTS < TARG_TYPE_PERCENTAGES[2]) {
            selectedInd = FastMath.rand.nextInt(3);
        } else {
            for( int i = 0; i < 3; ++i ) {
                if(100 * targTypeCounts[i]/NUM_AGENTS < TARG_TYPE_PERCENTAGES[i] &&
                   100 * targTypeCounts[(i+1)%3]/NUM_AGENTS < TARG_TYPE_PERCENTAGES[(i+1)%3]) {
                    selectedInd = (FastMath.nextRandomFloat() > 0.5f ? i : (i+1)%3);
                }
            } 
            
            if( selectedInd == -1 ) {
                for( int i = 0; i < 3; ++i ) {
                    if(100 * targTypeCounts[i]/NUM_AGENTS < TARG_TYPE_PERCENTAGES[i]) {
                        selectedInd = i;
                    }
                } 
            }
        }

        ++targTypeCounts[selectedInd];
        
//        System.out.println("Type (" + 
//                    TargetType.getType(selectedInd).toString() + 
//                    ") target assignments used: " + targTypeCounts[selectedInd] + 
//                    "("+(100 * targTypeCounts[selectedInd]/NUM_AGENTS)+"%)");
        
        return TargetType.getType(selectedInd);

    }
    
    public float getVehiclesContainerVolume() {
        return containerVolume;
    }
    
    public ArrayList<Vector3f> getInContainerVehPosList() {
        return inContainerVehPositions;
    }

    private void addShip() {
        // Load agent and attach to the node
       shipModel = assetManager.loadModel("Models/Ships/Boat_Orig/boat.j3o");
       
       //shipModel.scale(0.008f);
       
       //Material mat_default = new Material(assetManager, "Common/MatDefs/Misc/ShowNormals.j3md");
       //shipModel.setMaterial(mat_default);
       
       physicsShip = new RigidBodyControl(0f);
       
       
       
       Quaternion rotY = new Quaternion();
       rotY.fromAngleAxis(FastMath.DEG_TO_RAD * -45, Vector3f.UNIT_Y);

       shipModel.addControl(physicsShip);
       
       physicsShip.setPhysicsRotation(rotY);
       physicsShip.setPhysicsLocation(new Vector3f(-30, 29, 10));
       
       rootNode.attachChild(shipModel);
       bulletAppState.getPhysicsSpace().add(physicsShip);
       
    }
    
    private void addTarget() {
        
        // Load agent and attach to the node
       targetModel = assetManager.loadModel("Models/Targets/Boxes/barrel_metal.j3o");
       
       targetModel.scale(0.2f);
       
       physicsTarget = new RigidBodyControl(20);

       targetModel.addControl(physicsTarget);
       
       physicsTarget.setPhysicsLocation(new Vector3f(70, 8, 15));
       
       rootNode.attachChild(targetModel);
       bulletAppState.getPhysicsSpace().add(physicsTarget);    
       
    }
    
    private void createInfinitePlane() {
        
        Box box1 = new Box(500,500,1);
        //Geometry waterPlane=new Geometry("water", quad);
        Geometry infinitePlane=new Geometry("infPlane", box1);
        infinitePlane.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
        infinitePlane.setLocalTranslation(0, 20, 0);
        Material material = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        //material.setTransparent(true);
        material.setColor("Color", ColorRGBA.Blue/*new ColorRGBA(1,1,1,0.01f)*/);
        //material.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        //material.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Front);
        infinitePlane.setMaterial(material);
        
        //waterPlane.addControl(new RigidBodyControl(0f));
        rootNode.attachChild(infinitePlane);
        
//        Quad quad = new Quad(1500,1500);
//        Geometry geo = new Geometry("MyQuad", quad);
//        
//        Quaternion rotX = new Quaternion();
//        rotX.fromAngleAxis(90*FastMath.DEG_TO_RAD, Vector3f.UNIT_X);
//        
//        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
//        mat.setColor("Color", ColorRGBA.Blue);
//        geo.setMaterial(mat);
//        
//        geo.rotate(rotX);
//        geo.setLocalTranslation(new Vector3f(0, -300, 0));
//        geo.setCullHint(Spatial.CullHint.Never);
//        
//        rootNode.attachChild(geo);
        
        
    }
    
    private void attachCoordinateAxes(Vector3f pos){
        Arrow arrow = new Arrow(new Vector3f(128, 0, 0));
        arrow.setLineWidth(1); // make arrow thicker
        putShape(arrow, ColorRGBA.Red).setLocalTranslation(pos);

        arrow = new Arrow(new Vector3f(0, 128, 0));
        arrow.setLineWidth(1); // make arrow thicker
        putShape(arrow, ColorRGBA.Green).setLocalTranslation(pos);

        arrow = new Arrow(new Vector3f(0, 0, 128));
        arrow.setLineWidth(1); // make arrow thicker
        putShape(arrow, ColorRGBA.Blue).setLocalTranslation(pos);
    }
 
    public Swarms getSwarms() {
        return swarms;
    }
    
    public void addCommMedium() {   
        commMedium = new CommMedium(this); 
        
        //Thread comMedThread = new Thread(commMedium);
        //comMedThread.start();
    }
    
    private Geometry putShape(Mesh shape, ColorRGBA color){
        Geometry g = new Geometry("coordinate axis", shape);
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.getAdditionalRenderState().setWireframe(true);
        mat.setColor("Color", color);
        g.setMaterial(mat);
        rootNode.attachChild(g);
        return g;
    }

    public void notifyTargetFound() {
        numVehiclesAtTarget++;
    }
    
    public CommMedium getCommMedium() {
        return commMedium;
    }
    
//    @Override
//    public void destroy() {
//        super.destroy();
//        executor.shutdown();
//    }
    
    public ScheduledThreadPoolExecutor getExecutor() {
        return executor;
    }
    
    public HashMap<String, UWVehicleControl> getUWVehiclesMap() {
        return vehControlsMap;
    }
    
//    public HashMap<String, AdaptivePSOControl> getUWVehiclesMap() {
//        return vehControlsMap;
//    }
    
    public static int getNumAgents() {
        return NUM_AGENTS;
    }
    
    public String[] getVehicleNames() {
        return vehicleNames;
    }
    
    public void reportProcessedTargets(ArrayList<String> processedTargets) {    
        for(String target : processedTargets) {
            targetProcessings.put(target, targetProcessings.get(target) + 1);
        }
        
        float totalProcTargetCount = 0;
        
        for(Entry<String, Integer> entry : targetProcessings.entrySet()) {
            if(entry.getValue() > 0) {
                totalProcTargetCount++;
            }
        }
        
        if(totalProcTargetCount != 0) {
            
            float fract = totalProcTargetCount/totalTargetsNum;
            float percent = FastMath.floor(fract * 100);
            float modulo = percent % 5;

            if( modulo == 0 && percent != 0) {
                int ind = (int)(percent/5);
                if(percentagesFlags[ind-1] != true) {
                    printTimeToProcessPercent(percent);
                    percentagesFlags[ind-1] = true;
                }
            }
        }
        
        //if(!targ33PerentileFound) { printTimeToProcessPercent(33, totalProcTargetCount); }
        //if(!targ66PerentileFound) { printTimeToProcessPercent(66, totalProcTargetCount); }
        //if(!targ99PerentileFound) { printTimeToProcessPercent(99, totalProcTargetCount); }
        if(TASK_ALLOC_MODE_ENABLED && SHOW_PLOT) {
            trace.addPoint((getCurrentTick() - simStartTime), totalProcTargetCount);
        }
    }
    
    private void printTimeToProcessPercent(float percentTargsProcessed/*, float totalProcTargetCount*/) {
        
        //if( (totalProcTargetCount/totalTargetsNum)*100 - percentTargsProcessed > 0 ) {
            double timeToProcess = getCurrentTick() - simStartTime;
            //System.out.println("Time to process "+percentTargsProcessed+"% of targets: "+timeToProcess+" seconds");
            System.out.println(timeToProcess+" , "+percentTargsProcessed+"%");
            
            // In task allocation mode only ... forces simulation to stop once 100% target processing is reached
            if( getStateManager().getState(TaskAllocMode.class) != null &&
                    sim.getStateManager().getState(TaskAllocMode.class).isEnabled() && 
                    percentTargsProcessed == 100f
              ) {  
                simulationTicks = MAX_SIMULATION_TICK_COUNT - 1;
            }
            
//            switch(percentTargsProcessed) {
//                case 33:
//                    targ33PerentileFound = true;
//                    break;
//                case 66:
//                    targ66PerentileFound = true;
//                    break;
//                case 99:
//                    targ99PerentileFound = true;
//                    break;
//                default:
//                    break;
//            }
        //}
    }
    
    public void setNumTargets(int numTargets) {
        totalTargetsNum = numTargets;
    }
    
    public void testCSG() {
        
//        Material mat_csg = assetManager.loadMaterial("Common/Materials/RedColor.j3m");
//        mat_csg.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
//
//        CSGNode csg = new CSGNode();
//        csg.setMaterial(mat_csg);
//        
//        Spatial s = assetManager.loadModel("Models/Agent/agm65/AUV.j3o");
//        
//        ArrayList<Geometry> g = new ArrayList<Geometry>();
//        
//        Box box0 = new Box(128,128,25);
//        Geometry boxy = new Geometry("boxy", box0);
//        
//        g.add(boxy);
//        
//        GeometryBatchFactory.gatherGeoms(s, g);
//        Mesh m = new Mesh();
//        GeometryBatchFactory.mergeGeometries(g, m);
//        MeshBrush mb = new MeshBrush(m);
//        
//        csg.addBrush(mb);
//        CubeBrush base = new CubeBrush(new Vector3f(0f, 0.5f, 0f), new Vector3f(1f, 0.1f, 1f));
//        base.setType(BrushType.SUBTRACTIVE);
//
//        csg.addBrush(base);
//        csg.regenerate();
//        csg.move(0f, 1f, 0f);
//
//        rootNode.attachChild(csg);
        
        
//        Material mat_csg = assetManager.loadMaterial("Common/Materials/RedColor.j3m");
//        mat_csg.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
//
//        CSGNode csg = new CSGNode();
//        csg.setMaterial(mat_csg);
//
//        CubeBrush base = new CubeBrush(new Vector3f(0f, 50f, 0f), new Vector3f(1f, 1f, 1f));
//        csg.addBrush(base);
//
//        SphereBrush hole = new SphereBrush(new Vector3f(0f, 50f, 0f), 1.3f, 16, 8);
//        hole.setType(BrushType.SUBTRACTIVE);
//
//        csg.addBrush(hole);
//        csg.regenerate();
//        csg.move(0f, 70f, 0f);
//
//        rootNode.attachChild(csg);
    }
   
    public BulletAppState getBulletAppState() {
        return bulletAppState;
    }
    
    public SearchAlgorithms[] getAvailableSearchAlgList() {
        return UWVehicleControl.getAvailableSearchAlgorithms();
    }
    
    public TaskAllocAlgorithms[] getAvailableTaskAllocAlgList() {
        return UWVehicleControl.getAvailableTaskAllocAlgorithms();
    }
    
    public IntegrationAlgorithms[] getAvailableIntegAlgList() {
        return UWVehicleControl.getAvailableIntegAlgorithms();
    }
    
    public static void setNumberOfAgents(int number) {
        NUM_AGENTS = number;
    }
    
    public static float getTargetNeighborhoodRadius() {
        return TARGET_NEIGHBORHOOD_RADIUS;
    }

    @Override
    public void destroy() {
        super.destroy();
        
        cleanup();
        
        if(simCore          != null) { simCore.cleanup();        }
        if(searchSim        != null) { searchSim.cleanup();      }
        if(taskAllocSim     != null) { taskAllocSim.cleanup();   }
        if(initSelfOrgSim   != null) { initSelfOrgSim.cleanup(); }
        if(generalTestsSim  != null) { generalTestsSim.cleanup();}
        if(integrationSim   != null) { integrationSim.cleanup(); }
        
        stopped = true;
    }
    
    private void cleanup() {
        avgDistToTargValues.clear();
        avgSpreadValues.clear();
        maxDistFromSourceValues.clear();
        maxDistFromTargValues.clear();
        numReachedTargValues.clear();
        numLostAgentsValues.clear();
        numSurvivingAgentsValues.clear();
        numSentMsgsValues.clear();
        numReceivedMsgsValues.clear();
    }
    
    public static boolean bulkSimActive() {
        return IN_BULK_SIM_RUN_MODE;
    }
    
    public int getNumSimRuns() {
        if(IN_BULK_SIM_RUN_MODE) {
           return numRuns; 
        }
        return -1;
    }
    
    public int getRunNumber() {
        if(IN_BULK_SIM_RUN_MODE) {
           return runNumber; 
        }
        return -1;
    }
    
    public void setTargsControlsMap(HashMap<String, RigidBodyControl> map) {
        targControlsMap = map;
    }
    
    public HashMap<String, RigidBodyControl> getTargsControlsMap() {
        return targControlsMap;
    }
    
    public int getMaxSimTickCount() {
        return MAX_SIMULATION_TICK_COUNT;
    }
    
    
    // Light (task indicator) sensing
    public HashMap<String, UWVehicleControl> getTargBeaconingAUVsMap() {
        return (HashMap)targetBeaconingAUVs;
    }
    
    public void addToTargBeaconingAUVsMap(UWVehicleControl auv) {
        targetBeaconingAUVs.put(auv.getVehicleName(), auv);
    }
    
    public void removeFromTargBeaconingAUVsMap(String auv) {
        targetBeaconingAUVs.remove(auv);
    }
    
    public void setTargetTypesMap(HashMap<String, TargetType> targetTypes) {
        targetTypesMap = targetTypes;
    }
    
    public TargetType getTargetType(String name) {
        if( targetTypesMap != null ) {
            return targetTypesMap.get(name);
        } else {
            System.out.println("Target types map not set!");
            return null;
        }
    }
    
    public void reportVehicleSurfaced(String auvName) {
        if(!recoveredVehicles.contains(auvName)) recoveredVehicles.add(auvName);
        System.out.println(auvName + " was recovered.");
        reportCompletedMission(auvName);
    }
    
    public void reportReleasedIntoWater(String auvName) {
        if(!releasedVehicles.contains(auvName)) releasedVehicles.add(auvName);
        System.out.println(auvName + " was released.");
        
        if(releasedVehicles.size() == NUM_AGENTS) {
            missionStartTick = getCurrentTick();
            System.out.println("Mission started: "+missionStartTick);
        }
    }
    
    public void reportCompletedMission(String auvName) {
        if(!doneVehicles.contains(auvName)) doneVehicles.add(auvName);
        
        //System.out.println(auvName + " is done.");
        
        if(doneVehicles.size() == numSurvivingAUVs) {
            missionEndTick = getCurrentTick();
            if( !printedMissionCompl ) { 
                System.out.println("Mission Completed: "+getCurrentTick());
                printedMissionCompl = true;
            }
            actualTimeSpentInMission = missionEndTick - missionStartTick;
            missionCompleted = true;
        }
    }
    
}

    //private float speed = 8;

    //private AgentBehavior agentController;
    //private AgentControl agentControl;
    //private RigidBodyControl agentControl;
    //private AgentController agentControl;

    //private Node agentNode;
    
    //private Node simNavigator;
    //private BetterCharacterControl simNavControl;
    
    //private CameraNode camNode;
    
    //private Vector3f navDirection = new Vector3f(0,0,0);
    //private Vector3f viewDirection = new Vector3f(0,0,1);
    //private Vector3f agentForwardDir;

    //private boolean rotateLeft = false, 
    //        rotateRight = false, 
    //        forward = false, 
    //        backward = false;

    //agentControl = new AgentController(bulletAppState.getPhysicsSpace());
    //agentControl = new RigidBodyControl();

    // Create an invisible entity to be used as a navigator to navigate 
    // though the simulation world. it would simply replace the camera and 
    // provide means for a solid object navigation (similar to a first-person 
    // in Games)
    //simNavigator = new Node("sim navigator");

    // Position it at the desired in initial location
    //simNavigator.setLocalTranslation(new Vector3f(0f,150f,350f));

    // Attach it to the rootNode
    //rootNode.attachChild(simNavigator);

    // Create a Control for the navigator
    //simNavControl = new BetterCharacterControl(1.5f, 4f, 30f);

    // Set its gravity
    //simNavControl.setGravity(new Vector3f(0f, -10f, 0f));

    // Add the Control to the simNavigator
    //simNavigator.addControl(simNavControl);

    // Register the Control to the Physics space
    //bulletAppState.getPhysicsSpace().add(simNavControl);

    // =====================================================================
    // LESSON: Don't add a PhysicsControl to the PhysicsSpace UNTIL you 
    //         attach bulletAppState to the stateManager
    // =====================================================================

    // Add agent's controller to the Physics space
    //bulletAppState.getPhysicsSpace().add(agentController);
    //bulletAppState.getPhysicsSpace().add(agentControl);

    // Register agentController with the PhysicsSpace
    //bulletAppState.getPhysicsSpace().addTickListener(agentController);

    //waterProcessor.getMaterial().getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);

    // Overwrite WASD deafult key mappings
    //inputManager.addMapping("Forward", new KeyTrigger(KeyInput.KEY_W));
    //inputManager.addMapping("Back", new KeyTrigger(KeyInput.KEY_S));
    //inputManager.addMapping("Rotate Left", new KeyTrigger(KeyInput.KEY_A));
    //inputManager.addMapping("Rotate Right", new KeyTrigger(KeyInput.KEY_D));

    //inputManager.addListener(this, "Rotate Left", "Rotate Right");
    //inputManager.addListener(this, "Forward", "Back");

    // Create a BetterCharacterControl object to control the behavior of agent
    // and access its physics properties (The physical properties of a 
    // spatial are stored in its physics control)
    //agentController = new AgentBehavior(agent, 200);

    //agentControl = new AgentControl();
    //agentControl = new RigidBodyControl(50);

    //agentControl.setKinematic(true);
    //agentController.setKinematic(true);

    //agentController.setLinearVelocity(new Vector3f(5f, 0f, 5f));

    // Add it as controller for agent
    //agent.addControl(agentController);

    //agent.getControl(AgentController.class).setPhysicsAgent(physicsAgent);
    //agent.getControl(AgentController.class).setState(State.move);      

    // BoundingBox bounds = (BoundingBox)entityModel.getWorldBound();

    //TODO: add update code

    // Create a Node to hold the camera and enable its control!
    //camNode = new CameraNode("CamNode", cam);

    //  
    //camNode.setControlDir(CameraControl.ControlDirection.SpatialToCamera);

    //
    //camNode.setLocalTranslation(new Vector3f(0f, 150f, -350f));

    // 
    //Quaternion quat = new Quaternion();

    // 
    //quat.lookAt(Vector3f.UNIT_Z, Vector3f.UNIT_Y);

    // 
    //camNode.setLocalRotation(quat);

    // 
    //simNavigator.attachChild(camNode);

    // Enable the new camera 
    //camNode.setEnabled(true);

    // Disable flyCam
    //flyCam.setEnabled(false);

    // Get current forward and left vectors of the simNavNode
    //Vector3f modelForwardDir = simNavigator.getWorldRotation().mult(Vector3f.UNIT_Z);
    //Vector3f modelLeftDir = simNavigator.getWorldRotation().mult(Vector3f.UNIT_X);

    // Determine change in direction
//        navDirection.set(0, 0, 0);
//        if(forward) {
//            navDirection.addLocal(modelForwardDir.mult(speed));
//        } else if(backward) {
//            navDirection.addLocal(modelForwardDir.mult(speed).negate()); 
//        }

    // Walk
    //simNavControl.setWalkDirection(navDirection);

    // Determine the change in rotation
//        if(rotateLeft) {
//            Quaternion rotateL = new Quaternion().fromAngleAxis(FastMath.PI * tpf, Vector3f.UNIT_Z);
//            rotateL.mult(viewDirection);
//        } else if (rotateRight) {
//            Quaternion rotateR = new Quaternion().fromAngleAxis(-FastMath.PI * tpf, Vector3f.UNIT_Y);
//            rotateR.mult(viewDirection);
//        }

    // Turn
    //simNavControl.setViewDirection(viewDirection);

    //agentController.setLinearVelocity(new Vector3f(5f, 0f, 5f));

    //agent.setLocalTranslation(new Vector3f(5f, 0f, 5f));
    //agentController.setPhysicsLocation(new Vector3f(50f, -1f, 50f));
    //agent.move(new Vector3f(10f, 0f, 10f));

    //    public void onAction(String binding, boolean isPressed, float tpf) {
    //        
    //        if(binding.equals("Rotate Left")) {
    //            rotateLeft = isPressed;
    //        } else if(binding.equals("Rotate Right")) {
    //            rotateRight = isPressed;
    //        } else if(binding.equals("Forward")) {
    //            forward = isPressed;
    //        } else if(binding.equals("Back")) {
    //            backward = isPressed;
    //        }
    //        
    //    }