/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.audio.AudioNode;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.input.FlyByCamera;
import com.jme3.input.InputManager;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.water.WaterFilter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import swim.core.Agent;
import swim.core.CommMedium;
import swim.physics.PhysicsHelper;
import swim.sim.uwswarm.Swarms;
import swim.sim.uwswarm.UWVehicle;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class SimulationCore extends AbstractAppState {
   
    private SimpleApplication app;
    private Node              rootNode;
    private AssetManager      assetManager;
    private AppStateManager   stateManager;
    private InputManager      inputManager;
    private ViewPort          viewPort;
    private BulletAppState    physics;
    private FlyByCamera       flyCam;
    private Camera            cam;
    
    // =========================================================================
    // Config
    // =========================================================================
    // Robots
    private static final int NUM_AGENTS = 20;

    // Physics
    private final boolean   DEBUG_ENABLED = false; 
    
    // =========================================================================
    // Config
    // =========================================================================
    
    // Physics
    private BulletAppState bulletAppState;
    
    // Terrain
    private Spatial terrain = null;
    private RigidBodyControl terrainPhysics = null;
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {

        super.initialize(stateManager, app);
        this.app = (SimpleApplication) app;

        this.rootNode     = this.app.getRootNode();
        this.assetManager = this.app.getAssetManager();
        this.stateManager = this.app.getStateManager();
        this.inputManager = this.app.getInputManager();
        this.viewPort     = this.app.getViewPort();
        this.physics      = this.stateManager.getState(BulletAppState.class);
        this.cam          = this.app.getCamera();
        this.flyCam       = this.app.getFlyByCamera();

        initSimulation();
        initPhysics(); 
        setupCamera();

    }
  
    private void initSimulation() {

        viewPort.setBackgroundColor(ColorRGBA.White); //new ColorRGBA(0.17f, 0.6f, 1.0f, 1.0f)

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
        terrain = PhysicsHelper.createPhysicsWorld(rootNode, assetManager, bulletAppState.getPhysicsSpace(), (Simulator)app);
        
        terrainPhysics = PhysicsHelper.getTerrainPhysics();
              
    }
  
    private void setupCamera() {
        
        flyCam.setMoveSpeed(100); 
     
        // Create a Quaternion to use in rotating the camera
        Quaternion roll320 = new Quaternion();
        
        // Roll it around x-axis 45 degrees
        roll320.fromAngleAxis( 320*FastMath.DEG_TO_RAD , Vector3f.UNIT_X );
        
        // Get current camera rotation
        Quaternion qRot = cam.getRotation();
        
        // Calculate new rotation
        Quaternion newCamRot = roll320.mult(qRot);
        
        // Rotate the camera
        cam.setRotation(newCamRot);
        
        // Change camera location
        cam.setLocation(new Vector3f(0f,200f,260f));
        
    }
    
    public void addToPhysicsSpace(Control control) {
        bulletAppState.getPhysicsSpace().add(control);
    }
    
    public BulletAppState getBulletAppState() {
        return bulletAppState;
    }
    
    public Spatial getTerrain() {
        return terrain;
    }
    
    public RigidBodyControl getTerrainPhysics() {
        return terrainPhysics;
    }
    
}
