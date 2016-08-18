package swim.sim;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.input.FlyByCamera;
import com.jme3.input.InputManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import swim.algorithm.search.SearchAlgorithms;
import swim.sim.uwswarm.UWVehicleControl;
import swim.util.DebugUtils;

/**
 *
 * @author Sherif
 */
public class SearchMode extends AbstractAppState {
    
    // Essential App variables
    private SimpleApplication app;
    private Node              rootNode;
    private AssetManager      assetManager;
    private AppStateManager   stateManager;
    private InputManager      inputManager;
    private ViewPort          viewPort;
    private BulletAppState    physics;
    private FlyByCamera       flyCam;
    private Camera            cam;
    private BulletAppState    bulletAppState;
    
    // =========================================================================
    // Config
    // =========================================================================
    // Particles
    private static final Vector3f  PARTICLE_ORIGIN = new Vector3f(45, 6, 30);
    private final int NUM_PARTICLES = 1000; // was 1000
    // Ship wreck
    private static final Vector3f  SHIP_WRECK_LOC = PARTICLE_ORIGIN;
    
    private boolean TARGET_NEIGH_DEBUG_ENABLED = false,
                    VIEW_GRID = false;
    // =========================================================================
    // Config
    // =========================================================================
    
    // Search-specific variables
    
    // Ship wreck
    private Spatial shipWreckModel;
    private RigidBodyControl physicsShipWreck;
    
    // Particles
    private List<Spatial> particlesList = new ArrayList<Spatial>(NUM_PARTICLES);
    
    // Search
    private List<Spatial> targetList = new ArrayList<Spatial>();
    
    
    private boolean previouslyInitialized = false;
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {

        if( !previouslyInitialized ) {
        
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
            this.bulletAppState = ((Simulator)this.app).getBulletAppState();


            if(UWVehicleControl.getActiveSearchAlgorithm().index() == SearchAlgorithms.RPSO.index()) {
                addParticles();
            }
            addShipWreck(); // Must be added before agents
            if(VIEW_GRID) {
                drawGrid();
            }
        
            previouslyInitialized = true;
            
        }
        
    }
    
    @Override
    public void update(float tpf) {
        
    }
    
    public void drawGrid() {
        for(float x = -128; x <= 128; x+=4) {
           for(float z = -128; z <= 128; z+=4) {
                DebugUtils.plotCrossHair(new Vector3f(x, 0.0f, z), 2, 0.5f, ColorRGBA.Red, (Simulator)app);
           }     
        }
    }
    
    private void addParticles() {
        
        Random rng = new Random();
        
        Vector3f particlePos;
        
        float G_x, G_y, G_z;
        
        Spatial particleModel;
        RigidBodyControl physicsParticle;
        
        for(int i = 0; i < NUM_PARTICLES; i++) {
            
            G_x = (float)( Math.random() >= 0.5 ? rng.nextGaussian() : - rng.nextGaussian() );
            G_y = (float)( Math.random() >= 0.5 ? rng.nextGaussian() : - rng.nextGaussian() );        
            G_z = (float)( Math.random() >= 0.5 ? rng.nextGaussian() : - rng.nextGaussian() ); 

            particlePos = new Vector3f(G_x, G_y, G_z);
            particlePos = particlePos.mult(32);
            particlePos = particlePos.add(PARTICLE_ORIGIN);
            
            if( particlePos.y < 0    || particlePos.y > 20 ||
                particlePos.x < -128 || particlePos.x > 128 ||
                particlePos.z < -128 || particlePos.z > 128) {
                continue;
            }
            
            //@TODO: Add keep in bounds function to avoid having particle overflow outside PhysicsSpace universe
            
            //System.out.println("Location: " + particlePos.toString());
        
            // Original
            ////////////////////////////////////////////////////////////////////
            // Load particle model
            particleModel = assetManager.loadModel("Models/particles/particle.j3o");
            ////////////////////////////////////////////////////////////////////
            
            // =================================================================
            // Create a simpler model
//            Node particleNode = new Node();
//            Mesh mesh = new Mesh();
//            
//            Vector3f [] vertices = new Vector3f[4];
//            vertices[0] = new Vector3f(0,0,0);
//            vertices[1] = new Vector3f(3,0,0);
//            vertices[2] = new Vector3f(0,3,0);
//            
//            Vector2f[] texCoord = new Vector2f[4];
//            texCoord[0] = new Vector2f(0,0);
//            texCoord[1] = new Vector2f(1,0);
//            texCoord[2] = new Vector2f(0,1);
//            
//            int [] indexes = { 2,0,1 };
//            
//            mesh.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
//            mesh.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
//            mesh.setBuffer(Type.Index,    3, BufferUtils.createIntBuffer(indexes));
//            mesh.updateBound();
//            
//            Geometry geo = new Geometry("ParticleMesh", mesh); // using our custom mesh object
//            Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
//            mat.setColor("Color", ColorRGBA.Yellow);
//            
//            geo.setMaterial(mat);
//            geo.setCullHint(Spatial.CullHint.Never);
//            particleNode.attachChild(geo);
//            particleModel = particleNode;
            // =================================================================
            
            particleModel.setName("particle_" + i);
            
            particleModel.scale(0.5f);

            //Material mat_default = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            //mat_default.setColor("Color", ColorRGBA.Yellow);
            //particleModel.setMaterial(mat_default);

            physicsParticle = new RigidBodyControl(0f);
            
            particleModel.addControl(physicsParticle);

            physicsParticle.setPhysicsLocation(particlePos);
            
            rootNode.attachChild(particleModel);
            
            particlesList.add(particleModel);
            
            bulletAppState.getPhysicsSpace().add(physicsParticle);
            //simCore.addToPhysicsSpace(physicsParticle);
        
        }
    }
    
    private void addShipWreck() {
        // Load agent and attach to the node
        shipWreckModel = assetManager.loadModel("Models/Ships/ship4/submarin.j3o");
        shipWreckModel.setName("ShipWreck");

        //Material mat_default = new Material(assetManager, "Common/MatDefs/Misc/ShowNormals.j3md");

        //shipWreckModel.setMaterial(mat_default);


        shipWreckModel.scale(3f);

        physicsShipWreck = new RigidBodyControl(0f);

        Quaternion rotY = new Quaternion();
        rotY.fromAngleAxis(FastMath.DEG_TO_RAD * 30, Vector3f.UNIT_X);

        shipWreckModel.addControl(physicsShipWreck);

        physicsShipWreck.setPhysicsRotation(rotY);
        physicsShipWreck.setPhysicsLocation(SHIP_WRECK_LOC);



        rootNode.attachChild(shipWreckModel);
        bulletAppState.getPhysicsSpace().add(physicsShipWreck);
        //simCore.addToPhysicsSpace(physicsShipWreck);

        targetList.add(shipWreckModel);

        if(TARGET_NEIGH_DEBUG_ENABLED) {
            float height = 2*(12.5f + getTargetLocation().y);         
            Cylinder cylinder = new Cylinder(50, 50, Simulator.getTargetNeighborhoodRadius(), height);
            Geometry targNeighborhood = new Geometry("neighborhood", cylinder);
            targNeighborhood.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
            Material material2 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            material2.setColor("Color", new ColorRGBA(0.0f,0.25f,1f,0.45f));
            material2.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
            material2.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
            targNeighborhood.setLocalTranslation(getTargetLocation());
            targNeighborhood.setMaterial(material2);
            targNeighborhood.setQueueBucket(RenderQueue.Bucket.Transparent);
            rootNode.attachChild(targNeighborhood);
        }
       
    } 
    
    public static Vector3f getTargetLocation() {
        return SHIP_WRECK_LOC;
    }
    
    public List<Spatial> getTargetList() {
        return targetList;
    }
    
    public List<Spatial> getParticlesList() {
        return particlesList;
    }
    
}
