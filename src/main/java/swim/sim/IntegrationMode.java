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
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import swim.algorithm.common.TargetConfig;
import swim.algorithm.search.SearchAlgorithms;
import swim.algorithm.taskalloc.TargetType;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class IntegrationMode extends AbstractAppState {
    
    // =========================================================================
    // Config
    // =========================================================================
    private boolean USE_SINGLE_BARREL_GROUP = false,
                    USE_MULTIPLE_BARREL_GROUPS = !USE_SINGLE_BARREL_GROUP,
                    USE_SHIP = true,
                    USE_INTERMIXED_TARGETS = false,
                    USE_RAND_BARREL_LOCATIONS = false,
                    USE_RAND_SHIP_LOCATION = false;
    
    // Ship wreck
    private Vector3f  SHIP_WRECK_LOC = new Vector3f(45, 6, 30);
    
    private final float BARREL_MASS = 100f,
                        TEST_PLATFORM_SIDE_LENGTH = 256,
                        BARREL_DROP_HEIGHT = 20;
    
    private final float TRIANGLE_SIDE = 70,
                        HALF_TRIANGLE_SIDE = TRIANGLE_SIDE/2,
                        TRIANGLE_HEIGHT = (FastMath.sqrt(3)/2) * TRIANGLE_SIDE;
    
    private final int GROUP_COUNT = 40,
                      NUM_BARREL_GROUPS = 3;
    
    // Particles
    private Vector3f  PARTICLE_ORIGIN = SHIP_WRECK_LOC;
    private final int NUM_PARTICLES = 1000; // was 1000
    // =========================================================================
    // Config
    // =========================================================================
    
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
    
    // Search
    private List<Spatial> targetList = new ArrayList<Spatial>();
    private Spatial shipWreckModel;
    private RigidBodyControl physicsShipWreck;
    
    // Particles
    private List<Spatial> particlesList = new ArrayList<Spatial>(NUM_PARTICLES);
    
    // Control variables
    private boolean componentsAdded = false,
                    previouslyInitialized = false;
    
    // Barrel placement variables
    private float setContainerVolume;
    private ArrayList<Vector3f> inContainerBarrelPositions;
    private Vector3f inContainerBarrelPos;
    private float boxSideLength;
    
    // Barrels
    private Spatial barrelModel;
    private RigidBodyControl physicsBarrel;
    private HashMap<String, RigidBodyControl> targControlsMap;
    
    private HashMap<String, TargetType> targetTypes;
    
    // =========================================================================
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {

        if( !previouslyInitialized ) {
            super.initialize(stateManager, app);
            this.app = (SimpleApplication) app;

            //System.out.println("Integration mode initialized ...");

            this.rootNode     = this.app.getRootNode();
            this.assetManager = this.app.getAssetManager();
            this.stateManager = this.app.getStateManager();
            this.inputManager = this.app.getInputManager();
            this.viewPort     = this.app.getViewPort();
            this.physics      = this.stateManager.getState(BulletAppState.class);
            this.cam          = this.app.getCamera();
            this.flyCam       = this.app.getFlyByCamera();
            this.bulletAppState = ((Simulator)this.app).getBulletAppState();

            targetTypes = new HashMap<String, TargetType>();
            int numTargs = 1;

            if( USE_SHIP ) {
                
                if( UWVehicleControl.getActiveSearchAlgorithm().equals(SearchAlgorithms.RPSO) ) {
                    addParticles();
                }
                
                addShipWreck(); // Must be added before agents

            } else if( USE_SINGLE_BARREL_GROUP || USE_MULTIPLE_BARREL_GROUPS ) {

                int[] targetCounts = new int[]{GROUP_COUNT};;

                if( USE_SINGLE_BARREL_GROUP ) {

                    addBarrels(
                        targetCounts, 
                        new Vector3f[]{ SHIP_WRECK_LOC });

                    numTargs = targetCounts[0];

                } else {

                    targetCounts = new int[]{GROUP_COUNT, GROUP_COUNT, GROUP_COUNT};
                    
                    addBarrels(
                        targetCounts, 
                          new Vector3f[]{ new Vector3f(HALF_TRIANGLE_SIDE, 20f, TRIANGLE_HEIGHT/3),
                                          new Vector3f(-HALF_TRIANGLE_SIDE, 20f, TRIANGLE_HEIGHT/3),
                                          new Vector3f(0f, 20f, -2*TRIANGLE_HEIGHT/3)});
//                        new Vector3f[]{ new Vector3f(35f, 20f, 35f),
//                                        new Vector3f(-35f, 20f, 35f),
//                                        new Vector3f(0f, 20f, -40.414518843273803515640414635137f)});   //z = x/cos30                                  

                    numTargs = 0;
                    for(int targCount : targetCounts) {
                        numTargs += targCount;
                    }            
                }
            }

            ((Simulator)app).setNumTargets(numTargs);   
            ((Simulator)app).setTargetTypesMap(targetTypes);
            
            previouslyInitialized = true;
        }
            
    }
    
    @Override
    public void update(float tpf) {
//        if(!componentsAdded && this.isEnabled()) {
//
//            componentsAdded = true;
//        }
        
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
            
            // Load particle model
            particleModel = assetManager.loadModel("Models/particles/particle.j3o");
            
            particleModel.setName("particle_" + i);
            
            particleModel.scale(0.5f);

            physicsParticle = new RigidBodyControl(0f);
            
            particleModel.addControl(physicsParticle);

            physicsParticle.setPhysicsLocation(particlePos);
            
            rootNode.attachChild(particleModel);
            
            particlesList.add(particleModel);
            
            bulletAppState.getPhysicsSpace().add(physicsParticle);
        
        }
    }
    
    private void addShipWreck() {
        // Load agent and attach to the node
       shipWreckModel = assetManager.loadModel("Models/Ships/ship4/submarin.j3o");
       shipWreckModel.setName("ShipWreck");
       
       //Material mat_default = new Material(assetManager, "Common/MatDefs/Misc/ShowNormals.j3md");
       //shipWreckModel.setMaterial(mat_default);
       
       targetTypes.put(shipWreckModel.getName(), TargetType.SHIP);
       
       shipWreckModel.scale(3f);
       
       physicsShipWreck = new RigidBodyControl(0f);

       Quaternion rotY = new Quaternion();
       rotY.fromAngleAxis(FastMath.DEG_TO_RAD * 30, Vector3f.UNIT_X);

       shipWreckModel.addControl(physicsShipWreck);
       
       physicsShipWreck.setPhysicsRotation(rotY);
       
       if( USE_RAND_SHIP_LOCATION ) {
           SHIP_WRECK_LOC = getRandShipLocation();
           System.out.println("Ship location: "+SHIP_WRECK_LOC);
       }
       
       physicsShipWreck.setPhysicsLocation(SHIP_WRECK_LOC);
       
       rootNode.attachChild(shipWreckModel);
       bulletAppState.getPhysicsSpace().add(physicsShipWreck);
       //simCore.addToPhysicsSpace(physicsShipWreck);
       
       targetList.add(shipWreckModel);
       
       
    }   
    
    private void addBarrels(int[] setSizes, Vector3f[] setPositions) {
        
        // Get and store barrel info
        barrelModel = assetManager.loadModel("Models/Targets/Boxes/barrel_metal.j3o");
        float barrelVolume = barrelModel.getWorldBound().getVolume(); 
        
        // Find total number of barrels
        int totalNumBarrels = 0;
        for(int k = 0; k < setSizes.length; k++) {
           totalNumBarrels +=  setSizes[k];
        }

        // A list that stores all barrels
        targetList = new ArrayList<Spatial>(totalNumBarrels);
        
        // Barrel name counter
        int k = 0;
        
        ColorRGBA[] barrelColors = new ColorRGBA[]{ColorRGBA.Red, ColorRGBA.Green, ColorRGBA.Blue};
        
        if( USE_RAND_BARREL_LOCATIONS ) {
            setPositions = new Vector3f[NUM_BARREL_GROUPS];
        }
        
        // Loop over barrel sets
        for(int i = 0; i < setSizes.length; i++) {
            
            setContainerVolume = barrelVolume * setSizes[i];
            inContainerBarrelPositions = new ArrayList<Vector3f>(setSizes[i]);
            
            boxSideLength = (float) Math.cbrt(setContainerVolume);
            boxSideLength += 0.2 * boxSideLength;
            
            
            if( USE_RAND_BARREL_LOCATIONS ) {
                    // Box centers should be "at most" at half the box side distance
                    // from any side of the test platform. To avoid barrels falling
                    // outside the platform when first dropped as a result of rolling,
                    // we leave a guard distance of 10 World Units (WUs)
                    
                    float coord, randCoord, sign;
                    setPositions[i] = new Vector3f();

                    for( int p = 0; p < 3; ++p ) {
                        if( p == 1 ) {
                           coord =  BARREL_DROP_HEIGHT;
                        } else {
                            randCoord = FastMath.nextRandomFloat() * ((TEST_PLATFORM_SIDE_LENGTH/2) - (boxSideLength/2) - 10);
                            sign = FastMath.nextRandomFloat() > 0.5f ? 1 : -1;
                            coord = sign * randCoord;
                        }
                        setPositions[i].set(p, coord);
                    }
                    System.out.println("Group "+i+" location: "+setPositions[i]);
                }

            for(int j = 0; j < setSizes[i]; j++) {
                
                physicsBarrel = new RigidBodyControl(BARREL_MASS);

                barrelModel = assetManager.loadModel("Models/Targets/Boxes/barrel_metal.j3o");
                barrelModel.setName("barrel_" + k);

                TargetType[] targTypes = TargetType.values();
                
                int selectedIndex;
                if( USE_INTERMIXED_TARGETS ) {
                    selectedIndex = FastMath.rand.nextInt(3);
                } else {
                    selectedIndex = i;
                }
                
                ColorRGBA selectedColor = barrelColors[selectedIndex];
                targetTypes.put(barrelModel.getName(), targTypes[selectedIndex]);

                barrelModel.scale(0.5f);
                barrelModel.addControl(physicsBarrel);
                
                Material barrelMaterial = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
                barrelMaterial.setBoolean("UseMaterialColors",true); 
                barrelMaterial.setColor("Diffuse", selectedColor);
                barrelMaterial.setColor("Specular",ColorRGBA.White); // for shininess
                barrelMaterial.setFloat("Shininess", 64f); // [1,128] for shininess
                barrelModel.setMaterial(barrelMaterial);

                do {
                    inContainerBarrelPos = generateInBoxBarrelPos();
                    inContainerBarrelPos = setPositions[i].add(inContainerBarrelPos);
                } while(inContainerBarrelPositions.contains(inContainerBarrelPos));


                physicsBarrel.setPhysicsLocation(inContainerBarrelPos);
                
                inContainerBarrelPositions.add(inContainerBarrelPos);
                
                rootNode.attachChild(barrelModel);

                bulletAppState.getPhysicsSpace().add(physicsBarrel);
                
                targetList.add(barrelModel);

                k++;
                
            }         
        }
        
//        for (Spatial barrelsList1 : barrelsList) {
//            System.out.print(barrelsList1 + ", ");
//        }
//        System.out.println();
    }
    
    private Vector3f generateInBoxBarrelPos() {
        float xPos = (float)( Math.random() * (Math.random() < 0.5 ? -boxSideLength : boxSideLength) );
        float yPos = (float)( Math.random() * (Math.random() < 0.5 ? -boxSideLength : boxSideLength) );
        float zPos = (float)( Math.random() * (Math.random() < 0.5 ? -boxSideLength : boxSideLength) );
        
        return new Vector3f(xPos, yPos, zPos);
    }
    
    public List<Spatial> getTargetList() {
        return targetList;
    }
    
    public boolean usingTargetConfiguration( TargetConfig config ) {
        switch(config) {
            case SINGLE_BARREL_GROUP:
                return USE_SINGLE_BARREL_GROUP;
            case MULTIPLE_BARREL_GROUPS:
                return USE_MULTIPLE_BARREL_GROUPS;
            case SHIP:
                return USE_SHIP;
        }
        return false;
    }
    
    private Vector3f getRandShipLocation() {
        Vector3f loc = new Vector3f();
        float coord, randCoord, sign;
        for( int p = 0; p < 3; ++p ) {
            if( p == 1 ) {
               coord =  6;
            } else {
                randCoord = FastMath.nextRandomFloat() * ((TEST_PLATFORM_SIDE_LENGTH/2) - (boxSideLength/2) - 10);
                sign = FastMath.nextRandomFloat() > 0.5f ? 1 : -1;
                coord = sign * randCoord;
            }
            loc.set(p, coord);
        }   
        return loc;
    }
    
    public List<Spatial> getParticlesList() {
        return particlesList;
    }
    
    // Add any required functions ...
    
}
