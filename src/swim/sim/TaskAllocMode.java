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
import swim.algorithm.taskalloc.TargetType;

/**
 *
 * @author Sherif
 */
public class TaskAllocMode extends AbstractAppState {
    
    // =========================================================================
    // Config
    // =========================================================================
    private boolean USE_SINGLE_BARREL_GROUP = false,
                    USE_MULTIPLE_BARREL_GROUPS = true,
                    USE_SHIP = false,
                    USE_INTERMIXED_TARGETS = false,
                    TWO_TARGET_GROUPS = false,
                    THREE_TARGET_GROUPS = !TWO_TARGET_GROUPS;
    
    private final float BARREL_MASS = 100f;
    
    private final float TRIANGLE_SIDE = 70,
                        HALF_TRIANGLE_SIDE = TRIANGLE_SIDE/2,
                        TRIANGLE_HEIGHT = (FastMath.sqrt(3)/2) * TRIANGLE_SIDE;
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
    
    // Barrel placement variables
    private float setContainerVolume;
    private ArrayList<Vector3f> inContainerBarrelPositions;
    private Vector3f inContainerBarrelPos;
    private float boxSideLength;
    
    // Barrels
    private Spatial barrelModel;
    private RigidBodyControl physicsBarrel;
    private List<Spatial> targetList = new ArrayList<Spatial>();
    
    private HashMap<String, TargetType> targetTypes;
    
    // Ship wreck
    private final Vector3f  SHIP_WRECK_LOC = new Vector3f(45, 6, 30);
    private Spatial shipWreckModel;
    private RigidBodyControl physicsShipWreck;
    
    private boolean previouslyInitialized = false;
    
    // =========================================================================
    
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

            targetTypes = new HashMap<String, TargetType>();
            int numTargs = 1;

            if( USE_SHIP ) {
                addShipWreck(); // Must be added before agents

            } else if( USE_SINGLE_BARREL_GROUP || USE_MULTIPLE_BARREL_GROUPS ) {

                int[] targetCounts = new int[]{10};;

                if( USE_SINGLE_BARREL_GROUP ) {

                    addBarrels(
                        targetCounts, 
                        new Vector3f[]{ SHIP_WRECK_LOC });

                    numTargs = targetCounts[0];

                } else {

                    if( THREE_TARGET_GROUPS ) {
                        targetCounts = new int[]{10, 10, 10};
                    addBarrels(
                        targetCounts, 
                          new Vector3f[]{ new Vector3f(HALF_TRIANGLE_SIDE, 20f, TRIANGLE_HEIGHT/3),
                                          new Vector3f(-HALF_TRIANGLE_SIDE, 20f, TRIANGLE_HEIGHT/3),
                                          new Vector3f(0f, 20f, -2*TRIANGLE_HEIGHT/3)});
//                        new Vector3f[]{ new Vector3f(35f, 20f, 35f),
//                                        new Vector3f(-35f, 20f, 35f),
//                                        new Vector3f(0f, 20f, -40.414518843273803515640414635137f)});   //z = x/cos30                                  
                    }
                    
                    if( TWO_TARGET_GROUPS ) {
                        targetCounts = new int[]{10, 10};
                        addBarrels(
                        targetCounts, 
                        new Vector3f[]{ new Vector3f(50f, 20f, 0f),
                                        new Vector3f(-50f, 20f, 0f)});
                    }

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
        
        // Loop over barrel sets
        for(int i = 0; i < setSizes.length; i++) {
            
            setContainerVolume = barrelVolume * setSizes[i];
            inContainerBarrelPositions = new ArrayList<Vector3f>(setSizes[i]);

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
                
                
                boxSideLength = (float) Math.cbrt(setContainerVolume);
                boxSideLength += 0.2 * boxSideLength;

                do {
                    inContainerBarrelPos = generateInBoxBarrelPos(); 
                    inContainerBarrelPos = setPositions[i].add(inContainerBarrelPos);
                } while(inContainerBarrelPositions.contains(inContainerBarrelPos));

                inContainerBarrelPositions.add(inContainerBarrelPos);
                
                physicsBarrel.setPhysicsLocation(inContainerBarrelPos);
                
                rootNode.attachChild(barrelModel);

                bulletAppState.getPhysicsSpace().add(physicsBarrel);
                
                targetList.add(barrelModel);

                k++;
                
            }         
        }
        
//        for (Spatial barrelsList1 : targetList) {
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
    
}
