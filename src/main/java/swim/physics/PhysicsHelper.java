package swim.physics;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.material.RenderState.FaceCullMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Sphere;
import com.jme3.texture.Texture;
import com.jme3.util.SkyFactory;
import com.jme3.water.SimpleWaterProcessor;
import swim.sim.SearchMode;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public class PhysicsHelper {
    
    // Config
    private static boolean DISPLAY_TERRAIN_TEXTURE = false;
    
    private static boolean COMPLEX_ENVIRONMENT;
    
    private static RigidBodyControl terrainPhysics = null;
    private static Node rootNode = null;
    /**
     * Create the basic physics world used in the simulation
     * @param rootNode
     * @param assetManager
     * @param physicsSpace
     * @param sim
     * @return 
     */
    public static Spatial createPhysicsWorld(Node rootNode, AssetManager assetManager, PhysicsSpace physicsSpace, Simulator sim) {
        
        COMPLEX_ENVIRONMENT = Simulator.USE_COMPLEX_ENVIRONMENT;
        
        PhysicsHelper.rootNode = rootNode;
        
        // Create an ambient light
        AmbientLight dayLight = new AmbientLight();
        
        // Attach it to the rootNode
        rootNode.addLight(dayLight);
        
        // Create a directional light
        DirectionalLight sun = new DirectionalLight();
        
        // Set its location
        sun.setDirection(new Vector3f(-0.57735026f, -0.57735026f, -0.57735026f));
        
        // Attach it to the rootNode
        rootNode.addLight(sun);
        
        // Load terrain
        Spatial terrain = null;
        
        if( COMPLEX_ENVIRONMENT ) {
            // Original Terrain (peaky)
            terrain = assetManager.loadModel("Scenes/terrain.j3o");
        } else {
            if( DISPLAY_TERRAIN_TEXTURE ) {
                // Terrain with texture
                terrain = assetManager.loadModel("Scenes/testPlatform.j3o");
            } else {
                // Plain terrain (blue)
                terrain = assetManager.loadModel("Scenes/testPlatformPlain.j3o");
            }
        }
        
        // Set some properties
        terrain.setName("Terrain");
        terrain.setShadowMode(ShadowMode.Receive);
        
        // Add RigidBodyControl to the terrain to enable Physics rules
        // and set its mass to zero to prevent gravity effect on the terrain
        // itself
        terrainPhysics = new RigidBodyControl(0f);
        
        // Set terrainPhysics as the terrain's Control
        terrain.addControl(terrainPhysics);
        
        // Set some properties
        terrainPhysics.setKinematic(true);
        
        // Attach it to the scene graph's rootNode
        rootNode.attachChild(terrain);
        
        // Add universePhysics to the PhysicsSpace as well
        physicsSpace.add(terrain);
        
        //return terrain;
        
        if( COMPLEX_ENVIRONMENT ) {
            
            // advanced water  
            Quad quad = new Quad(256,256);
            quad.scaleTextureCoordinates(new Vector2f(6f,6f));
            // we create the water geometry from the quad
             Box box1 = new Box(128,128,70);
            //Geometry waterPlane=new Geometry("water", quad);
            Geometry waterPlane=new Geometry("water", box1);
            waterPlane.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
            waterPlane.setLocalTranslation(0, 0, 0);
            Material material = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            //material.setTransparent(true);
            material.setColor("Color", new ColorRGBA(1,1,1,0.01f));
            material.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
            material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Front);
            waterPlane.setMaterial(material);

            //waterPlane.addControl(new RigidBodyControl(0f));
            rootNode.attachChild(waterPlane);
            physicsSpace.add(waterPlane);
            
        } else {
            // =====================================================================        
            // Fake water 
    //        Quad quad2 = new Quad(256,256);
    //        quad2.scaleTextureCoordinates(new Vector2f(6f,6f));
    //        Geometry water = new Geometry("water", quad2);
    //        water.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
    //        water.setLocalTranslation(-128, 25, 128);

            Box box1 = new Box(128,128,12.5f);
            Geometry water = new Geometry("water", box1);
            water.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
            water.setLocalTranslation(0, 12f, 0);

            water.setShadowMode(ShadowMode.Receive);
            Material material2 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            material2.setColor("Color", new ColorRGBA(0.0f,0.25f,1f,0.45f));
            material2.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
            material2.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
            water.setQueueBucket(Bucket.Transparent);
            water.setMaterial(material2);
            rootNode.attachChild(water);

            Box box2 = new Box(128,128,12.5f);
            Geometry seaFloor = new Geometry("water", box2);
            seaFloor.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
            seaFloor.setLocalTranslation(0, -12.75f, 0);

            seaFloor.setShadowMode(ShadowMode.Receive);

            Material material3 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");

            if( DISPLAY_TERRAIN_TEXTURE ) {
                // With texture ...
                Texture  cube1Tex = assetManager.loadTexture("Textures/dirt_2.jpg");
                material3.setTexture("ColorMap", cube1Tex);
            } else {
                // Plane ...
                material3.setColor("Color", new ColorRGBA(0.95f,0.73f,0.65f,1f));
                material3.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
                material3.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
            }

            seaFloor.setQueueBucket(Bucket.Transparent);
            seaFloor.setMaterial(material3);
            rootNode.attachChild(seaFloor);
            // =====================================================================            
        }
        
        // Create an unshaded material for the floor
        //Material material = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        
        // Set its texture
        //material.setTexture("ColorMap", assetManager.loadTexture("Textures/dirt.jpg"));
        
        // Create a floor Shape
        //Box floorBox = new Box(128f, 0.25f, 128f);
        
        // Create a floor Geometry and set its Shape
        //Geometry floorGeom = new Geometry("Floor", floorBox);
        
        // Set the material of the floor
        //floorGeom.setMaterial(material);
        
        // Move the floor 5 wu's down
        //floorGeom.setLocalTranslation(0, -5, 0);
        
        // Add RigidBodyControl to the floor to enable Physics rules
        // and set its mass to zero to prevent gravity effect on the floor
        // itself
        //floorGeom.addControl(new RigidBodyControl(0));
        
        // Attach the floor to the scene graph's rootNode
        //rootNode.attachChild(floorGeom);
        
        // Add the floor to the PhysicsSpace as well
        //space.add(floorGeom);

        return terrain;
    }
    
    public static RigidBodyControl getTerrainPhysics() {
        return terrainPhysics;
    }
    
    private static void createSimpleWater(Simulator sim, Spatial terrain) {
        // we create a water processor
        SimpleWaterProcessor waterProcessor = new SimpleWaterProcessor(sim.getAssetManager());
        waterProcessor.setReflectionScene(terrain);

        // we set the water plane
        Vector3f waterLocation=new Vector3f(0,-6,0);
        waterProcessor.setPlane(new Plane(Vector3f.UNIT_Y, waterLocation.dot(Vector3f.UNIT_Y)));
        sim.getViewPort().addProcessor(waterProcessor);

        // we set wave properties
        waterProcessor.setWaterDepth(5);         // transparency of water
        waterProcessor.setDistortionScale(0.05f); // strength of waves
        waterProcessor.setWaveSpeed(0.05f);       // speed of waves

        // we define the wave size by setting the size of the texture coordinates
        Quad quad2 = new Quad(256,256);
        quad2.scaleTextureCoordinates(new Vector2f(6f,6f));

        // we create the water geometry from the quad
        Geometry water=new Geometry("water", quad2);
        water.setLocalRotation(new Quaternion().fromAngleAxis(-FastMath.HALF_PI, Vector3f.UNIT_X));
        water.setLocalTranslation(-128, 25, 128);
        water.setShadowMode(ShadowMode.Receive);
        water.setMaterial(waterProcessor.getMaterial());
        rootNode.attachChild(water);
    }
}
