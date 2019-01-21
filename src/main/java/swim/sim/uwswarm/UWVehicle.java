package swim.sim.uwswarm;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingSphere;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import java.util.ArrayList;
import java.util.List;

import swim.core.Agent;
import swim.core.GenericModem;
import swim.core.Modem;
import swim.core.ParticleSensor;
import swim.core.Sensor;
import swim.core.State;
import swim.sim.Simulator;

/**
 * @author Sherif
 *
 */
public class UWVehicle extends Agent {
	
    // =========================================================================
    // Config
    // =========================================================================
    private final boolean   SIMPLE_VEHICLE_MODE = false;
    private static float    VEHICLE_VICINITY = 0.3f;  
    private final float     SENSING_RANGE = 5;
    private final float     COMM_RANGE = 10;
    protected static float  MAX_SPEED = 100f;
    private float           VEHICLE_MASS = 10f;
    private final float VEHICLE_SURFACE_AREA = 61.607304f; // (calculated using getVehicleSurfaceArea())
    // =========================================================================
    // Config
    // =========================================================================

    private float velFactor = 0.3f;
    
    // Vehicle controls
    protected UWVehicleControl vehicleControl;
    //protected AdaptivePSOControl vehicleControl;
    
    //protected GhostControl sonarSensor;
    
    protected ParticleSensor particleSensor;
    protected GenericModem modem;
    
    private Spatial vehicleModel;
    //private Node vehicleModel;
    private Spatial vehBody, 
            veh_FR_Thruster, veh_FL_Thruster, 
            veh_BR_Thruster, veh_BL_Thruster;
    
    
    private Spatial loadedModel;
    
    
    private BulletAppState bulletAppState;
    
    private Simulator sim;
    
    private float vehicleVolume;

    //private static float AVOID_WEIGHT = 0.2f;
    //private UWObstacles	obstacles;
    //private Terrain		terrain;
    //private Vector3f obstAvoidanceVel = new Vector3f();
    //private float currentFloorHeight;

    public UWVehicle(Vector3f vehPos, float velFactor, Node rootNode, BulletAppState bulletAppState, AssetManager assetManager, Simulator sim) {
        
        super(rootNode);
        
        this.bulletAppState = bulletAppState;
        this.sim = sim;
        
        this.velFactor = velFactor;

        // Set vehicle's maximum speed
        this.maxSpeed = this.velFactor * MAX_SPEED;
        
        // Load agent and attach to the node
        if( SIMPLE_VEHICLE_MODE ) {
            vehicleModel = assetManager.loadModel("Models/particles/particle.j3o");
        } else {
           vehicleModel = assetManager.loadModel("Models/Agent/agm65/AUV.j3o"); 
        }
        
        vehicleModel.scale(0.5f);
        
        vehicleVolume = vehicleModel.getWorldBound().getVolume();
        
        //getVehicleSurfaceArea();
        
        // ========================================================================================        
//        vehicleModel = new Node();
      
//        vehBody = assetManager.loadModel("Models/Agent/agm65/new_agent/Body.j3o");
//        vehBody.setName("body");
//        vehBody.rotate(-90 * FastMath.DEG_TO_RAD, 0, 0);
//        
//        veh_FR_Thruster = assetManager.loadModel("Models/Agent/agm65/new_agent/Vert_Thruster.j3o");
//        veh_FL_Thruster = assetManager.loadModel("Models/Agent/agm65/new_agent/Vert_Thruster.j3o");
//        veh_FR_Thruster.setName("frontRightThruster");
//        veh_FL_Thruster.setName("frontLeftThruster");
//        veh_FR_Thruster.setLocalTranslation(-1.5f, 0, 2.5f);
//        veh_FL_Thruster.setLocalTranslation(1.5f, 0, 2.5f);
//        
//        veh_BR_Thruster = assetManager.loadModel("Models/Agent/agm65/new_agent/Hor_Thruster.j3o");
//        veh_BL_Thruster = assetManager.loadModel("Models/Agent/agm65/new_agent/Hor_Thruster.j3o");
//        veh_BR_Thruster.setName("backRightThruster");
//        veh_BL_Thruster.setName("backLeftThruster");
//        veh_BR_Thruster.setLocalTranslation(-1.5f, 0, -3);
//        veh_BL_Thruster.setLocalTranslation(1.5f, 0, -3);
//        
//        vehicleModel.attachChild(vehBody);
//        vehicleModel.attachChild(veh_FR_Thruster);
//        vehicleModel.attachChild(veh_FL_Thruster);
//        vehicleModel.attachChild(veh_BR_Thruster);
//        vehicleModel.attachChild(veh_BL_Thruster);
//        
//        vehicleModel.scale(0.5f);
        // ========================================================================================
        
        
        //vehicleModel.setName("VehicleModel");        
        vehicleModel.scale(0.5f);
        setModel(vehicleModel);
        
        // Add vehicle control to the node
        
        // Default Control
        vehicleControl = new UWVehicleControl(this, bulletAppState, sim);
        
        // Adaptive PSO control
        //vehicleControl = new AdaptivePSOControl(this, bulletAppState, sim);
        
        addControl(vehicleControl);
        
        bulletAppState.getPhysicsSpace().add(vehicleModel); 

        // Store a reference to the Obstacles
        //this.obstacles = obstacles;

        // Store a reference to the Terrain
        //this.terrain = terrain;

        // Create a behavior for the vehicle
        //agentBeh = vehicleBeh;

        // Define vehicle's vicinity
        //vicinity = new BoundingSphere();
        //vicinity.setRadius(VEHICLE_VICINITY);

        
        
        // Add a range sensor       
        //particleSensor = new ParticleSensor(new SphereCollisionShape(), SENSING_RANGE, sim);
        //addSensor(particleSensor);
        
        // Add a modem       
        //modem = new GenericModem(new SphereCollisionShape(), COMM_RANGE);
        //addModem(modem);
        
        //vehicleControl.getPhysicsVehicle().setKinematic(false);

        //physicsVehicle.setPhysicsLocation(position);

        
        //physicsVehicle.setCollisionShape(CollisionShapeFactory.createBoxShape(vehModel));
        
        //initPos();
        
        //physicsVehicle.setPhysicsLocation(vehPos);    // was active
        
        //physicsVehicle.setInitLocation();
        
        this.heading = vehicleControl.getPhysicsVehicle().getPhysicsRotation();
        
        this.headingDirection = heading.mult(Vector3f.UNIT_Z);
        
        
        //addControl(vehicleControl);
        
        
        // Set initial vehicle state
        vehicleControl.setState(State.Drop);
        //vehicleControl.setState(State.Search);
        //vehicleControl.setState(State.Idle);
        
        //System.out.println("Heading Direction: " + headingDirection.toString());

        
        // Set the initial floor height to the terrain start height (maximum height of the terrain)
        //currentFloorHeight = terrain.TERRAIN_START_HEIGHT;

        // Move the vehicle
        //moveAgent();
        //move(60);

        // Create a VehicleShape object
        //VehicleShape vehShape = new VehicleShape(vehicleColor);

        // Add vehicle's shape to its TransformGroup
        //agentTG.addChild( vehShape.getVehicleTG() );

    }
    
    private float getVehicleSurfaceArea() {
        
        List<Spatial> parts = ((Node)vehicleModel).getChildren();
        
        Geometry thrusters = (Geometry)((Node)parts.get(0)).getChildren().get(0);
        Geometry body = (Geometry)((Node)parts.get(0)).getChildren().get(1);
        
        Mesh thrusMesh = thrusters.getMesh();
        Mesh bodyMesh = body.getMesh();
        
        int thrusTriCount = thrusMesh.getTriangleCount();
        int bodyTriCount = bodyMesh.getTriangleCount();
        
        float thrusArea = 0, bodyArea = 0, vehicleSurfaceArea;
        Vector3f A, B, C, P, Q;
        
        A = new Vector3f();
        B = new Vector3f();
        C = new Vector3f();

        for(int i = 0; i < thrusTriCount; ++i) {
            thrusMesh.getTriangle(i, A, B, C);
            P = B.subtract(A);
            Q = C.subtract(A);
            thrusArea += 0.5 * (P.cross(Q)).length();
            //System.out.println("P length: " + P.length());
            //System.out.println("Q length: " + Q.length());
            //System.out.println("Triangle area: " + 0.5 * (P.cross(Q)).length());
        }
        
        for(int i = 0; i < bodyTriCount; ++i) {
            bodyMesh.getTriangle(i, A, B, C);
            P = B.subtract(A);
            Q = C.subtract(A);
            bodyArea += 0.5 * (P.cross(Q)).length();
        }
        
        vehicleSurfaceArea = bodyArea + thrusArea;
        
        // Because we scaled the spatial's size down by 0.5, it would be fair to 
        // scale down its area too
        vehicleSurfaceArea *= 0.5f;
        
        System.out.println("AUV surface area: " + vehicleSurfaceArea); 
        
        return vehicleSurfaceArea;
    }
    
    public float getSurfaceArea() {
        return VEHICLE_SURFACE_AREA;
    }
    
    @Override
    public Spatial getModel() {
        return this.vehicleModel;
    }

    public float getVolume() {
        return vehicleVolume;
    }
    
    public Quaternion getHeading() {
        return this.heading;
    }
    
    public void setName(String name) {
        vehicleModel.setName(name); 
    }
    
    public Vector3f getInContainerVehPos() {
        return vehicleControl.getInContainerVehPos();
    }
    
    //@TODO: get sensors and iterate over them ... 
    public float getSensorReading() {
        return particleSensor.getReading();
    }
    
    public void setHeading(Quaternion newHeading) {
        this.heading = newHeading;
    }
    
    public void adjustHeading(Vector3f newHeading) {
        rootNode.getChild("VehicleModel").lookAt(newHeading, Vector3f.UNIT_Y);
    }
    
    public Vector3f getHeadingDir() {
        return this.headingDirection;
    }
    
    public void setHeadingDir(Vector3f headingDir) {
        this.headingDirection = headingDir;
    } 
    
    public void setMass(float mass) {
        vehicleControl.getPhysicsVehicle().setMass(mass);
    }

    public float getMass() {
        return VEHICLE_MASS;
    }
    
    public Vector3f getLinearVelocity() {
        return vehicleControl.getPhysicsVehicle().getLinearVelocity();
    }
    
    public void rotate(Quaternion rot){
        vehicleControl.getPhysicsVehicle().setPhysicsRotation(rot);
    }
   
    public void move(float tpf) {
        vehicleControl.move(tpf);
    }

    public void turn(String turnDirection, float targetTurnAngle, float tpf) {
        vehicleControl.turn(turnDirection, targetTurnAngle, tpf);
    }
    
//    public void directionTurn(Vector3f targetRelativeDir, float tpf) {
//        vehicleControl.directionTurn(targetRelativeDir, tpf);
//    }
    
    public Vector3f getPos() {
        return vehicleControl.getPhysicsVehicle().getPhysicsLocation();
    }

    public Vector3f getVel() {
        //return physicsVehicle.getLinearVelocity();
        return this.velocity;
    }
    
    public void setVel(Vector3f velocity) {
        this.velocity = velocity;
    }
    
    public void setVelX(float xVel) {
        this.velocity.setX(xVel);
    }
    
    public void setVelY(float yVel) {
        this.velocity.setY(yVel);
    }
    
    public void setVelZ(float zVel) {
        this.velocity.setZ(zVel);
    }
    
    public void setNewVel(Vector3f newVelocity) {
        this.newVelocity.set(newVelocity);
    }
    
    public float getMaxSpeed() {
        return this.maxSpeed;
    }
    
    public void pushUp(Vector3f force) {
        vehicleControl.getPhysicsVehicle().applyCentralForce(force);
    }
    
    public UWVehicleControl getControl() {
        return vehicleControl;
    }
    
//    public AdaptivePSOControl getControl() {
//        return vehicleControl;
//    }
    
    public void setPos(Vector3f position) {
        vehicleControl.getPhysicsVehicle().setPhysicsLocation(position);
    }
    
    public final void addSensor(Sensor sensor) {
        agentModel.addControl(sensor.getControl());
        bulletAppState.getPhysicsSpace().add(sensor.getControl()); 
    }
    
    public final void addModem(Modem modem) {
        agentModel.addControl(modem.getControl());
        bulletAppState.getPhysicsSpace().add(modem.getControl()); 
    }
    
    private void addControl(Control control) {
        vehicleModel.addControl(control);
    }
    public void setVehicleTargets(List<Spatial> targets) {
        vehicleControl.setVehicleTargets(targets);
    }
    
    public void setAuxSearchTargets(List<Spatial> auxSearchTargets) {
        vehicleControl.setAuxSearchTargets(auxSearchTargets);
    }

    public void setUltimateSearchTargets(List<Spatial> ultimateSearchTargets) {
        vehicleControl.setUltimateSearchTargets(ultimateSearchTargets);
    }
    
//    private void initPos() {
//        physicsVehicle.setPhysicsLocation(physicsVehicle.initialPosition);
//    }
        
        
        //	@Override
//	protected Vector3f avoidObstacles() {
//
//		// Reset the obstacle avoidance velocity
//		obstAvoidanceVel.set(0.0f, 0.0f, 0.0f);
//		
//		// Update vehicle's vicinity (center of the BoundingSphere) 
//		// based on its current position
//		agentVicinity.setCenter( new Point3d((double)agentPos.x, (double)agentPos.y, (double)agentPos.z) );
//		
//		// Check of vehicle's vicinity overlaps/intersects with any of the obstacles
//		// if so, set the avoidance velocity to the negative of the xz-velocity, weighted 
//		// by a random number and then scale it with a factor to reduce the distance moved
//		// away from the obstacle
//		if( obstacles.isOverlapping(agentVicinity) ) {
//			obstAvoidanceVel.set( -(float)(Math.random() * agentPos.x), 0.0f, 
//								  -(float)(Math.random() * agentPos.z));
//			
//			obstAvoidanceVel.scale(AVOID_WEIGHT);
//		}
//		
//		return obstAvoidanceVel;
//	}

//	@Override
//	protected void doVelocityRules() {
//		
//		// Calculate cohesion, separation, and alignment velocities
//		Vector3f v1 = physicsAgent.cohesion(agentPos);
//		Vector3f v2 = physicsAgent.separation(agentPos);
//		Vector3f v3 = physicsAgent.alignment(agentPos, agentVel);
//		
//		// Add them to the velocity changes ArrayList
//		velChanges.add(v1);
//		velChanges.add(v2);
//		velChanges.add(v3);		
//	}

}
