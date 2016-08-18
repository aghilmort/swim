package swim.core;

import com.jme3.bounding.BoundingSphere;
import com.jme3.bullet.BulletAppState;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;

import java.util.ArrayList;

/**
 *
 * @author Sherif
 */
public abstract class Agent {
    
    protected BoundingSphere vicinity;
    
    protected Vector3f position = new Vector3f();
    protected Vector3f velocity = new Vector3f();
    protected Quaternion heading = new Quaternion();
    protected Vector3f headingDirection = new Vector3f();
    protected Vector3f newVelocity = new Vector3f();	// Used for consecutive calculations
    
    protected float maxSpeed;
	
    protected ArrayList<Vector3f> velChanges = new ArrayList<Vector3f>(); 
    
    //protected AgentBehavior agentBeh;
    
    protected Spatial agentModel;
    
    protected Node rootNode;
    
    public Agent(Node rootNode){
        
        this.rootNode = rootNode;
        
    }
    
    protected void setModel(Spatial agentModel) {
        this.agentModel = agentModel;
        
        // Add agent to the root node
        rootNode.attachChild(agentModel);
    }
    
    protected abstract void move(float tpf);

    protected abstract Vector3f getPos();

    protected abstract Vector3f getVel();
    
    public Spatial getModel() {
        return this.agentModel;
    }
    
    public String getName() {
        return this.agentModel.getName();
    }

    //protected abstract Vector3f avoidObstacles();

    //protected abstract void doVelocityRules();
   
}
