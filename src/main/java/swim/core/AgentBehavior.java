package swim.core;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.control.RigidBodyControl;
import java.util.Enumeration;

import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;

/**
 *
 * @author Sherif
 */
public class AgentBehavior extends RigidBodyControl implements PhysicsTickListener {

    private Spatial agent;
    
    public AgentBehavior(Spatial agent, float mass) {
        super(mass);
        this.agent = agent;
    }
    
    public void prePhysicsTick(PhysicsSpace space, float tpf) {
        // apply state changes ...
        agent.move(new Vector3f(10f, 0f, 10f));
    }

    public void physicsTick(PhysicsSpace space, float tpf) {
        // poll game state ...
    }
    
}
