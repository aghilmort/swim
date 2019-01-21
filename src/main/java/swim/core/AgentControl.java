/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.control.AbstractControl;
import java.util.ArrayList;

/**
 *
 * @author Sherif
 */
public abstract class AgentControl extends AbstractControl /*implements PhysicsTickListener*/ {

    private float speed = 3;
    
    protected ArrayList<Vector3f> velChanges = new ArrayList<Vector3f>(); 
    
    public AgentControl() {}
    
    protected abstract float limitMaxSpeed();
    
    protected abstract void move(float tpf);
    
//    public void prePhysicsTick(PhysicsSpace space, float tpf) {}
//
//    public void physicsTick(PhysicsSpace space, float tpf) {}
    
    /**
     * @return the speed
     */
    public float getSpeed() {
        return speed;
    }

    /**
     * @param speed the speed to set
     */
    public void setSpeed(float speed) {
        this.speed = speed;
    }
    
    protected abstract Vector3f calcNewVel();
    
}
