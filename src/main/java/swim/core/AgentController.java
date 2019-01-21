/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.control.AbstractControl;

/**
 *
 * @author Sherif
 */
public class AgentController extends AbstractControl {

    private RigidBodyControl physicsAgent;
    private boolean forward, backwards;
    private Vector3f moveDirection = new Vector3f(0, 0, 0), viewDirection;
    private Vector3f agentForwardDir;
    private State state;
    private PhysicsSpace physicsSpace;
    
    
    
    public AgentController(PhysicsSpace physicsSpace) {
        this.physicsSpace = physicsSpace;
        
    }
    
    @Override
    protected void controlUpdate(float tpf) {
        
//        agentForwardDir = spatial.getWorldRotation().mult(Vector3f.UNIT_X);
//        
//        switch(state) {
//            case idle:
//                forward = false;
//                backwards = false;
//                break;
//            case move:
//                forward = true;
//                backwards = false;
//        }
//        
//        moveDirection.set(0, 0, 0);
//        if(forward) {
//            moveDirection.addLocal(agentForwardDir.mult(20f));
//            System.out.println(moveDirection.toString());
//        }
//
//        //physicsAgent.setLinearVelocity(new Vector3f(0,0,tpf*3f));
//        
//        Vector3f currLocation = physicsAgent.getPhysicsLocation();
//        physicsAgent.setPhysicsLocation(currLocation.add(new Vector3f(-tpf*3,0,0)));
//        //physicsAgent.setPhysicsLocation(moveDirection);
       
    }

    public void setState(State state) {
        this.state = state;
    }
    
    
    public void setPhysicsAgent(RigidBodyControl physicsAgent) {
        this.physicsAgent = physicsAgent;
    }
    
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }
    
}
