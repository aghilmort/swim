/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import java.util.Iterator;
import java.util.List;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public class ParticleSensor extends Sensor {

    public ParticleSensor(Simulator sim) {
        super(sim);
    }
    
    public ParticleSensor(CollisionShape sensingPattern, float sensingRange, Simulator sim) {
        super(sensingPattern, sensingRange, sim);
    }
    
    @Override
    public SensorType getType() {
        return SensorType.ParticleSensor;
    }

    @Override
    public float getReading() {
        
        if( ((GhostControl/*SensorControl*/)this.getControl()).getOverlappingCount() > 0 ) {
            
            List overlappingObjs = ((GhostControl/*SensorControl*/)this.getControl()).getOverlappingObjects();
            
            Iterator overlappingObjIter = overlappingObjs.iterator();
            
            PhysicsCollisionObject collObj;
            
            boolean isParticle, isTarget;
            
            this.reading = 0;
            
            while(overlappingObjIter.hasNext()) {
                collObj = (PhysicsCollisionObject)overlappingObjIter.next();
        
                isParticle = collObj.getUserObject().toString().matches("(particle\\_){1}(\\d+)(\\s\\(Node\\)){1}");
                
                //System.out.println("Object Name: " + collObj.getUserObject().toString());
                
                isTarget = collObj.getUserObject().toString().matches("(ShipWreck){1}(\\s\\(Node\\)){1}");
                //System.out.println(isParticle);
                
                if( isParticle ) {
                    reading++;
                }
                
                if( isTarget ) {
                    reading = Integer.MAX_VALUE;
                }
            }
            
        } else {
           this.reading = 0; 
        }
        
        return this.reading;
    }
    
}
