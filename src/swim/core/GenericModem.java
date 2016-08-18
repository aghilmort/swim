/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.GhostControl;
import java.util.Iterator;
import java.util.List;

/**
 *
 * @author Sherif
 */
public class GenericModem extends Modem {

    public GenericModem() {
        super();
    }
    
    public GenericModem(CollisionShape commPattern, float commRange) {
        super(commPattern, commRange);
    }
    
    @Override
    public ModemType getType() {
        return ModemType.GenericModem;
    }

    @Override
    public float getSignal() {
        
        if( ((GhostControl)this.getControl()).getOverlappingCount() > 0 ) {
            
            List overlappingObjs = ((GhostControl)this.getControl()).getOverlappingObjects();
            
            Iterator overlappingObjIter = overlappingObjs.iterator();
            
            PhysicsCollisionObject collObj;
            
            boolean isVehicle, isObstacle;
            
            this.signal = 0;
            
            while(overlappingObjIter.hasNext()) {
                collObj = (PhysicsCollisionObject)overlappingObjIter.next();
        
                isVehicle = collObj.getUserObject().toString().matches("(VehicleModel){1}(\\s\\(Node\\)){1}");
                
                System.out.println("Object Name: " + collObj.getUserObject().toString());
                 
                isObstacle = collObj.getUserObject().toString().matches("(New\\sScene){1}(\\s\\(Node\\)){1}");
                //System.out.println(isParticle);
                
                if( isVehicle ) {
                    signal++;
                }
                
                if( isObstacle ) {
                    signal = 0;
                }
            }
            
        } else {
           this.signal = 0; 
        }
        
        return this.signal;
    }
        
}
