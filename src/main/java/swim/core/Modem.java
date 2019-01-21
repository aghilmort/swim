/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.scene.control.Control;

/**
 *
 * @author Sherif
 */
public abstract class Modem {
    
    protected GhostControl modemControl;
    protected CollisionShape commPattern;
    protected float commRange;
    protected float signal;
    
    protected static final float DEFAULT_COMM_RANGE = 5;
    
    public Modem() {   
        this(new SphereCollisionShape(DEFAULT_COMM_RANGE), DEFAULT_COMM_RANGE);
    }
    
    public Modem(CollisionShape commPattern, float commRange) {
        this.commRange = commRange;
        this.commPattern = commPattern;
        
        if( commPattern instanceof SphereCollisionShape  ) {

            float radius = ((SphereCollisionShape)commPattern).getRadius();
            if( radius <= 0 ) {
                this.commPattern = new SphereCollisionShape(commRange);
            }
        }
        
        modemControl = new GhostControl(this.commPattern);
    }
    
    public Control getControl() {
        return this.modemControl;
    }
    
    public void setCommPattern(CollisionShape commPattern) {
        this.commPattern = commPattern;
    }
    
    public void setCommRange(float range) {
        this.commRange = range;
    }
    
    public abstract ModemType getType();
    
    public abstract float getSignal();

}
    