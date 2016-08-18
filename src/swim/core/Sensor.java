/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.scene.control.Control;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public abstract class Sensor {
    
    //protected SensorControl sensorControl;
    protected GhostControl sensorControl;
    protected CollisionShape sensingPattern;
    protected float sensingRange;
    protected float reading;
    
    protected static final float DEFAULT_SENSING_RANGE = 5;
    
    public Sensor(Simulator sim) {   
        this(new SphereCollisionShape(DEFAULT_SENSING_RANGE), DEFAULT_SENSING_RANGE, sim);
    }
    
    public Sensor(CollisionShape sensingPattern, float sensingRange, Simulator sim) {
        this.sensingRange = sensingRange;
        this.sensingPattern = sensingPattern;
        
        if( sensingPattern instanceof SphereCollisionShape  ) {

            float radius = ((SphereCollisionShape)sensingPattern).getRadius();
            if( radius <= 0 ) {
                this.sensingPattern = new SphereCollisionShape(sensingRange);
            }
        }
        
//        sensorControl = new SensorControl(this.sensingPattern, sim);
        sensorControl = new GhostControl(this.sensingPattern);
    }
    
    public Control getControl() {
        return this.sensorControl;
    }
    
    public void setSensingPattern(CollisionShape sensingPattern) {
        this.sensingPattern = sensingPattern;
    }
    
    public void setSensingRange(float range) {
        this.sensingRange = range;
    }
    
    public abstract SensorType getType();
    
    public abstract float getReading();
    
}
