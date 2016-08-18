/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.core;

import com.jme3.bullet.control.RigidBodyControl;
import java.util.Random;

/**
 *
 * @author Sherif
 */
public class ParticleControl extends RigidBodyControl {
    
    private String particleName = "particle";
    private long particleId;
    
    public ParticleControl() {
        super();
        Random rng = new Random();
        particleId = rng.nextLong();
    }
    
    public ParticleControl(float mass) {
        super(mass);
        Random rng = new Random();
        particleId = rng.nextLong();
    }
    
    public void setName(String particleName) {
        this.particleName = particleName;
    }
    
    public void setId(long particleId) {
        this.particleId = particleId;
    }
    
    public String getName() {
        return this.particleName;
    }
    
    public long getId() {
        return this.particleId;
    }

}
