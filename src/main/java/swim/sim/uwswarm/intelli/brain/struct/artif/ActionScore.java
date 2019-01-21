/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

/**
 *
 * @author Sherif
 */
public enum ActionScore {
    VERY_LOW    (1),
    LOW         (2),
    MEDIUM      (3),
    HIGH        (4),   
    VERY_HIGH   (5);
    
    private float value;
    
    ActionScore(float value) {
        this.value = value;
    }
    
    public float value() {
        return value;
    }
}
