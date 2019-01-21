/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.constr;

/**
 *
 * @author Sherif
 */
public enum LongTermRewardType {
    FOUND_TARGET_LOCATION           (500),
    SATISFIED_TIME_CONSTRAINT       (500),
    PROCESSED_N_TARGETS             (300),
    FLOATED_TO_SURFACE              (200),
    MAINTAINED_LONG_TERM_COHESION   (400);
    
    private final int value;
    
    LongTermRewardType(int value) {
        this.value = value;
    }
    
    public int value() {
        return value;
    }
}
