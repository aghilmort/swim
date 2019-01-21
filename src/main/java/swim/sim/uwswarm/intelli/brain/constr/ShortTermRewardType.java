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
public enum ShortTermRewardType {
    ESCAPED_CROWD                   (5),
    FLOCKED                         (10),
    FOUND_NEXT_TARGET               (20),
    ATTRACTED_OTHER_VEHS_TO_TARG    (30),
    IDENTIFIED_AND_PERFORMED_TASK   (50),
    ACTED_CREATIVELY                (40),
    FOUND_NEIGHBORS                 (30),
    SAVED_ENERGY                    (15);
    
    private final int value;
    
    ShortTermRewardType(int value) {
       this.value = value;
    }
   
    public int value() {
       return value;
    }
}
