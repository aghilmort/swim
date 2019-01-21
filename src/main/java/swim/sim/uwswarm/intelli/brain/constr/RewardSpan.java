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
public enum RewardSpan {
    LONG_TERM    (5000),
    SHORT_TERM   (10);
    
    private final int length;
    
    RewardSpan(int length) {
       this.length = length;
    }
   
    public int length() {
       return length;
    }
}
