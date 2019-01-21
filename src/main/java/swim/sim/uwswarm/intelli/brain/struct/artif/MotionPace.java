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
public enum MotionPace {
    
    PROP_TO_LAST_NEIGH_ENCOUNTER_TIME,
    AVERAGE_OF_NEIGHBORS,
    
    FAST,
    SLOW,
    MEDIUM,
    
    FAST_THEN_SLOW,
    SLOW_THEN_FAST,
    
    SAME,
    
    RANDOM,
    RANDOM_AMONG_PACES,
    
    OF_BEST_REWARDED_ACTION,
    
    INITIAL     // at the very beginning
    
}
