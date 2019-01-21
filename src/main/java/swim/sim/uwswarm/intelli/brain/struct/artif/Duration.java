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
public enum Duration {
    
    UNTIL_INTERRUPTED,
    
    RANDOM,
    PRESET,
    INDEFINITE,
    TASK_CAP,
    
    DUR_OF_SEQUENCE,
    
    TIME_TO_SENSE_TARGETS,
    TIME_TO_REACH_TARGETS,
    
    TIME_TO_SENSE_VEHICLES,
    TIME_TO_REACH_VEHICLES,
    
    TIME_TO_ESCAPE_NEIGHBORS,
    
    TIME_TO_SATISFY_GOAL,
    
    WHILE_TARGETS_BELOW_THRES,
    WHILE_REWARD_LVL_PERSISTS,
    
    OF_BEST_REWARDED_ACTION
    
}
